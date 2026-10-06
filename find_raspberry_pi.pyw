"""Small Windows GUI for finding the TERMIT Raspberry Pi on the local network.

The utility uses only the Python standard library.  It detects active IPv4
networks, scans a deliberately small number of hosts, and recognises TERMIT by
its health endpoint.  Generic SSH/web devices are shown as well, so a changed
service configuration does not make the Raspberry Pi completely invisible.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import ipaddress
import json
import queue
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from dataclasses import asdict, dataclass
from typing import Iterable


APP_TITLE = "Поиск Raspberry Pi — TERMIT"
SCAN_PORTS = (22, 8080)
# The Tailscale address provides a useful fallback when the local address has
# changed.  Local mDNS names are intentionally not scanned here: on some phone
# hotspots a failed .local lookup stalls Windows DNS for many seconds.
KNOWN_ADDRESSES = ("100.99.95.103",)
CONNECT_TIMEOUT = 0.35
HTTP_TIMEOUT = 1.0
MAX_HOSTS_PER_NETWORK = 254


@dataclass
class Device:
    ip: str
    name: str = ""
    ssh: bool = False
    web_port: int | None = None
    kind: str = "Сетевое устройство"
    details: str = ""
    commit: str = ""

    @property
    def web_url(self) -> str:
        return f"http://{self.ip}:{self.web_port}" if self.web_port else ""


def _hidden_process_kwargs() -> dict:
    if sys.platform != "win32":
        return {}
    return {"creationflags": getattr(subprocess, "CREATE_NO_WINDOW", 0)}


def active_ipv4_interfaces() -> list[tuple[str, int, str]]:
    """Return (address, prefix length, adapter name) for active interfaces."""
    if sys.platform == "win32":
        command = (
            "Get-NetIPConfiguration | "
            "Where-Object {$_.NetAdapter.Status -eq 'Up' -and $_.IPv4Address} | "
            "ForEach-Object { [PSCustomObject]@{"
            "Name=$_.InterfaceAlias; Address=$_.IPv4Address.IPAddress; "
            "Prefix=$_.IPv4Address.PrefixLength; "
            "Gateway=$_.IPv4DefaultGateway.NextHop} } | ConvertTo-Json -Compress"
        )
        try:
            completed = subprocess.run(
                ["powershell", "-NoProfile", "-Command", command],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=8,
                check=True,
                **_hidden_process_kwargs(),
            )
            raw = json.loads(completed.stdout.strip() or "[]")
            rows = raw if isinstance(raw, list) else [raw]
            result: list[tuple[str, int, str]] = []
            for row in rows:
                addresses = row.get("Address") or []
                prefixes = row.get("Prefix") or []
                if isinstance(addresses, str):
                    addresses = [addresses]
                if isinstance(prefixes, int):
                    prefixes = [prefixes]
                for index, address in enumerate(addresses):
                    prefix = int(prefixes[index] if index < len(prefixes) else 24)
                    parsed = ipaddress.ip_address(address)
                    if parsed.version == 4 and not parsed.is_loopback:
                        result.append((address, prefix, str(row.get("Name") or "")))
            if result:
                return result
        except (OSError, ValueError, subprocess.SubprocessError):
            pass

    result = []
    try:
        for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = item[4][0]
            if not ipaddress.ip_address(address).is_loopback:
                result.append((address, 24, "Сетевой адаптер"))
    except OSError:
        pass
    return list(dict.fromkeys(result))


def scan_networks(manual: str = "") -> tuple[list[ipaddress.IPv4Network], list[str]]:
    """Choose safe scan ranges and return explanatory labels for the GUI."""
    networks: list[ipaddress.IPv4Network] = []
    labels: list[str] = []
    if manual.strip():
        for value in manual.replace(";", ",").split(","):
            value = value.strip()
            if not value:
                continue
            network = ipaddress.ip_network(value, strict=False)
            if network.version != 4:
                raise ValueError("Поддерживаются только IPv4-подсети")
            if network.num_addresses - 2 > MAX_HOSTS_PER_NETWORK:
                raise ValueError("Сеть слишком большая. Укажите подсеть /24 или меньше.")
            networks.append(network)
            labels.append(str(network))
    else:
        for address, prefix, adapter in active_ipv4_interfaces():
            ip = ipaddress.ip_address(address)
            if not ip.is_private or ip.is_loopback or ip.is_link_local:
                continue
            adapter_lower = adapter.lower()
            if any(token in adapter_lower for token in ("wsl", "tailscale", "happ-xray")):
                continue
            # VPN/WSL ranges can be huge. Scan only the local /24 containing us.
            safe_prefix = max(prefix, 24)
            network = ipaddress.ip_network(f"{address}/{safe_prefix}", strict=False)
            networks.append(network)
            labels.append(str(network))
    unique = list(dict.fromkeys(networks))
    return unique, labels


def _tcp_open(host: str, port: int) -> tuple[bool, str]:
    try:
        with socket.create_connection((host, port), timeout=CONNECT_TIMEOUT) as sock:
            if port == 22:
                sock.settimeout(CONNECT_TIMEOUT)
                try:
                    return True, sock.recv(160).decode("ascii", errors="replace").strip()
                except OSError:
                    return True, ""
            return True, ""
    except OSError:
        return False, ""


def _health(host: str, port: int = 8080) -> tuple[dict | None, str]:
    url = f"http://{host}:{port}/api/health"
    try:
        request = urllib.request.Request(url, headers={"User-Agent": "TERMIT-Finder/1.0"})
        with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT) as response:
            body = response.read(64 * 1024).decode("utf-8", errors="replace")
        parsed = json.loads(body)
        return (parsed if isinstance(parsed, dict) else None), body[:200]
    except (OSError, ValueError, urllib.error.URLError):
        return None, ""


def _reverse_name(host: str) -> str:
    try:
        return socket.gethostbyaddr(host)[0]
    except OSError:
        return ""


def probe(host: str) -> Device | None:
    """Probe one address and describe it if SSH or the control UI responds."""
    try:
        resolved = socket.gethostbyname(host)
    except OSError:
        return None

    ssh, ssh_banner = _tcp_open(resolved, 22)
    web, _ = _tcp_open(resolved, 8080)
    if not ssh and not web:
        return None

    # Some mobile hotspots accept TCP connections on every unused address and
    # then discard the traffic.  A real SSH banner or a valid health response
    # is therefore required before an address is shown as a device.
    verified_ssh = ssh and bool(ssh_banner)
    device = Device(ip=resolved, ssh=verified_ssh)
    recognised = False
    if web:
        device.web_port = 8080
        health, _ = _health(resolved)
        if health is not None:
            signature = " ".join(str(key) for key in health.keys()).lower()
            if "git_commit" in signature or "esp32_connected" in signature or "firmware_version" in signature:
                device.kind = "Робот TERMIT (Raspberry Pi)"
                recognised = True
                device.commit = str(health.get("git_commit") or "")
                esp = health.get("esp32_connected")
                status = health.get("status", "unknown")
                device.details = f"Система: {status}; ESP32: {'подключён' if esp else 'не подключён'}"
            else:
                device.kind = "Веб-устройство"
    if device.kind == "Сетевое устройство" and verified_ssh:
        recognised = True
        lowered = f"{device.name} {host}".lower()
        if "raspberry" in lowered or "termit" in lowered:
            device.kind = "Возможный Raspberry Pi"
        device.details = ssh_banner or "SSH доступен"
    if recognised and not device.kind.startswith("Робот TERMIT"):
        device.name = _reverse_name(resolved)
        if "raspberry" in device.name.lower() or "termit" in device.name.lower():
            device.kind = "Возможный Raspberry Pi"
    if recognised:
        return device
    return None


def address_candidates(networks: Iterable[ipaddress.IPv4Network]) -> list[str]:
    addresses = [str(host) for network in networks for host in network.hosts()]
    addresses.extend(KNOWN_ADDRESSES)
    return list(dict.fromkeys(addresses))


def discover(manual_networks: str = "", progress=None) -> tuple[list[Device], list[str]]:
    networks, labels = scan_networks(manual_networks)
    candidates = address_candidates(networks)
    found: dict[str, Device] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=48) as executor:
        futures = {executor.submit(probe, host): host for host in candidates}
        total = len(futures)
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            try:
                device = future.result()
                if device:
                    previous = found.get(device.ip)
                    if previous is None or device.kind.startswith("Робот TERMIT"):
                        found[device.ip] = device
            except Exception:
                pass
            if progress:
                progress(index, total)
    devices = sorted(
        found.values(),
        key=lambda item: (not item.kind.startswith("Робот TERMIT"), ipaddress.ip_address(item.ip)),
    )
    return devices, labels


def run_cli(networks: str) -> int:
    devices, labels = discover(networks)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print(json.dumps({"networks": labels, "devices": [asdict(d) for d in devices]}, ensure_ascii=False, indent=2))
    return 0 if devices else 1


def run_gui() -> None:
    import tkinter as tk
    from tkinter import messagebox, ttk

    class FinderApp:
        def __init__(self, root: tk.Tk):
            self.root = root
            self.root.title(APP_TITLE)
            self.root.geometry("850x500")
            self.root.minsize(720, 420)
            self.events: queue.Queue = queue.Queue()
            self.devices: dict[str, Device] = {}

            outer = ttk.Frame(root, padding=14)
            outer.pack(fill="both", expand=True)
            ttk.Label(outer, text="Поиск робота в Wi-Fi", font=("Segoe UI", 17, "bold")).pack(anchor="w")
            ttk.Label(
                outer,
                text="Подсеть определяется автоматически. При необходимости можно указать её вручную, например 172.20.10.0/28.",
            ).pack(anchor="w", pady=(3, 12))

            controls = ttk.Frame(outer)
            controls.pack(fill="x")
            ttk.Label(controls, text="Подсеть:").pack(side="left")
            self.network_var = tk.StringVar()
            self.network_entry = ttk.Entry(controls, textvariable=self.network_var, width=28)
            self.network_entry.pack(side="left", padx=(7, 10))
            self.scan_button = ttk.Button(controls, text="Найти Raspberry Pi", command=self.start_scan)
            self.scan_button.pack(side="left")

            self.status_var = tk.StringVar(value="Нажмите «Найти Raspberry Pi».")
            ttk.Label(outer, textvariable=self.status_var).pack(anchor="w", pady=(12, 5))
            self.progress = ttk.Progressbar(outer, mode="determinate")
            self.progress.pack(fill="x", pady=(0, 10))

            columns = ("ip", "kind", "ssh", "web", "commit", "details")
            self.table = ttk.Treeview(outer, columns=columns, show="headings", height=11)
            headings = {
                "ip": "IP-адрес", "kind": "Что найдено", "ssh": "SSH",
                "web": "Веб-пульт", "commit": "Версия", "details": "Состояние",
            }
            widths = {"ip": 120, "kind": 190, "ssh": 55, "web": 80, "commit": 75, "details": 250}
            for column in columns:
                self.table.heading(column, text=headings[column])
                self.table.column(column, width=widths[column], anchor="w")
            self.table.pack(fill="both", expand=True)
            self.table.bind("<Double-1>", lambda _event: self.open_web())

            buttons = ttk.Frame(outer)
            buttons.pack(fill="x", pady=(10, 0))
            ttk.Button(buttons, text="Открыть веб-пульт", command=self.open_web).pack(side="left")
            ttk.Button(buttons, text="Копировать IP", command=self.copy_ip).pack(side="left", padx=8)
            ttk.Button(buttons, text="Повторить поиск", command=self.start_scan).pack(side="left")
            self.root.after(100, self.process_events)
            self.root.after(250, self.start_scan)

        def start_scan(self):
            if str(self.scan_button["state"]) == "disabled":
                return
            self.scan_button.configure(state="disabled")
            self.devices.clear()
            for row in self.table.get_children():
                self.table.delete(row)
            self.progress["value"] = 0
            self.status_var.set("Определяю сеть и ищу робота…")
            manual_networks = self.network_var.get()
            threading.Thread(target=self.worker, args=(manual_networks,), daemon=True).start()

        def worker(self, manual_networks):
            started = time.monotonic()
            try:
                devices, labels = discover(
                    manual_networks,
                    progress=lambda done, total: self.events.put(("progress", done, total)),
                )
                self.events.put(("done", devices, labels, time.monotonic() - started))
            except Exception as error:
                self.events.put(("error", str(error)))

        def process_events(self):
            try:
                while True:
                    event = self.events.get_nowait()
                    if event[0] == "progress":
                        _, done, total = event
                        self.progress["maximum"] = max(total, 1)
                        self.progress["value"] = done
                        self.status_var.set(f"Проверено адресов: {done} из {total}")
                    elif event[0] == "done":
                        _, devices, labels, elapsed = event
                        self.show_results(devices, labels, elapsed)
                    elif event[0] == "error":
                        self.scan_button.configure(state="normal")
                        self.status_var.set("Поиск завершился с ошибкой.")
                        messagebox.showerror(APP_TITLE, event[1])
            except queue.Empty:
                pass
            self.root.after(100, self.process_events)

        def show_results(self, devices, labels, elapsed):
            self.scan_button.configure(state="normal")
            for device in devices:
                self.devices[device.ip] = device
                self.table.insert(
                    "", "end", iid=device.ip,
                    values=(
                        device.ip, device.kind, "Да" if device.ssh else "Нет",
                        "Да" if device.web_port else "Нет", device.commit, device.details,
                    ),
                )
            robot = next((d for d in devices if d.kind.startswith("Робот TERMIT")), None)
            network_text = ", ".join(labels) or "адреса из списка"
            if robot:
                self.table.selection_set(robot.ip)
                self.table.focus(robot.ip)
                self.status_var.set(f"Робот найден: {robot.ip}  •  сеть {network_text}  •  {elapsed:.1f} с")
            elif devices:
                self.status_var.set(f"Робот TERMIT не распознан; показаны другие устройства. Сеть: {network_text}")
            else:
                self.status_var.set(f"Робот не найден. Проверена сеть: {network_text}")

        def selected(self) -> Device | None:
            selection = self.table.selection()
            return self.devices.get(selection[0]) if selection else None

        def open_web(self):
            device = self.selected()
            if not device:
                messagebox.showinfo(APP_TITLE, "Сначала выберите найденное устройство.")
            elif not device.web_url:
                messagebox.showinfo(APP_TITLE, "У этого устройства веб-пульт не обнаружен.")
            else:
                webbrowser.open(device.web_url)

        def copy_ip(self):
            device = self.selected()
            if not device:
                messagebox.showinfo(APP_TITLE, "Сначала выберите найденное устройство.")
                return
            self.root.clipboard_clear()
            self.root.clipboard_append(device.ip)
            self.status_var.set(f"IP {device.ip} скопирован в буфер обмена.")

    root = tk.Tk()
    try:
        ttk.Style(root).theme_use("vista")
    except Exception:
        pass
    FinderApp(root)
    root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Find a TERMIT Raspberry Pi on the local network")
    parser.add_argument("--scan", action="store_true", help="print JSON results instead of opening the GUI")
    parser.add_argument("--network", default="", help="CIDR network, for example 172.20.10.0/28")
    args = parser.parse_args()
    if args.scan:
        return run_cli(args.network)
    run_gui()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
