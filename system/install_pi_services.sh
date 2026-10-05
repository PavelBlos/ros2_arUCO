#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
install -d -m 0755 /etc/termit
install -m 0644 "${SCRIPT_DIR}/hostapd.conf" /etc/termit/hostapd.conf
install -m 0644 "${SCRIPT_DIR}/dnsmasq.conf" /etc/termit/dnsmasq.conf
install -m 0755 "${SCRIPT_DIR}/termit-hotspot.sh" /usr/local/sbin/termit-hotspot
install -m 0644 "${SCRIPT_DIR}/termit.service" /etc/systemd/system/termit.service
install -m 0644 "${SCRIPT_DIR}/termit-hotspot.service" /etc/systemd/system/termit-hotspot.service
# The fallback script starts isolated hostapd/dnsmasq instances itself. Their
# distribution-wide services must stay disabled or they would bind DNS/Wi-Fi
# even while the Pi is connected to a normal network.
systemctl disable --now hostapd dnsmasq 2>/dev/null || true
systemctl daemon-reload
systemctl enable termit.service termit-hotspot.service

echo "TERMiT services installed. The fallback AP is TERMiT-Setup / termit2026."
