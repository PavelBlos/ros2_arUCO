#!/usr/bin/env bash
set -eo pipefail

WLAN="wlan0"
AP_ADDRESS="10.42.0.1/24"
HOSTAPD_PID="/run/termit-hostapd.pid"
DNSMASQ_PID="/run/termit-dnsmasq.pid"

wifi_is_online() {
    ip -4 address show dev "${WLAN}" | grep -q 'inet ' \
        && ip -4 route show default dev "${WLAN}" | grep -q '^default '
}

stop_hotspot() {
    if [[ -s "${DNSMASQ_PID}" ]]; then kill "$(cat "${DNSMASQ_PID}")" 2>/dev/null || true; fi
    if [[ -s "${HOSTAPD_PID}" ]]; then kill "$(cat "${HOSTAPD_PID}")" 2>/dev/null || true; fi
    rm -f "${DNSMASQ_PID}" "${HOSTAPD_PID}"
}

start_hotspot() {
    stop_hotspot
    systemctl stop netplan-wpa-wlan0.service 2>/dev/null || true
    networkctl down "${WLAN}" 2>/dev/null || true
    ip link set "${WLAN}" down || true
    ip address flush dev "${WLAN}" || true
    ip link set "${WLAN}" up
    ip address add "${AP_ADDRESS}" dev "${WLAN}"
    /usr/sbin/hostapd -B -P "${HOSTAPD_PID}" /etc/termit/hostapd.conf
    /usr/sbin/dnsmasq --conf-file=/etc/termit/dnsmasq.conf --pid-file="${DNSMASQ_PID}"
    logger -t termit-hotspot "fallback AP TERMiT-Setup started at 10.42.0.1"
}

case "${1:-wait-and-start}" in
    stop)
        stop_hotspot
        exit 0
        ;;
    wait-and-start)
        for _ in $(seq 1 45); do
            if wifi_is_online; then
                logger -t termit-hotspot "normal Wi-Fi is online; fallback AP is not needed"
                exit 0
            fi
            sleep 1
        done
        start_hotspot
        while true; do sleep 3600; done
        ;;
    *)
        echo "usage: $0 {wait-and-start|stop}" >&2
        exit 2
        ;;
esac
