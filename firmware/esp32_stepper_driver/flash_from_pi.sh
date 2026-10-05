#!/usr/bin/env bash
set -euo pipefail

# Flash a pre-built merged ESP32 image from Raspberry Pi. The explicit safety
# environment variable prevents this maintenance action from being started by
# an accidental click or copied command.
if [[ "${TERMIT_LASER_PHYSICALLY_SAFE:-}" != "YES" ]]; then
    echo "Refusing to flash: disconnect laser power and set TERMIT_LASER_PHYSICALLY_SAFE=YES" >&2
    exit 2
fi

IMAGE_PATH="${1:-}"
PORT="${2:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
if [[ -z "${IMAGE_PATH}" || ! -f "${IMAGE_PATH}" ]]; then
    echo "Usage: TERMIT_LASER_PHYSICALLY_SAFE=YES $0 /path/to/firmware.merged.bin [serial-port]" >&2
    exit 2
fi
if [[ ! -e "${PORT}" ]]; then
    echo "ESP32 serial port does not exist: ${PORT}" >&2
    exit 2
fi

VENV_DIR="/home/raspberry/.local/share/termit-esptool-5.3.1"
if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
    python3 -m venv "${VENV_DIR}"
    "${VENV_DIR}/bin/python" -m pip install --disable-pip-version-check "esptool==5.3.1"
fi

# The running localization service owns the UART. Stop it and leave all motion
# and optical requests at zero before entering the ROM bootloader.
curl -fsS --max-time 1 -X POST http://127.0.0.1:8080/api/laser/off >/dev/null 2>&1 || true
curl -fsS --max-time 1 'http://127.0.0.1:8080/test_drive?type=stop' >/dev/null 2>&1 || true
pkill -15 -f 'localization_node' 2>/dev/null || true
sleep 1

"${VENV_DIR}/bin/python" -m esptool \
    --chip esp32 --port "${PORT}" --baud 460800 \
    write-flash 0x0 "${IMAGE_PATH}"

echo "ESP32 firmware written successfully. Start TERMiT with /home/raspberry/arUco_termit/start_system.sh"
