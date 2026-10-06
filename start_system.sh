#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_LAUNCHER="${SCRIPT_DIR}/current/start_system.sh"

# The copy in /home/raspberry/arUco_termit is the stable entry point. The
# copy inside a release performs the actual launch.
if [[ -x "${CURRENT_LAUNCHER}" ]] && [[ "$(readlink -f "${CURRENT_LAUNCHER}")" != "${SCRIPT_PATH}" ]]; then
    exec "${CURRENT_LAUNCHER}" "$@"
fi

export PYTHONPATH="${SCRIPT_DIR}:${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher:${PYTHONPATH:-}"
# Prefer the locally built libcamera 0.7 GStreamer plugin. Ubuntu's packaged
# 0.2 plugin cannot drive the Raspberry Pi 5 camera stack installed here.
export GST_PLUGIN_PATH="/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0:${GST_PLUGIN_PATH:-}"
if [[ -f "${SCRIPT_DIR}/manifest.json" ]]; then
    export ROBOT_RELEASE_COMMIT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("git_commit", "unknown"))' "${SCRIPT_DIR}/manifest.json")"
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
    echo "[+] Sourcing isolated release workspace: ${SCRIPT_DIR}/install/setup.bash"
    source "${SCRIPT_DIR}/install/setup.bash"
elif [[ -f /home/raspberry/ros2_ws/install/setup.bash ]]; then
    echo "[+] Sourcing global ros2_ws: /home/raspberry/ros2_ws/install/setup.bash"
    source /home/raspberry/ros2_ws/install/setup.bash
fi

echo "[+] Gracefully stopping previous instances..."
curl -s -m 1 http://localhost:8080/test_drive?type=stop >/dev/null 2>&1 || true
sleep 0.2

pkill -15 -f 'localization_node' 2>/dev/null || true
pkill -15 -f 'video_tag_detector' 2>/dev/null || true
pkill -15 -f 'path_follower' 2>/dev/null || true
pkill -15 -f 'esp32_bridge' 2>/dev/null || true
pkill -15 -f 'static_transform_publisher' 2>/dev/null || true

for _ in {1..20}; do
    if ! pgrep -f 'localization_node|video_tag_detector' >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done

pkill -9 -f 'localization_node' 2>/dev/null || true
pkill -9 -f 'video_tag_detector' 2>/dev/null || true
pkill -9 -f 'path_follower' 2>/dev/null || true
pkill -9 -f 'esp32_bridge' 2>/dev/null || true
pkill -9 -f 'static_transform_publisher' 2>/dev/null || true
sleep 0.5

if [[ -f "${SCRIPT_DIR}/localization_node.py" ]]; then
    LOC_PY="${SCRIPT_DIR}/localization_node.py"
elif [[ -f "${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher/localization_node.py" ]]; then
    LOC_PY="${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher/localization_node.py"
else
    LOC_PY="/home/raspberry/ros2_ws/src/fake_tag_publisher/fake_tag_publisher/localization_node.py"
fi

if [[ -f "${SCRIPT_DIR}/video_tag_detector.py" ]]; then
    VID_PY="${SCRIPT_DIR}/video_tag_detector.py"
elif [[ -f "${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher/video_tag_detector.py" ]]; then
    VID_PY="${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher/video_tag_detector.py"
else
    VID_PY="/home/raspberry/ros2_ws/src/fake_tag_publisher/fake_tag_publisher/video_tag_detector.py"
fi

echo "[+] Active release directory: ${SCRIPT_DIR}"
echo "[+] Using localization_node:  ${LOC_PY}"
echo "[+] Using video_tag_detector: ${VID_PY}"
echo "[+] Dynamic Camera TF (base_link -> camera_link) is broadcast by localization_node from camera_extrinsics.yaml"

start_video_detector() {
    echo "[+] Starting Video Tag Detector (native libcamera GStreamer)..."
    python3 "${VID_PY}" --ros-args -p video_path:=0 -p aruco_dictionary:=DICT_4X4_100 > /tmp/video_tag_detector.log 2>&1 &
    PID_VID=$!
    # Camera initialization and DDS discovery can take several seconds. Do not
    # mistake that startup interval for another frozen stream.
    CAMERA_GRACE_UNTIL=$((SECONDS + 15))
}

stop_video_detector() {
    if [[ -n "${PID_VID:-}" ]] && kill -0 "${PID_VID}" 2>/dev/null; then
        kill -15 "${PID_VID}" 2>/dev/null || true
        # Let libcamera release the CSI device before escalating to SIGKILL.
        # A rushed hard kill can leave the pipeline wedged until both robot
        # processes are restarted.
        for _ in {1..30}; do
            kill -0 "${PID_VID}" 2>/dev/null || break
            sleep 0.1
        done
        kill -9 "${PID_VID}" 2>/dev/null || true
        wait "${PID_VID}" 2>/dev/null || true
    fi
}

restart_video_detector() {
    echo "[!] Camera watchdog: no new JPEG frames; restarting detector..."
    stop_video_detector
    sleep 0.5
    start_video_detector
}

shutdown_children() {
    trap - TERM INT
    set +e
    curl -s -m 1 http://localhost:8080/test_drive?type=stop >/dev/null 2>&1 || true
    stop_video_detector
    if [[ -n "${PID_LOC:-}" ]] && kill -0 "${PID_LOC}" 2>/dev/null; then
        kill -15 "${PID_LOC}" 2>/dev/null || true
        for _ in {1..30}; do
            kill -0 "${PID_LOC}" 2>/dev/null || break
            sleep 0.1
        done
        kill -9 "${PID_LOC}" 2>/dev/null || true
        wait "${PID_LOC}" 2>/dev/null || true
    fi
}

handle_shutdown() {
    echo "[+] Stopping TERMiT processes cleanly..."
    shutdown_children
    exit 0
}

request_full_restart() {
    echo "[!] Camera watchdog: detector restarts did not recover frames; restarting the complete service..."
    shutdown_children
    # termit.service uses Restart=on-failure. Leaving with an error recreates
    # both the camera pipeline and localization node from a clean state.
    exit 1
}

trap handle_shutdown TERM INT

start_video_detector

echo "[+] Starting Localization & Autopilot Web Server (port 8080)..."
python3 "${LOC_PY}" > /tmp/localization_node.log 2>&1 &
PID_LOC=$!

IP_LIST="$(hostname -I | tr ' ' '\n' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | tr '\n' ' ' || true)"
PRIMARY_IP="$(echo "${IP_LIST}" | awk '{print $1}')"
if [[ -z "${PRIMARY_IP}" ]]; then PRIMARY_IP="172.20.10.2"; fi

echo "============================================================="
echo "TERMiT SYSTEM RUNNING (PID: ${PID_LOC})"
echo "Web Interface: http://${PRIMARY_IP}:8080/"
echo "Video Stream:  http://${PRIMARY_IP}:8080/video_feed"
echo "All available IPs: ${IP_LIST}"
echo "============================================================="

STALE_CAMERA_POLLS=0
CAMERA_RECOVERY_ATTEMPTS=0
while kill -0 "${PID_LOC}" 2>/dev/null; do
    sleep 2
    if ! kill -0 "${PID_VID}" 2>/dev/null; then
        restart_video_detector
        CAMERA_RECOVERY_ATTEMPTS=$((CAMERA_RECOVERY_ATTEMPTS + 1))
        STALE_CAMERA_POLLS=0
        continue
    fi
    if (( SECONDS < CAMERA_GRACE_UNTIL )); then
        STALE_CAMERA_POLLS=0
        continue
    fi

    CAMERA_STATE="$(curl -s -m 1 http://localhost:8080/api/health 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
    age = data.get("camera_frame_age_s")
    print("stale" if age is None or float(age) > 4.0 else "fresh")
except Exception:
    print("unknown")
' 2>/dev/null || true)"
    if [[ "${CAMERA_STATE}" == "stale" ]]; then
        STALE_CAMERA_POLLS=$((STALE_CAMERA_POLLS + 1))
    elif [[ "${CAMERA_STATE}" == "fresh" ]]; then
        STALE_CAMERA_POLLS=0
        CAMERA_RECOVERY_ATTEMPTS=0
    fi
    if (( STALE_CAMERA_POLLS >= 2 )); then
        CAMERA_RECOVERY_ATTEMPTS=$((CAMERA_RECOVERY_ATTEMPTS + 1))
        if (( CAMERA_RECOVERY_ATTEMPTS >= 3 )); then
            request_full_restart
        fi
        restart_video_detector
        STALE_CAMERA_POLLS=0
    fi
done

shutdown_children
wait "${PID_LOC}" 2>/dev/null || true
