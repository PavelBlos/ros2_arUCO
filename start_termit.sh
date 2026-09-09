#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${SCRIPT_DIR}:${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher:${PYTHONPATH}"

# Source ROS 2 base environment
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source isolated release install if present, otherwise fallback to global ros2_ws
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "[+] Sourcing isolated release workspace: ${SCRIPT_DIR}/install/setup.bash"
    source "${SCRIPT_DIR}/install/setup.bash"
elif [ -f /home/raspberry/ros2_ws/install/setup.bash ]; then
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

for i in {1..20}; do
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

# Determine active python script locations
if [ -f "${SCRIPT_DIR}/localization_node.py" ]; then
    LOC_PY="${SCRIPT_DIR}/localization_node.py"
elif [ -f "${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher/localization_node.py" ]; then
    LOC_PY="${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher/localization_node.py"
else
    LOC_PY="/home/raspberry/ros2_ws/src/fake_tag_publisher/fake_tag_publisher/localization_node.py"
fi

if [ -f "${SCRIPT_DIR}/video_tag_detector.py" ]; then
    VID_PY="${SCRIPT_DIR}/video_tag_detector.py"
elif [ -f "${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher/video_tag_detector.py" ]; then
    VID_PY="${SCRIPT_DIR}/src/fake_tag_publisher/fake_tag_publisher/video_tag_detector.py"
else
    VID_PY="/home/raspberry/ros2_ws/src/fake_tag_publisher/fake_tag_publisher/video_tag_detector.py"
fi

echo "[+] Active release directory: ${SCRIPT_DIR}"
echo "[+] Using localization_node:  ${LOC_PY}"
echo "[+] Using video_tag_detector: ${VID_PY}"

echo "[+] Starting Static TF (base_link -> camera_link)..."
ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.0 --z 0.0 --roll 0.0 --pitch -1.570796 --yaw 1.570796 --frame-id base_link --child-frame-id camera_link > /tmp/static_tf.log 2>&1 &

echo "[+] Starting Video Tag Detector (CSI Camera via libcamerify)..."
/usr/local/bin/libcamerify python3 "${VID_PY}" --ros-args -p video_path:=0 -p aruco_dictionary:=DICT_4X4_100 > /tmp/video_tag_detector.log 2>&1 &

echo "[+] Starting Localization & Autopilot Web Server (port 8080)..."
python3 "${LOC_PY}" > /tmp/localization_node.log 2>&1 &
PID_LOC=$!

IP_LIST=$(hostname -I | tr ' ' '\n' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | tr '\n' ' ')
PRIMARY_IP=$(echo "$IP_LIST" | awk '{print $1}')
if [ -z "$PRIMARY_IP" ]; then PRIMARY_IP="172.20.10.2"; fi

echo "============================================================="
echo "🎉 TERMiT SYSTEM RUNNING! (PID: ${PID_LOC})"
echo "📱 Web Interface: http://${PRIMARY_IP}:8080/"
echo "📷 Video Stream:  http://${PRIMARY_IP}:8080/video_feed"
echo "🌐 All available IPs: ${IP_LIST}"
echo "============================================================="

while kill -0 $PID_LOC 2>/dev/null; do
    sleep 1
done

