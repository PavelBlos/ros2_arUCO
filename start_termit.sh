#!/bin/bash

source /opt/ros/jazzy/setup.bash
if [ -f /home/raspberry/ros2_ws/install/setup.bash ]; then
    source /home/raspberry/ros2_ws/install/setup.bash
fi

echo "[+] Stopping any previous instances..."
pkill -9 -f 'localization_node' 2>/dev/null || true
pkill -9 -f 'video_tag_detector' 2>/dev/null || true
pkill -9 -f 'static_transform_publisher' 2>/dev/null || true
sleep 1

echo "[+] Starting Static TF (base_link -> camera_link)..."
ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.0 --z 0.0 --roll 0.0 --pitch -1.570796 --yaw 1.570796 --frame-id base_link --child-frame-id camera_link > /tmp/static_tf.log 2>&1 &

echo "[+] Starting Video Tag Detector (CSI Camera via libcamerify)..."
/usr/local/bin/libcamerify python3 /home/raspberry/ros2_ws/src/fake_tag_publisher/fake_tag_publisher/video_tag_detector.py --ros-args -p video_path:=0 -p aruco_dictionary:=DICT_4X4_100 > /tmp/video_tag_detector.log 2>&1 &

echo "[+] Starting Localization & Autopilot Web Server (port 8080)..."
python3 /home/raspberry/ros2_ws/src/fake_tag_publisher/fake_tag_publisher/localization_node.py > /tmp/localization_node.log 2>&1 &
PID_LOC=$!

IP_LIST=$(hostname -I | tr ' ' '\n' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | tr '\n' ' ')
PRIMARY_IP=$(echo "$IP_LIST" | awk '{print $1}')
if [ -z "$PRIMARY_IP" ]; then PRIMARY_IP="172.20.10.2"; fi

echo "============================================================="
echo "🎉 TERMiT SYSTEM RUNNING!"
echo "📱 Web Interface: http://${PRIMARY_IP}:8080/"
echo "📷 Video Stream:  http://${PRIMARY_IP}:8080/video_feed"
echo "🌐 All available IPs: ${IP_LIST}"
echo "============================================================="

while kill -0 $PID_LOC 2>/dev/null; do
    sleep 1
done
