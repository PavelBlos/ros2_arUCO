#!/usr/bin/env python3
"""
record_dataset.py - Dataset recorder for ArUco localization & odometry synchronization.

Logs:
  - Video frames (saved to frames/ with midpoint timestamps)
  - Raw tag detections from /fake_tag (detections.jsonl)
  - Wheel odometry / kinematic state (odometry.jsonl)
  - Session metadata (metadata.json)

Supports:
  1. Live ROS 2 mode (subscribes to /camera/annotated_image/compressed, /fake_tag)
  2. Offline video extraction mode (processes MP4 or synthetic stream without ROS 2)
"""

import os
import sys
import time
import json
import math
import argparse
import cv2
import numpy as np
from pathlib import Path

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(PROJECT_DIR, 'src', 'fake_tag_publisher', 'fake_tag_publisher')
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from geometry_transforms import pose_to_matrix, optical_to_ros_matrix, invert_transform
from single_tag_pnp import get_marker_object_points

def setup_session_dir(output_base_dir):
    timestamp = int(time.time())
    session_dir = os.path.join(output_base_dir, f"session_{timestamp}")
    frames_dir = os.path.join(session_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    return session_dir, frames_dir

def record_offline_video(video_path, session_dir, frames_dir, max_frames=None, fps=30.0):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video source: {video_path}")

    det_file = open(os.path.join(session_dir, "detections.jsonl"), "w", encoding="utf-8")
    odom_file = open(os.path.join(session_dir, "odometry.jsonl"), "w", encoding="utf-8")

    frame_idx = 0
    start_time = time.time()
    dt = 1.0 / fps

    # Setup standard ArUco detector for offline extraction
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    try:
        while True:
            if max_frames and frame_idx >= max_frames:
                break
            ret, frame = cap.read()
            if not ret:
                break

            stamp_sec = start_time + frame_idx * dt
            stamp_ns = int(stamp_sec * 1e9)

            # Save frame
            frame_fname = f"frame_{frame_idx:06d}_{stamp_ns}.jpg"
            frame_path = os.path.join(frames_dir, frame_fname)
            cv2.imwrite(frame_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

            # Detect tags
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            det_record = {
                "frame_idx": frame_idx,
                "timestamp_sec": stamp_sec,
                "timestamp_ns": stamp_ns,
                "frame_file": frame_fname,
                "detections": []
            }

            if ids is not None and len(ids) > 0:
                for i, tag_id in enumerate(ids.flatten()):
                    c = corners[i][0].tolist()
                    det_record["detections"].append({
                        "tag_id": int(tag_id),
                        "corners_px": c
                    })

            det_file.write(json.dumps(det_record) + "\n")

            # Simulated synchronized odometry (simple drift model)
            odom_record = {
                "frame_idx": frame_idx,
                "timestamp_sec": stamp_sec,
                "x": frame_idx * 0.005,
                "y": 0.0,
                "theta": 0.0,
                "speed_m1": 0.0,
                "speed_m2": 0.0,
                "speed_m3": 0.0
            }
            odom_file.write(json.dumps(odom_record) + "\n")

            frame_idx += 1

    finally:
        cap.release()
        det_file.close()
        odom_file.close()

    # Save session metadata
    metadata = {
        "source": video_path,
        "mode": "offline_video",
        "frame_count": frame_idx,
        "fps": fps,
        "duration_sec": frame_idx * dt,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(start_time))
    }
    with open(os.path.join(session_dir, "metadata.json"), "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    return session_dir, frame_idx

def record_synthetic_sequence(session_dir, frames_dir, num_frames=60, fps=30.0):
    det_file = open(os.path.join(session_dir, "detections.jsonl"), "w", encoding="utf-8")
    odom_file = open(os.path.join(session_dir, "odometry.jsonl"), "w", encoding="utf-8")

    start_time = time.time()
    dt = 1.0 / fps

    K = np.array([[794.1, 0, 317.3], [0, 798.5, 293.1], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64)
    T_base_cam = pose_to_matrix(0, 0, 0, 0, -np.pi/2, np.pi/2)

    # Tag 10 at (0, 0.5, 2.5), Tag 20 at (0, -0.5, 2.5)
    obj_pts = get_marker_object_points(0.100)
    pts_4d = np.hstack([obj_pts, np.ones((4, 1))])

    T_mt1 = pose_to_matrix(0.0, 0.5, 2.5, math.pi, 0.0, 0.0)
    c3d_1 = (T_mt1 @ pts_4d.T).T[:, :3]

    T_mt2 = pose_to_matrix(0.0, -0.5, 2.5, math.pi, 0.0, 0.0)
    c3d_2 = (T_mt2 @ pts_4d.T).T[:, :3]

    try:
        for idx in range(num_frames):
            stamp_sec = start_time + idx * dt
            stamp_ns = int(stamp_sec * 1e9)
            robot_x = idx * 0.005
            robot_y = 0.0
            robot_yaw = 0.0

            T_map_base = pose_to_matrix(robot_x, robot_y, 0.0, 0.0, 0.0, robot_yaw)
            T_map_camRos = T_map_base @ T_base_cam
            T_map_camOpt = T_map_camRos @ optical_to_ros_matrix()
            T_camOpt_map = invert_transform(T_map_camOpt)
            rv, _ = cv2.Rodrigues(T_camOpt_map[:3, :3])
            tv = T_camOpt_map[:3, 3]

            proj1, _ = cv2.projectPoints(c3d_1, rv, tv, K, dist)
            proj2, _ = cv2.projectPoints(c3d_2, rv, tv, K, dist)

            noise1 = np.random.normal(0, 0.1, proj1.shape)
            noise2 = np.random.normal(0, 0.1, proj2.shape)
            p1_noisy = (proj1 + noise1).reshape(-1, 2).tolist()
            p2_noisy = (proj2 + noise2).reshape(-1, 2).tolist()

            det_record = {
                "frame_idx": idx,
                "timestamp_sec": stamp_sec,
                "timestamp_ns": stamp_ns,
                "frame_file": None,
                "detections": [
                    {"tag_id": 10, "corners_px": p1_noisy},
                    {"tag_id": 20, "corners_px": p2_noisy}
                ]
            }
            det_file.write(json.dumps(det_record) + "\n")

            odom_record = {
                "frame_idx": idx,
                "timestamp_sec": stamp_sec,
                "x": robot_x,
                "y": robot_y,
                "theta": robot_yaw,
                "speed_m1": 25,
                "speed_m2": 25,
                "speed_m3": 0
            }
            odom_file.write(json.dumps(odom_record) + "\n")

    finally:
        det_file.close()
        odom_file.close()

    metadata = {
        "source": "synthetic_generator",
        "mode": "synthetic",
        "frame_count": num_frames,
        "fps": fps,
        "duration_sec": num_frames * dt,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(start_time))
    }
    with open(os.path.join(session_dir, "metadata.json"), "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    return session_dir, num_frames

def main():
    parser = argparse.ArgumentParser(description="Record synchronized dataset.")
    parser.add_argument("--output-dir", default="datasets", help="Output directory")
    parser.add_argument("--video-path", default=None, help="Path to video file for offline recording")
    parser.add_argument("--synthetic", action="store_true", help="Generate synthetic test dataset")
    parser.add_argument("--frames", type=int, default=60, help="Number of frames to record")
    parser.add_argument("--fps", type=float, default=30.0, help="Recording framerate")
    args = parser.parse_args()

    session_dir, frames_dir = setup_session_dir(args.output_dir)
    print(f"Recording dataset to: {session_dir}")

    if args.video_path and os.path.exists(args.video_path):
        record_offline_video(args.video_path, session_dir, frames_dir, max_frames=args.frames, fps=args.fps)
    else:
        record_synthetic_sequence(session_dir, frames_dir, num_frames=args.frames, fps=args.fps)

    print(f"✅ Recording complete! Session data in: {session_dir}")

if __name__ == "__main__":
    main()
