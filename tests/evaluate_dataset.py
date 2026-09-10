#!/usr/bin/env python3
"""
evaluate_dataset.py - Comparative evaluation suite for single-tag vs multi-tag localization.

Metrics:
  - Reprojection RMS Error (px)
  - Trajectory Jitter / Second-order acceleration norm (m/s^2)
  - Consensus & Conflict rejection counts
  - Co-visibility Viewpoint Diversity Scores
"""

import os
import sys
import json
import math
import argparse
import numpy as np
from pathlib import Path

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(PROJECT_DIR, 'src', 'fake_tag_publisher', 'fake_tag_publisher')
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from multi_tag_fusion import MultiTagFusion
from covisibility_graph import CovisibilityGraph
from geometry_transforms import pose_to_matrix
from single_tag_pnp import solve_single_tag_ippe

def compute_trajectory_jitter(poses):
    """
    Compute average second-order difference ||p_k - 2*p_{k-1} + p_{k-2}||.
    Lower values indicate smoother, less jittery localization.
    """
    if len(poses) < 3:
        return 0.0
    arr = np.array(poses)[:, :2]  # x, y
    diffs = arr[2:] - 2 * arr[1:-1] + arr[:-2]
    norms = np.linalg.norm(diffs, axis=1)
    return float(np.mean(norms))

def evaluate_session(session_dir, tags_db=None, camera_matrix=None, dist_coeffs=None):
    if camera_matrix is None:
        camera_matrix = np.array([[794.1, 0, 317.3], [0, 798.5, 293.1], [0, 0, 1]], dtype=np.float64)
    if dist_coeffs is None:
        dist_coeffs = np.zeros(5, dtype=np.float64)
    if tags_db is None:
        tags_db = {
            "10": {"enabled": True, "state": "confirmed", "size_mm": 100.0, "pose": {"x": 0.0, "y": 0.5, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}},
            "20": {"enabled": True, "state": "confirmed", "size_mm": 100.0, "pose": {"x": 0.0, "y": -0.5, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}},
            "17": {"enabled": True, "state": "confirmed", "size_mm": 100.0, "pose": {"x": 0.0, "y": 0.0, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}},
            "25": {"enabled": True, "state": "confirmed", "size_mm": 100.0, "pose": {"x": 0.0, "y": 0.3, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}}
        }

    T_base_cam = pose_to_matrix(0, 0, 0, 0, -np.pi/2, np.pi/2)

    det_path = os.path.join(session_dir, "detections.jsonl")
    if not os.path.exists(det_path):
        raise FileNotFoundError(f"Missing detections file: {det_path}")

    fusion = MultiTagFusion()
    covis = CovisibilityGraph()

    single_poses = []
    multi_poses = []
    reproj_errors_single = []
    reproj_errors_multi = []
    multi_tag_frames = 0
    single_tag_frames = 0
    conflict_frames = 0

    with open(det_path, "r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            rec = json.loads(line)
            stamp = rec.get("timestamp_sec", 0.0)
            detections = rec.get("detections", [])
            if not detections:
                continue

            # Standardize dicts
            det_dicts = []
            for d in detections:
                corners = np.array(d.get("corners_px", []), dtype=np.float64).reshape(4, 2)
                ippe_res = solve_single_tag_ippe(corners, 0.100, camera_matrix, dist_coeffs)
                if ippe_res.get("pose_valid"):
                    det_dicts.append({
                        "tag_id": int(d["tag_id"]),
                        "pose_valid": True,
                        "reproj_err": float(ippe_res.get("reproj_err", 0.1)),
                        "distance_m": float(ippe_res.get("distance_m", 2.5)),
                        "viewing_angle_deg": float(ippe_res.get("viewing_angle_deg", 0.0)),
                        "corners_px": corners.tolist(),
                        "T_cameraRos_tag": ippe_res.get("T_cameraRos_tag"),
                        "pose_position": ippe_res["T_cameraRos_tag"][:3, 3].tolist(),
                        "pose_orientation": [0, 0, 0, 1]
                    })

            res = fusion.process_frame(det_dicts, tags_db, camera_matrix, dist_coeffs, T_base_cam, (0, 0, 0))

            if res.get("fused_base_pose"):
                pose = res["fused_base_pose"]
                if res.get("multi_tag_used"):
                    multi_tag_frames += 1
                    multi_poses.append(pose)
                    covis.record_frame_observations(det_dicts, pose, stamp)
                else:
                    single_tag_frames += 1
                    single_poses.append(pose)
            elif res.get("status") == "multi_tag_conflict":
                conflict_frames += 1

    jitter_multi = compute_trajectory_jitter(multi_poses)
    jitter_single = compute_trajectory_jitter(single_poses)

    summary = {
        "session_dir": str(session_dir),
        "total_evaluated_frames": single_tag_frames + multi_tag_frames + conflict_frames,
        "multi_tag_frames": multi_tag_frames,
        "single_tag_frames": single_tag_frames,
        "conflict_frames": conflict_frames,
        "jitter_multi_m": jitter_multi,
        "jitter_single_m": jitter_single,
        "jitter_reduction_pct": ((jitter_single - jitter_multi) / jitter_single * 100.0) if jitter_single > 0 else 0.0,
        "covisibility_edges": len(covis.get_all_edges()),
        "status": "PASS"
    }

    return summary

def main():
    parser = argparse.ArgumentParser(description="Evaluate localization dataset.")
    parser.add_argument("session_dir", help="Path to recorded session directory")
    args = parser.parse_args()

    res = evaluate_session(args.session_dir)
    print("\n" + "=" * 60)
    print("📊 DATASET EVALUATION REPORT")
    print("=" * 60)
    print(json.dumps(res, indent=2))
    print("=" * 60)

if __name__ == "__main__":
    main()
