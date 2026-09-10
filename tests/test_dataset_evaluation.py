"""
test_dataset_evaluation.py - Unit tests for dataset recording and comparative evaluation.
"""

import os
import sys
import json
import pytest

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from tests.record_dataset import record_synthetic_sequence
from tests.evaluate_dataset import evaluate_session, compute_trajectory_jitter

def test_record_and_evaluate_synthetic_dataset(tmp_path):
    session_dir = str(tmp_path / "test_session")
    frames_dir = os.path.join(session_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    # 1. Record 30 synthetic frames
    sdir, nframes = record_synthetic_sequence(session_dir, frames_dir, num_frames=30, fps=30.0)
    assert nframes == 30

    # Verify generated files
    assert os.path.exists(os.path.join(session_dir, "metadata.json"))
    assert os.path.exists(os.path.join(session_dir, "detections.jsonl"))
    assert os.path.exists(os.path.join(session_dir, "odometry.jsonl"))

    with open(os.path.join(session_dir, "metadata.json"), "r", encoding="utf-8") as f:
        meta = json.load(f)
        assert meta["frame_count"] == 30
        assert meta["mode"] == "synthetic"

    # 2. Evaluate dataset
    eval_res = evaluate_session(session_dir)
    assert eval_res["status"] == "PASS"
    assert eval_res["total_evaluated_frames"] > 0
    assert eval_res["covisibility_edges"] >= 1

def test_compute_trajectory_jitter():
    # Constant position -> zero jitter
    poses_still = [(0.0, 0.0, 0.0)] * 10
    assert compute_trajectory_jitter(poses_still) == 0.0

    # Linear motion -> zero second order acceleration
    poses_linear = [(i * 0.1, i * 0.2, 0.0) for i in range(10)]
    assert compute_trajectory_jitter(poses_linear) < 1e-6

    # Jittery motion -> non-zero jitter
    poses_noisy = [(i * 0.1 + (-1)**i * 0.05, 0.0, 0.0) for i in range(10)]
    assert compute_trajectory_jitter(poses_noisy) > 0.05
