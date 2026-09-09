"""
test_calibration_wizard.py - Unit tests for TagCalibrationWizard and MotionAuthorityManager.
"""

import math
import numpy as np
import pytest
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher')))

from tag_calibration_wizard import (
    MotionAuthorityMode,
    MotionAuthorityManager,
    WizardState,
    TagCalibrationWizard
)
from geometry_transforms import pose_to_matrix

def test_motion_authority_mutex():
    mgr = MotionAuthorityManager(default_lease_sec=0.5)
    assert mgr.current_mode == MotionAuthorityMode.IDLE

    # Request lease for CALIBRATION
    ok, msg = mgr.request_lease(MotionAuthorityMode.CALIBRATION, 0.5)
    assert ok is True
    assert mgr.current_mode == MotionAuthorityMode.CALIBRATION

    # Conflicting request from ROUTE must be rejected
    ok2, msg2 = mgr.request_lease(MotionAuthorityMode.ROUTE, 0.5)
    assert ok2 is False
    assert "conflict" in msg2.lower()

    # Renewal succeeds
    ok3, _ = mgr.renew_lease(MotionAuthorityMode.CALIBRATION, 0.5)
    assert ok3 is True

    # Release lease reverts to IDLE
    mgr.release_lease(MotionAuthorityMode.CALIBRATION)
    assert mgr.current_mode == MotionAuthorityMode.IDLE

def test_wizard_heartbeat_timeout():
    mgr = MotionAuthorityManager()
    wizard = TagCalibrationWizard(motion_manager=mgr, heartbeat_timeout_sec=0.5)

    ok, _ = wizard.start(target_tag_id=42, now=100.0)
    assert ok is True
    assert wizard.state == WizardState.ACQUIRING_LEASES

    # Tick at 100.1s
    state, cmd, _ = wizard.update([], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.1)
    assert state == WizardState.FINE_CENTERING

    # Tick at 100.7s (> 0.5s without heartbeat) -> ABORT
    state, cmd, msg = wizard.update([], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.7)
    assert state == WizardState.ABORTED
    assert "heartbeat" in msg.lower()
    assert cmd == (0.0, 0.0, 0.0)
    assert mgr.current_mode == MotionAuthorityMode.IDLE

def test_wizard_target_loss_watchdog():
    mgr = MotionAuthorityManager()
    wizard = TagCalibrationWizard(motion_manager=mgr, target_loss_timeout_sec=0.25, heartbeat_timeout_sec=2.0)

    wizard.start(target_tag_id=42, now=100.0)
    wizard.update([], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.05)
    assert wizard.state == WizardState.FINE_CENTERING

    # Tag lost for 0.30s (> 0.25s watchdog)
    state, cmd, msg = wizard.update([], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.35)
    assert state == WizardState.ABORTED
    assert "lost" in msg.lower()
    assert cmd == (0.0, 0.0, 0.0)

def test_visual_servoing_coordinate_signs():
    wizard = TagCalibrationWizard(cx=320.0, cy=240.0, centering_tol_px=10.0)
    wizard.start(target_tag_id=42, now=100.0)
    wizard.update([], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.0)

    # 1. Tag is towards top of image (v = 150 < cy = 240, tag is in front of robot)
    # Robot must drive FORWARD (+vx > 0)
    det_front = {
        "tag_id": 42,
        "corners_px": [310, 140, 330, 140, 330, 160, 310, 160]  # center (320, 150)
    }
    state, cmd, msg = wizard.update([det_front], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.05)
    assert state == WizardState.FINE_CENTERING
    vx, vy, w = cmd
    assert vx > 0.0  # Must drive forward!
    assert abs(vy) < 0.001

    # 2. Tag is towards left of image (u = 200 < cx = 320, tag is to the left of robot)
    # Robot must drive LEFT (+vy > 0 in REP-103)
    det_left = {
        "tag_id": 42,
        "corners_px": [190, 230, 210, 230, 210, 250, 190, 250]  # center (200, 240)
    }
    state, cmd, msg = wizard.update([det_left], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.10)
    vx, vy, w = cmd
    assert abs(vx) < 0.001
    assert vy > 0.0  # Must drive left!

def test_full_calibration_flow_to_completion():
    K = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]], dtype=np.float64)
    dist = np.zeros(5)
    T_base_cam = np.eye(4)

    wizard = TagCalibrationWizard(
        cx=320.0, cy=240.0,
        centering_tol_px=15.0,
        settling_delay_sec=0.5,
        min_stationary_frames=5,
        stationary_duration_sec=0.5,
        heartbeat_timeout_sec=5.0
    )

    t = 100.0
    wizard.start(target_tag_id=99, marker_size_m=0.100, now=t)
    wizard.update([], False, (0, 0, 0), K, dist, T_base_cam, now=t)

    # 1. Provide centered tag detection (320, 240)
    # 4 corners at distance 2.5m
    det_centered = {
        "tag_id": 99,
        "pose_valid": True,
        "corners_px": [308, 228, 332, 228, 332, 252, 308, 252]  # center (320, 240)
    }

    t += 0.05
    state, cmd, msg = wizard.update([det_centered], False, (1.0, 2.0, 0.0), K, dist, T_base_cam, now=t)
    assert state == WizardState.SETTLING
    assert cmd == (0.0, 0.0, 0.0)

    # 2. Wait settling delay (0.5s)
    t += 0.60
    state, cmd, msg = wizard.update([det_centered], False, (1.0, 2.0, 0.0), K, dist, T_base_cam, now=t)
    assert state == WizardState.STOPPED_CHECK

    # 3. Stopped check
    t += 0.05
    state, cmd, msg = wizard.update([det_centered], True, (1.0, 2.0, 0.0), K, dist, T_base_cam, now=t)
    assert state == WizardState.STATIONARY_SOLVE

    # 4. Accumulate 5 stationary frames
    for _ in range(5):
        t += 0.12
        state, cmd, msg = wizard.update([det_centered], True, (1.0, 2.0, 0.0), K, dist, T_base_cam, now=t)

    assert state == WizardState.COMPLETED
    assert wizard.calibrated_tag_result is not None
    assert wizard.calibrated_tag_result["tag_id"] == 99
    assert wizard.calibrated_tag_result["state"] == "provisional"
    assert wizard.calibrated_tag_result["diagnostics"]["samples_count"] >= 5