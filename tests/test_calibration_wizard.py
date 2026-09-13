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
    wizard._response_matrix = np.array([[0.0, 1.0], [-1.0, 0.0]])

    # Tag above the frame centre: move backward so it moves down in view.
    T_front = np.eye(4)
    T_front[:3, 3] = [0.20, 0.0, 2.5]
    det_front = {
        "tag_id": 42,
        "pose_valid": True,
        "T_cameraRos_tag": T_front,
        "corners_px": [310, 140, 330, 140, 330, 160, 310, 160]
    }
    state, cmd, msg = wizard.update([det_front], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.05)
    assert state == WizardState.FINE_CENTERING
    vx, vy, w = cmd
    assert vx < 0.0
    assert abs(vy) < 0.001

    # Tag left of the frame centre: move left so it moves right in view.
    T_left = np.eye(4)
    T_left[:3, 3] = [0.0, 0.20, 2.5]
    det_left = {
        "tag_id": 42,
        "pose_valid": True,
        "T_cameraRos_tag": T_left,
        "corners_px": [190, 230, 210, 230, 210, 250, 190, 250]
    }
    state, cmd, msg = wizard.update([det_left], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.10)
    vx, vy, w = cmd
    assert abs(vx) < 0.001
    assert vy > 0.0  # Must drive left!


def test_centering_uses_geometric_frame_center_and_measured_response():
    wizard = TagCalibrationWizard(cx=320.0, cy=240.0, heartbeat_timeout_sec=3.0)
    wizard.start(target_tag_id=42, now=100.0)
    K = np.array([[798.0, 0.0, 317.0], [0.0, 798.0, 293.0], [0.0, 0.0, 1.0]])
    wizard.update([], False, (0, 0, 0), K, np.zeros(5), np.eye(4), now=100.0)
    wizard._response_matrix = np.array([[0.0, 1.0], [-1.0, 0.0]])

    det = {
        "tag_id": 42,
        "pose_valid": True,
        "T_cameraRos_tag": np.array([
            [1.0, 0.0, 0.0, 0.20],
            [0.0, 1.0, 0.0, 0.30],
            [0.0, 0.0, 1.0, 2.00],
            [0.0, 0.0, 0.0, 1.00],
        ]),
        "corners_px": [220, 270, 240, 270, 240, 290, 220, 290],
    }
    state, cmd, _ = wizard.update([det], False, (0, 0, 0), K, np.zeros(5), np.eye(4), now=100.05)

    assert state == WizardState.FINE_CENTERING
    assert wizard.cx == pytest.approx(320.0)
    assert wizard.cy == pytest.approx(240.0)
    assert wizard.centering_axis == "combined"
    assert cmd[0] > 0.0
    assert cmd[1] > 0.0

    det["T_cameraRos_tag"] = det["T_cameraRos_tag"].copy()
    det["T_cameraRos_tag"][1, 3] = 0.30
    det["corners_px"] = [310, 170, 330, 170, 330, 190, 310, 190]
    _, cmd, _ = wizard.update([det], False, (0, 0, 0), K, np.zeros(5), np.eye(4), now=100.10)

    assert wizard.centering_axis == "combined"
    assert cmd[0] < 0.0
    assert abs(cmd[1]) < 1e-9


def test_centering_ignores_inconsistent_3d_error_for_pixel_objective():
    wizard = TagCalibrationWizard(cx=320.0, cy=240.0, heartbeat_timeout_sec=3.0)
    wizard.start(target_tag_id=42, now=100.0)
    wizard.update([], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.0)
    wizard._response_matrix = np.array([[0.0, 1.0], [-1.0, 0.0]])
    det = {
        "tag_id": 42,
        "pose_valid": True,
        # Deliberately inconsistent with pixels, matching the field failure.
        "T_cameraRos_tag": np.array([
            [1.0, 0.0, 0.0, 0.50],
            [0.0, 1.0, 0.0, 0.50],
            [0.0, 0.0, 1.0, 2.00],
            [0.0, 0.0, 0.0, 1.00],
        ]),
        "corners_px": [430, 220, 450, 220, 450, 240, 430, 240],
    }
    _, cmd, _ = wizard.update([det], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.05)
    assert wizard.centering_axis == "combined"
    assert cmd[0] < 0.0 and cmd[1] < 0.0

    det["corners_px"] = [250, 220, 270, 220, 270, 240, 250, 240]
    _, cmd, _ = wizard.update([det], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.10)
    assert wizard.centering_axis == "combined"
    assert cmd[0] < 0.0 and cmd[1] > 0.0


def test_centering_measures_rotated_camera_response_before_servoing():
    wizard = TagCalibrationWizard(cx=320.0, cy=240.0, heartbeat_timeout_sec=3.0)
    wizard.start(target_tag_id=42, now=100.0)
    wizard.update([], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.0)

    def detection(u, v):
        return {
            "tag_id": 42,
            "pose_valid": True,
            "T_cameraRos_tag": np.eye(4),
            "corners_px": [
                u - 10, v - 10, u + 10, v - 10,
                u + 10, v + 10, u - 10, v + 10,
            ],
        }

    _, cmd, _ = wizard.update([detection(400, 240)], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.05)
    assert cmd == (0.03, 0.0, 0.0)
    wizard.update([detection(394, 232)], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=100.71)
    wizard.update([detection(394, 232)], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=101.07)
    _, cmd, _ = wizard.update([detection(394, 232)], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=101.11)
    assert cmd == (0.0, 0.03, 0.0)
    wizard.update([detection(402, 226)], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=101.77)
    wizard.update([detection(402, 226)], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=102.13)

    assert wizard.centering_phase == "servo"
    assert np.asarray(wizard.centering_response_matrix) == pytest.approx(
        np.array([[-0.6, 0.8], [-0.8, -0.6]])
    )
    _, cmd, _ = wizard.update([detection(402, 226)], False, (0, 0, 0), np.eye(3), np.zeros(5), np.eye(4), now=102.17)
    assert cmd[0] != 0.0
    assert cmd[1] != 0.0

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
        "T_cameraRos_tag": np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 2.5],
            [0.0, 0.0, 0.0, 1.0],
        ]),
        "corners_px": [308, 228, 332, 228, 332, 252, 308, 252]  # center (320, 240)
    }

    t += 0.05
    state, cmd, msg = wizard.update([det_centered], False, (1.0, 2.0, 0.0), K, dist, T_base_cam, now=t)
    assert state == WizardState.FINE_CENTERING
    t += 0.40
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

    assert state == WizardState.REVIEW
    assert wizard.calibrated_tag_result is not None
    assert wizard.calibrated_tag_result["tag_id"] == 99
    assert wizard.calibrated_tag_result["state"] == "provisional"
    assert wizard.calibrated_tag_result["diagnostics"]["samples_count"] >= 5
    assert "covariance" in wizard.calibrated_tag_result

    # 5. Confirm review
    ok_conf, msg_conf = wizard.confirm_review()
    assert ok_conf is True
    assert wizard.state == WizardState.COMPLETED


def test_moderate_tag_pitch_is_reported_as_warning_instead_of_abort():
    wizard = TagCalibrationWizard(heartbeat_timeout_sec=5.0)
    wizard.start(target_tag_id=18, now=100.0)
    wizard.state = WizardState.VERIFYING
    wizard.state_enter_time = 100.0
    wizard.collected_samples = [
        {
            "x": 1.0 + index * 0.0001,
            "y": 2.0 - index * 0.0001,
            "z": 2.1,
            "yaw": 0.1,
            "pitch": math.radians(6.7),
            "reproj_err": 0.2,
            "viewing_angle_deg": 7.0,
        }
        for index in range(30)
    ]

    state, _, _ = wizard.update(
        [], True, (0.0, 0.0, 0.0), np.eye(3), np.zeros(5), np.eye(4), now=100.1
    )

    assert state == WizardState.REVIEW
    diagnostics = wizard.calibrated_tag_result["diagnostics"]
    assert diagnostics["tag_pitch_deg"] == pytest.approx(6.7)
    assert diagnostics["warnings"]


def test_motion_authority_estop():
    mgr = MotionAuthorityManager()
    mgr.request_lease(MotionAuthorityMode.MANUAL, duration_sec=5.0)
    assert mgr.current_mode == MotionAuthorityMode.MANUAL

    # ESTOP preempts manual
    ok, _ = mgr.request_lease(MotionAuthorityMode.ESTOP)
    assert ok is True
    assert mgr.current_mode == MotionAuthorityMode.ESTOP

    # Other requests must be blocked
    ok_man, _ = mgr.request_lease(MotionAuthorityMode.MANUAL)
    assert ok_man is False
    ok_cal, _ = mgr.request_lease(MotionAuthorityMode.CALIBRATION)
    assert ok_cal is False

    # Clear ESTOP
    assert mgr.clear_estop() is True
    assert mgr.current_mode == MotionAuthorityMode.IDLE
