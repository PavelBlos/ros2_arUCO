"""
test_single_tag_pnp.py - Unit tests for single_tag_pnp module.
"""

import math
import numpy as np
import pytest
import os
import sys
import cv2

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher')))

from single_tag_pnp import (
    validate_camera_calibration,
    get_marker_object_points,
    solve_single_tag_ippe
)

@pytest.fixture
def ideal_camera():
    K = np.array([
        [600.0, 0.0, 320.0],
        [0.0, 600.0, 240.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64)
    return K, dist

def test_camera_validation(ideal_camera):
    K, dist = ideal_camera
    ok, reason = validate_camera_calibration(K, dist, 640, 480)
    assert ok is True
    assert reason == "valid"

    ok, reason = validate_camera_calibration(None, dist)
    assert ok is False
    assert "none" in reason

    K_bad = K.copy()
    K_bad[0, 0] = -100.0
    ok, reason = validate_camera_calibration(K_bad, dist)
    assert ok is False

def test_synthetic_tag_ippe(ideal_camera):
    K, dist = ideal_camera
    marker_size = 0.100  # 100 mm
    
    # Project corners with tilt to test realistic IPPE
    half = marker_size / 2.0
    obj_pts = get_marker_object_points(marker_size)
    rvec = np.array([-math.pi + 0.01, 0.01, 0.0])
    tvec = np.array([0.0, 0.0, 2.0])
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    corners_2d = proj.reshape(-1, 2)

    res = solve_single_tag_ippe(corners_2d, marker_size, K, dist)
    
    assert res["pose_valid"] is True
    assert res["rejection_reason"] == ""
    assert res["reproj_err"] < 0.1
    assert abs(res["distance_m"] - 2.0) < 0.02
    assert abs(res["tvec"][2] - 2.0) < 0.02

def test_marker_size_scaling(ideal_camera):
    """
    CRITICAL CHECK:
    If marker size is 100mm vs 150mm for the same pixel corners,
    the calculated distance must scale exactly proportionally: dist(150mm) = 1.5 * dist(100mm).
    """
    K, dist = ideal_camera
    
    # Generate corners with realistic slight tilt
    obj_pts = get_marker_object_points(0.100)
    rvec = np.array([-math.pi + 0.02, 0.01, 0.0])
    tvec = np.array([0.0, 0.0, 1.5])
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    corners_2d = proj.reshape(-1, 2)

    res_100 = solve_single_tag_ippe(corners_2d, 0.100, K, dist)
    res_150 = solve_single_tag_ippe(corners_2d, 0.150, K, dist)

    assert res_100["pose_valid"] is True
    assert res_150["pose_valid"] is True

    ratio = res_150["distance_m"] / res_100["distance_m"]
    assert abs(ratio - 1.50) < 0.02

def test_real_detected_aruco_tag(ideal_camera):
    K, dist = ideal_camera
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    marker_img = cv2.aruco.generateImageMarker(d, 17, 200)
    img = np.ones((480, 640), dtype=np.uint8) * 255
    img[140:340, 220:420] = marker_img
    corners, ids, _ = cv2.aruco.ArucoDetector(d).detectMarkers(img)
    
    res = solve_single_tag_ippe(corners[0][0], 0.100, K, dist)
    assert res["pose_valid"] is True
    assert res["reproj_err"] < 0.05
    assert abs(res["distance_m"] - 0.30) < 0.02

def test_invalid_calibration_blocks_pose(ideal_camera):
    K, dist = ideal_camera
    corners_2d = np.array([[10, 10], [50, 10], [50, 50], [10, 50]], dtype=np.float64)
    res = solve_single_tag_ippe(corners_2d, 0.100, None, dist)
    assert res["pose_valid"] is False
    assert "invalid_calibration" in res["rejection_reason"]
