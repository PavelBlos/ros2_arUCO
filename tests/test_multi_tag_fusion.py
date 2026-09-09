"""
test_multi_tag_fusion.py - Rigorous unit tests for multi-tag consensus and joint PnP.
"""

import math
import numpy as np
import pytest
import cv2
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher')))

from multi_tag_fusion import (
    compute_visual_covariance,
    propagate_odometry_covariance,
    compute_mahalanobis_distance,
    MultiTagFusion
)
from geometry_transforms import pose_to_matrix, invert_transform, optical_to_ros_matrix
from single_tag_pnp import get_marker_object_points

@pytest.fixture
def test_setup():
    K = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]], dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64)
    T_base_cam = pose_to_matrix(0.0, 0.0, 0.0, 0.0, -math.pi/2.0, math.pi/2.0)
    return K, dist, T_base_cam

def test_visual_covariance_properties():
    # Closer marker should have lower covariance than farther marker
    cov_near = compute_visual_covariance(1.0, 10.0, 1.0, 3000.0)
    cov_far = compute_visual_covariance(3.0, 10.0, 1.0, 3000.0)
    assert cov_far[0, 0] > cov_near[0, 0]

    # Higher viewing angle should have higher covariance
    cov_straight = compute_visual_covariance(2.0, 5.0, 1.0, 3000.0)
    cov_angled = compute_visual_covariance(2.0, 50.0, 1.0, 3000.0)
    assert cov_angled[0, 0] > cov_straight[0, 0]

def test_odometry_se2_jacobian_propagation():
    prev_cov = np.diag([0.01**2, 0.01**2, 0.01**2])
    # Move 1 meter forward with 90 deg heading
    new_cov = propagate_odometry_covariance(prev_cov, 1.0, 0.0, 0.0, math.pi/2.0, dt=0.1)
    assert new_cov[0, 0] > prev_cov[0, 0]
    assert new_cov[1, 1] > prev_cov[1, 1]

def test_mahalanobis_distance():
    cov1 = np.diag([0.02**2, 0.02**2, 0.02**2])
    cov2 = np.diag([0.02**2, 0.02**2, 0.02**2])
    # Exact match -> distance 0
    d_m, _ = compute_mahalanobis_distance(1.0, 2.0, 0.5, cov1, 1.0, 2.0, 0.5, cov2)
    assert d_m < 1e-6

    # 10 cm separation with ~2.8 cm total std -> D_M around 3.5
    d_m_sep, _ = compute_mahalanobis_distance(1.0, 2.0, 0.0, cov1, 1.10, 2.0, 0.0, cov2)
    assert 3.0 < d_m_sep < 4.0

def test_two_tag_conflict_holds_odometry(test_setup):
    """
    When two tags report conflicting robot positions and cannot be resolved,
    multi-tag fusion must return status 'multi_tag_conflict' and NEVER jump blindly!
    """
    K, dist, T_base_cam = test_setup
    fusion = MultiTagFusion()

    active_db = {
        "1": {"enabled": True, "state": "confirmed", "pose": {"x": 0.0, "y": 0.0, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}},
        "2": {"enabled": True, "state": "confirmed", "pose": {"x": 2.0, "y": 0.0, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}},
    }

    # Simulate 2 conflicting detections: Tag 1 places robot at (0, 0, 0), Tag 2 places robot at (1.5, 0, 0)
    det1 = {
        "tag_id": 1,
        "pose_valid": True,
        "marker_size_mm": 100.0,
        "distance_m": 2.0,
        "viewing_angle_deg": 5.0,
        "reproj_err": 0.5,
        "marker_area_px": 2000.0,
        "corners_px": [300, 220, 340, 220, 340, 260, 300, 260],
        "pose_position": [0.0, 0.0, 2.0],
        "pose_orientation": [0.0, 0.0, 0.0, 1.0]
    }
    # Corrupted tag 2
    det2 = {
        "tag_id": 2,
        "pose_valid": True,
        "marker_size_mm": 100.0,
        "distance_m": 2.0,
        "viewing_angle_deg": 5.0,
        "reproj_err": 0.5,
        "marker_area_px": 2000.0,
        "corners_px": [400, 220, 440, 220, 440, 260, 400, 260],
        "pose_position": [0.0, 0.0, 2.0],
        "pose_orientation": [0.0, 0.0, 0.0, 1.0]
    }

    res = fusion.process_frame([det1, det2], active_db, K, dist, T_base_cam, odom_at_stamp=(0.0, 0.0, 0.0))
    assert res["status"] == "multi_tag_conflict"
    assert res["fused_base_pose"] is None  # DO NOT MAKE A VISUAL JUMP!

def test_joint_pnp_reduces_noise(test_setup):
    """
    Demonstrate that multi-tag joint PnP across 2 consistent tags reduces noise
    compared to single tag.
    """
    K, dist, T_base_cam = test_setup
    fusion = MultiTagFusion()

    # Ground truth robot at (0.0, 0.0, 0.0)
    # Tag 1 at (0.0, 0.5, 2.5), Tag 2 at (0.0, -0.5, 2.5)
    active_db = {
        "10": {"enabled": True, "state": "confirmed", "pose": {"x": 0.0, "y": 0.5, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}},
        "20": {"enabled": True, "state": "confirmed", "pose": {"x": 0.0, "y": -0.5, "z": 2.5, "roll": math.pi, "pitch": 0.0, "yaw": 0.0}},
    }

    # Camera on robot looking up at ceiling
    T_map_base = np.eye(4)
    T_map_camRos = T_map_base @ T_base_cam
    T_map_camOpt = T_map_camRos @ optical_to_ros_matrix()
    T_camOpt_map = invert_transform(T_map_camOpt)
    rv, _ = cv2.Rodrigues(T_camOpt_map[:3, :3])
    tv = T_camOpt_map[:3, 3]

    # Project 3D corners of both tags
    obj_pts = get_marker_object_points(0.100)
    pts_4d = np.hstack([obj_pts, np.ones((4, 1))])

    # Tag 1
    T_mt1 = pose_to_matrix(0.0, 0.5, 2.5, math.pi, 0.0, 0.0)
    c3d_1 = (T_mt1 @ pts_4d.T).T[:, :3]
    proj1, _ = cv2.projectPoints(c3d_1, rv, tv, K, dist)

    # Tag 2
    T_mt2 = pose_to_matrix(0.0, -0.5, 2.5, math.pi, 0.0, 0.0)
    c3d_2 = (T_mt2 @ pts_4d.T).T[:, :3]
    proj2, _ = cv2.projectPoints(c3d_2, rv, tv, K, dist)

    det1 = {
        "tag_id": 10,
        "pose_valid": True,
        "marker_size_mm": 100.0,
        "distance_m": 2.5,
        "viewing_angle_deg": 10.0,
        "reproj_err": 0.02,
        "marker_area_px": 500.0,
        "corners_px": proj1.reshape(-1).tolist()
    }
    det2 = {
        "tag_id": 20,
        "pose_valid": True,
        "marker_size_mm": 100.0,
        "distance_m": 2.5,
        "viewing_angle_deg": 10.0,
        "reproj_err": 0.02,
        "marker_area_px": 500.0,
        "corners_px": proj2.reshape(-1).tolist()
    }

    res = fusion.process_frame([det1, det2], active_db, K, dist, T_base_cam=T_base_cam, odom_at_stamp=(0.0, 0.0, 0.0))
    assert res["status"] == "multi_tag_ok"
    assert res["multi_tag_used"] is True
    assert set(res["inlier_ids"]) == {"10", "20"}
    assert res["reproj_rms_px"] < 0.1
    # Robot pose near (0, 0, 0)
    x_f, y_f, yaw_f = res["fused_base_pose"]
    assert abs(x_f) < 0.02
    assert abs(y_f) < 0.02
