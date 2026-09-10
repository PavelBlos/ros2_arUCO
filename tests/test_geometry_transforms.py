"""
test_geometry_transforms.py - Rigorous unit tests for geometry_transforms module.
"""

import math
import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher')))

from geometry_transforms import (
    normalize_angle,
    circular_mean,
    compute_midpoint_stamp,
    compute_latency_ms,
    optical_to_ros_rotation,
    optical_to_ros_matrix,
    ros_to_optical_matrix,
    pose_to_matrix,
    matrix_to_pose,
    invert_transform,
    camera_pose_from_tag,
    base_pose_from_camera,
    compute_map_to_odom_se2,
    compute_fused_pose_se2,
    smooth_map_to_odom_se2,
    map_velocity_to_body
)

@pytest.mark.parametrize(
    "map_velocity,yaw,expected_body",
    [
        ((1.0, 0.0), 0.0, (1.0, 0.0)),
        ((0.0, 1.0), 0.0, (0.0, 1.0)),
        ((1.0, 0.0), math.pi / 2.0, (0.0, -1.0)),
        ((0.0, 1.0), math.pi / 2.0, (1.0, 0.0)),
    ],
)
def test_map_velocity_to_rep103_body(map_velocity, yaw, expected_body):
    forward, left = map_velocity_to_body(*map_velocity, yaw)
    assert forward == pytest.approx(expected_body[0], abs=1e-9)
    assert left == pytest.approx(expected_body[1], abs=1e-9)

def test_angle_normalization():
    assert abs(normalize_angle(0.0)) < 1e-9
    assert abs(normalize_angle(math.pi)) < 1e-9 or abs(normalize_angle(math.pi) - math.pi) < 1e-9 or abs(normalize_angle(math.pi) + math.pi) < 1e-9
    assert abs(normalize_angle(3.0 * math.pi) - (-math.pi)) < 1e-9 or abs(normalize_angle(3.0 * math.pi) - math.pi) < 1e-9
    assert abs(normalize_angle(-3.0 * math.pi) - (-math.pi)) < 1e-9 or abs(normalize_angle(-3.0 * math.pi) - math.pi) < 1e-9
    assert abs(normalize_angle(2.5 * math.pi - 0.5 * math.pi)) < 1e-9

def test_circular_mean():
    # Mean of 10 deg and -10 deg should be 0 deg
    mean_val = circular_mean([math.radians(10), math.radians(-10)])
    assert abs(mean_val) < 1e-9

    # Near +/- pi boundary: +170 deg and -170 deg should average to 180 deg (or -180 deg)
    mean_boundary = circular_mean([math.radians(170), math.radians(-170)])
    assert abs(abs(mean_boundary) - math.pi) < 1e-9

def test_clock_helpers():
    t_mid = compute_midpoint_stamp(100.0, 100.04)
    assert abs(t_mid - 100.02) < 1e-9
    lat = compute_latency_ms(100.0, 100.035)
    assert abs(lat - 35.0) < 1e-6

def test_optical_ros_roundtrip():
    T_opt_ros = optical_to_ros_matrix()
    T_ros_opt = ros_to_optical_matrix()
    identity = T_opt_ros @ T_ros_opt
    assert np.allclose(identity, np.eye(4), atol=1e-9)

    # A point along optical Z (forward in OpenCV) must be along ROS +X (forward in ROS)
    p_opt = np.array([0.0, 0.0, 2.0, 1.0])
    p_ros = T_opt_ros @ p_opt
    assert np.allclose(p_ros[:3], [2.0, 0.0, 0.0], atol=1e-9)

    # A point along optical X (right in OpenCV) must be along ROS -Y (right in ROS)
    p_opt_right = np.array([1.5, 0.0, 0.0, 1.0])
    p_ros_right = T_opt_ros @ p_opt_right
    assert np.allclose(p_ros_right[:3], [0.0, -1.5, 0.0], atol=1e-9)


def test_original_robot_camera_mounting_axes():
    """The real mounting has image bottom forward and image right to robot right."""
    T_base_camera = pose_to_matrix(0.0, 0.0, 0.0, 0.0, -math.pi / 2.0, 0.0)
    T_base_optical = T_base_camera @ optical_to_ros_matrix()

    image_bottom = T_base_optical @ np.array([0.0, 1.0, 0.0, 1.0])
    image_right = T_base_optical @ np.array([1.0, 0.0, 0.0, 1.0])
    optical_axis = T_base_optical @ np.array([0.0, 0.0, 1.0, 1.0])

    assert np.allclose(image_bottom[:3], [1.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(image_right[:3], [0.0, -1.0, 0.0], atol=1e-9)
    assert np.allclose(optical_axis[:3], [0.0, 0.0, 1.0], atol=1e-9)

def test_se3_pose_matrix_roundtrip():
    x, y, z = 1.25, -0.75, 2.50
    roll, pitch, yaw = 0.1, -0.2, 1.5
    T = pose_to_matrix(x, y, z, roll, pitch, yaw)
    x2, y2, z2, r2, p2, yaw2 = matrix_to_pose(T)
    assert abs(x - x2) < 1e-9
    assert abs(y - y2) < 1e-9
    assert abs(z - z2) < 1e-9
    assert abs(roll - r2) < 1e-9
    assert abs(pitch - p2) < 1e-9
    assert abs(yaw - yaw2) < 1e-9

def test_invert_transform():
    T = pose_to_matrix(2.0, -1.0, 0.5, 0.3, -0.2, 0.7)
    T_inv = invert_transform(T)
    assert np.allclose(T @ T_inv, np.eye(4), atol=1e-9)
    assert np.allclose(T_inv @ T, np.eye(4), atol=1e-9)

def test_chain_camera_and_base():
    # Ceiling tag at (0, 0, 2.5), looking down (roll = pi)
    T_map_tag = pose_to_matrix(0.0, 0.0, 2.5, math.pi, 0.0, 0.0)
    # Camera sees tag at (0, 0, 2.0) in camera optical frame
    # Camera optical looking up: relative transform
    T_cam_tag = pose_to_matrix(0.0, 0.0, 2.0, math.pi, 0.0, 0.0)
    T_map_cam = camera_pose_from_tag(T_map_tag, T_cam_tag)
    # Camera at (0, 0, 0.5) in map frame
    assert abs(T_map_cam[2, 3] - 0.5) < 1e-6

    # Robot base has camera at offset (0.1, 0.0, 0.2)
    T_base_cam = pose_to_matrix(0.1, 0.0, 0.2, 0.0, 0.0, 0.0)
    T_map_base = base_pose_from_camera(T_map_cam, T_base_cam)
    assert abs(T_map_base[0, 3] - (-0.1)) < 1e-6
    assert abs(T_map_base[2, 3] - 0.3) < 1e-6

def test_exact_se2_composition_with_rotated_odom():
    """
    CRITICAL TEST:
    Demonstrates that scalar addition (x_visual - x_odom) fails when odometry has non-zero yaw,
    while exact SE(2) composition succeeds perfectly.
    """
    # Suppose odometry frame is rotated by 90 degrees (pi/2) relative to map frame.
    # True robot base in map is at (3.0, 4.0, pi/2).
    # In odom frame, robot is at (1.0, 0.0, 0.0).
    # Then map->odom origin must be at (3.0, 3.0, pi/2) because (1, 0) rotated by 90 deg is (0, 1).
    x_mb, y_mb, yaw_mb = 3.0, 4.0, math.pi / 2.0
    x_ob, y_ob, yaw_ob = 1.0, 0.0, 0.0

    x_mo, y_mo, yaw_mo = compute_map_to_odom_se2(x_mb, y_mb, yaw_mb, x_ob, y_ob, yaw_ob)

    # Verify map->odom transform:
    # yaw_mo = pi/2 - 0 = pi/2
    # x_mo = 3.0 - (1.0*cos(pi/2) - 0*sin(pi/2)) = 3.0 - 0 = 3.0
    # y_mo = 4.0 - (1.0*sin(pi/2) + 0*cos(pi/2)) = 4.0 - 1.0 = 3.0
    assert abs(yaw_mo - math.pi / 2.0) < 1e-9
    assert abs(x_mo - 3.0) < 1e-9
    assert abs(y_mo - 3.0) < 1e-9

    # Now robot moves further in odom to (2.0, 1.0, 0.2)
    x_curr_ob, y_curr_ob, yaw_curr_ob = 2.0, 1.0, 0.2
    x_fused, y_fused, yaw_fused = compute_fused_pose_se2(
        x_mo, y_mo, yaw_mo, x_curr_ob, y_curr_ob, yaw_curr_ob
    )

    # Expected:
    # rotated by pi/2: (2, 1) -> (-1, 2)
    # plus (3, 3) -> (2, 5)
    assert abs(x_fused - 2.0) < 1e-9
    assert abs(y_fused - 5.0) < 1e-9
    assert abs(yaw_fused - (math.pi / 2.0 + 0.2)) < 1e-9

def test_se2_smoothing():
    curr = (0.0, 0.0, 0.1)
    target = (1.0, 2.0, 0.5)
    smoothed = smooth_map_to_odom_se2(curr[0], curr[1], curr[2], target[0], target[1], target[2], alpha=0.5)
    assert abs(smoothed[0] - 0.5) < 1e-9
    assert abs(smoothed[1] - 1.0) < 1e-9
    assert abs(smoothed[2] - 0.3) < 1e-9
