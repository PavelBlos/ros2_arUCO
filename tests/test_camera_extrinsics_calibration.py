import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher'
)))

from camera_extrinsics_calibration import (
    estimate_base_camera, estimate_base_camera_from_frames, validate_camera_mount,
)
from geometry_transforms import invert_transform, optical_to_ros_matrix, pose_to_matrix
from single_tag_pnp import get_marker_object_points


def test_stationary_marker_samples_recover_base_camera_transform():
    expected = pose_to_matrix(0.035, -0.012, 0.18, 0.01, -math.pi / 2.0, 0.04)
    map_base = pose_to_matrix(1.0, 2.0, 0.0, 0.0, 0.0, 0.3)
    map_tag = pose_to_matrix(1.0, 2.0, 2.2, math.pi, 0.0, 0.0)
    camera_tag = invert_transform(expected) @ invert_transform(map_base) @ map_tag
    samples = [camera_tag.copy() for _ in range(12)]

    solved, diagnostics = estimate_base_camera(samples, map_base, map_tag)

    assert solved == pytest.approx(expected, abs=1e-9)
    assert diagnostics["inliers"] == 12


def test_extrinsics_estimate_rejects_one_large_outlier():
    expected = pose_to_matrix(0.02, 0.01, 0.15, 0.0, -math.pi / 2.0, 0.0)
    map_base = pose_to_matrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    map_tag = pose_to_matrix(0.0, 0.0, 2.0, math.pi, 0.0, 0.0)
    camera_tag = invert_transform(expected) @ map_tag
    samples = [camera_tag.copy() for _ in range(11)]
    samples.append(pose_to_matrix(3.0, -2.0, 0.5, 1.0, 0.5, -0.5))

    solved, diagnostics = estimate_base_camera(samples, map_base, map_tag)

    assert solved == pytest.approx(expected, abs=1e-9)
    assert diagnostics["inliers"] == 11


def test_mount_validation_rejects_metre_scale_false_solution():
    with pytest.raises(ValueError, match='horizontal camera offset'):
        validate_camera_mount(pose_to_matrix(4.0, 0.0, 0.2, 0.0, -math.pi/2, 0.0))


def test_joint_mapped_tags_recover_camera_mount_without_single_tag_rotation():
    import cv2

    expected = pose_to_matrix(0.035, -0.012, 0.18, 0.01, -math.pi / 2.0, 0.04)
    map_base = pose_to_matrix(0.10, -0.20, 0.0, 0.0, 0.0, 0.25)
    tags = {
        '18': {'enabled': True, 'state': 'confirmed', 'size_mm': 100.0,
               'pose': {'x': 0.10, 'y': -0.20, 'z': 2.2,
                        'roll': math.pi, 'pitch': 0.0, 'yaw': 0.0}},
        '19': {'enabled': True, 'state': 'confirmed', 'size_mm': 100.0,
               'pose': {'x': 0.40, 'y': -0.05, 'z': 2.2,
                        'roll': math.pi, 'pitch': 0.0, 'yaw': 0.4}},
    }
    K = np.array([[520., 0., 320.], [0., 520., 240.], [0., 0., 1.]])
    distortion = np.zeros(5)
    map_cam_ros = map_base @ expected
    map_cam_opt = map_cam_ros @ optical_to_ros_matrix()
    cam_opt_map = invert_transform(map_cam_opt)
    rvec, _ = cv2.Rodrigues(cam_opt_map[:3, :3])
    detections = []
    for tag_id, tag in tags.items():
        local = get_marker_object_points(tag['size_mm'] / 1000.0)
        p = tag['pose']
        map_tag = pose_to_matrix(p['x'], p['y'], p['z'], p['roll'], p['pitch'], p['yaw'])
        world = (map_tag @ np.column_stack([local, np.ones(4)]).T).T[:, :3]
        pixels, _ = cv2.projectPoints(world, rvec, cam_opt_map[:3, 3], K, distortion)
        detections.append({'tag_id': int(tag_id), 'pose_valid': True,
                           'corners_px': pixels.reshape(-1).tolist()})
    frames = [{'detections': detections} for _ in range(12)]

    solved, diagnostics = estimate_base_camera_from_frames(
        frames, tags, map_base, K, distortion, prior_base_camera=expected
    )

    assert solved == pytest.approx(expected, abs=1e-5)
    assert diagnostics['tags_used'] == ['18', '19']
    assert diagnostics['reprojection_rms_px'] == pytest.approx(0.0, abs=1e-4)
