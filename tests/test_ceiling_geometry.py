import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher'
)))

from ceiling_geometry import (
    locate_ceiling_tag,
    marker_map_corners,
    project_ceiling,
    rotation2,
    solve_ceiling_frame,
    vertical_target_pixel,
)
from geometry_transforms import pose_to_matrix


@pytest.fixture
def ceiling_scene():
    K = np.array([[794.108, 0.0, 317.316],
                  [0.0, 798.507, 293.119],
                  [0.0, 0.0, 1.0]], dtype=float)
    distortion = np.array([-0.40029, -0.06553, -0.00376, 0.00322, 1.10535])
    T_base_cam = pose_to_matrix(
        0.012, -0.018, 0.025,
        -0.5740434, -1.4643497, -0.3830106,
    )
    ceiling_z = 2.1677
    robot_pose = np.array([0.104, -0.027, 0.939])
    tags = {
        '18': {
            'enabled': True, 'state': 'confirmed', 'size_mm': 100.0,
            'pose': {'x': 0.0, 'y': 0.0, 'z': ceiling_z,
                     'roll': math.pi, 'pitch': 0.0, 'yaw': 0.0},
        },
        '19': {
            'enabled': True, 'state': 'confirmed', 'size_mm': 100.0,
            'pose': {'x': 0.308, 'y': -0.073, 'z': ceiling_z,
                     'roll': math.pi, 'pitch': 0.0, 'yaw': -0.941},
        },
    }
    body_to_map = rotation2(robot_pose[2])
    camera_xy = robot_pose[:2] + body_to_map @ T_base_cam[:2, 3]
    gap = ceiling_z - T_base_cam[2, 3]
    detections = []
    for tag_id, tag in tags.items():
        points = marker_map_corners(tag, tag['size_mm'] / 1000.0)
        pixels = project_ceiling(points, gap, body_to_map, camera_xy, K, distortion, T_base_cam)
        detections.append({
            'tag_id': int(tag_id), 'corners_px': pixels.reshape(-1).tolist(),
            'marker_size_mm': tag['size_mm'], 'pose_valid': False,
        })
    return K, distortion, T_base_cam, ceiling_z, robot_pose, tags, detections


def test_single_and_multi_tag_recover_same_robot_pose(ceiling_scene):
    K, distortion, T_base_cam, _, robot_pose, tags, detections = ceiling_scene
    single = solve_ceiling_frame(detections[:1], tags, K, distortion, T_base_cam)
    multi = solve_ceiling_frame(detections, tags, K, distortion, T_base_cam)

    assert single['status'] == 'single_tag_ok'
    assert multi['status'] == 'multi_tag_ok'
    assert single['fused_base_pose'] == pytest.approx(robot_pose, abs=1e-7)
    assert multi['fused_base_pose'] == pytest.approx(robot_pose, abs=1e-7)
    assert multi['reproj_rms_px'] < 1e-6


def test_conflicting_known_marker_map_is_rejected(ceiling_scene):
    K, distortion, T_base_cam, _, _, tags, detections = ceiling_scene
    broken = {key: {**value, 'pose': dict(value['pose'])} for key, value in tags.items()}
    broken['19']['pose']['x'] += 0.20

    result = solve_ceiling_frame(detections, broken, K, distortion, T_base_cam)

    assert result['status'] == 'multi_tag_conflict'
    assert result['fused_base_pose'] is None
    assert set(result['rejected_ids']) == {'18', '19'}


def test_centimetre_scale_learned_map_uncertainty_is_accepted(ceiling_scene):
    K, distortion, T_base_cam, _, _, tags, detections = ceiling_scene
    learned = {key: {**value, 'pose': dict(value['pose'])} for key, value in tags.items()}
    # At this distance 20 mm shifts the joint corners by about 3 px, while
    # each marker independently still reports a mutually compatible pose.
    learned['19']['pose']['x'] += 0.020

    result = solve_ceiling_frame(detections, learned, K, distortion, T_base_cam)

    assert result['status'] == 'multi_tag_ok'
    assert set(result['inlier_ids']) == {'18', '19'}


def test_tilted_legacy_marker_is_not_used(ceiling_scene):
    K, distortion, T_base_cam, _, _, tags, detections = ceiling_scene
    tilted = {'18': {**tags['18'], 'pose': dict(tags['18']['pose'])}}
    tilted['18']['pose']['pitch'] = math.radians(8.0)

    result = solve_ceiling_frame(detections[:1], tilted, K, distortion, T_base_cam)

    assert result['status'] == 'ceiling_no_valid_tags'
    assert result['fused_base_pose'] is None
    assert 'tilted marker' in result['rejection_reasons']['18']


def test_learning_new_tag_and_vertical_target_are_frame_consistent(ceiling_scene):
    K, distortion, T_base_cam, ceiling_z, robot_pose, tags, detections = ceiling_scene
    learned = locate_ceiling_tag(
        detections[1], tags['19']['size_mm'] / 1000.0,
        robot_pose, K, distortion, T_base_cam,
    )
    target_pixel = vertical_target_pixel(K, distortion, T_base_cam, ceiling_z)
    expected_pixel = project_ceiling(
        np.asarray([robot_pose[:2]]), ceiling_z - T_base_cam[2, 3],
        rotation2(robot_pose[2]),
        robot_pose[:2] + rotation2(robot_pose[2]) @ T_base_cam[:2, 3],
        K, distortion, T_base_cam,
    )[0]

    assert (learned['x'], learned['y'], learned['yaw']) == pytest.approx(
        (tags['19']['pose']['x'], tags['19']['pose']['y'], tags['19']['pose']['yaw']), abs=1e-7
    )
    assert learned['roll'] == pytest.approx(math.pi)
    assert learned['pitch'] == 0.0
    assert target_pixel == pytest.approx(expected_pixel, abs=1e-7)
