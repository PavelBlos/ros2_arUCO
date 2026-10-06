import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher'
)))

from camera_extrinsics_calibration import estimate_base_camera
from geometry_transforms import invert_transform, pose_to_matrix


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
