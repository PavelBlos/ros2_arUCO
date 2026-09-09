"""
test_covisibility_graph.py - Unit tests for CovisibilityGraph.
"""

import numpy as np
import pytest
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher')))

from covisibility_graph import CovisibilityGraph
from geometry_transforms import pose_to_matrix

def test_record_co_visibility_edges():
    graph = CovisibilityGraph()

    # Frame 1: Tag 10 and Tag 20 visible together
    det10 = {
        "tag_id": 10,
        "pose_valid": True,
        "reproj_err": 0.05,
        "viewing_angle_deg": 5.0,
        "distance_m": 2.5,
        "T_cameraRos_tag": pose_to_matrix(0, 0.5, 2.5, 3.14, 0, 0)
    }
    det20 = {
        "tag_id": 20,
        "pose_valid": True,
        "reproj_err": 0.06,
        "viewing_angle_deg": 6.0,
        "distance_m": 2.5,
        "T_cameraRos_tag": pose_to_matrix(0, -0.5, 2.5, 3.14, 0, 0)
    }

    graph.record_frame_observations([det10, det20], robot_pose=(0.0, 0.0, 0.0), timestamp=100.0)

    assert graph.get_edge_observations_count(10, 20) == 1
    assert graph.get_edge_observations_count(20, 10) == 1
    assert graph.get_edge_observations_count(10, 30) == 0

def test_path_to_anchor():
    graph = CovisibilityGraph()

    T_dummy = np.eye(4)
    # Build a chain: 30 <-> 20 <-> 10 (anchor 10)
    det10 = {"tag_id": 10, "pose_valid": True, "T_cameraRos_tag": T_dummy}
    det20 = {"tag_id": 20, "pose_valid": True, "T_cameraRos_tag": T_dummy}
    det30 = {"tag_id": 30, "pose_valid": True, "T_cameraRos_tag": T_dummy}

    # Frame 1: 10 and 20 visible
    graph.record_frame_observations([det10, det20], (0, 0, 0), timestamp=1.0)
    # Frame 2: 20 and 30 visible
    graph.record_frame_observations([det20, det30], (0.5, 0, 0), timestamp=2.0)

    # Path from 30 to anchor 10
    path = graph.find_path_to_anchor(30, 10)
    assert path == [30, 20, 10]

    # Path from disconnected tag 99
    assert graph.find_path_to_anchor(99, 10) is None

def test_viewpoint_diversity_and_confidence():
    graph = CovisibilityGraph()
    T_dummy = np.eye(4)

    # Simulate 30 frames with diverse angles and robot positions
    for i in range(30):
        pos_x = 0.1 * np.cos(i * 0.2)
        pos_y = 0.1 * np.sin(i * 0.2)
        angle = 5.0 + 15.0 * (i / 30.0)  # Angle spans from 5 deg to 20 deg
        det = {
            "tag_id": 17,
            "pose_valid": True,
            "reproj_err": 0.08,
            "viewing_angle_deg": angle,
            "distance_m": 2.5,
            "T_cameraRos_tag": T_dummy
        }
        graph.record_frame_observations([det], (pos_x, pos_y, 0.0), timestamp=float(i))

    div = graph.get_viewpoint_diversity(17)
    conf = graph.get_tag_confidence(17)

    assert div > 0.40
    assert conf > 0.50

    diag = graph.get_diagnostics()
    assert "17" in diag["tag_confidences"]
    assert diag["total_tags_tracked"] == 1
