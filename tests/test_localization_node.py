"""
test_localization_node.py - Unit tests for LocalizationNode REST API, Stop-and-Reanchor, and SE(2) fusion.
"""

import json
import time
import math
import os
import sys
import numpy as np
import pytest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'fake_tag_publisher', 'fake_tag_publisher')))

from geometry_transforms import pose_to_matrix
from tag_registry import TagRegistry

# Mock ROS 2 if rclpy is not fully initialized in standard python test environment
for mod in [
    'rclpy', 'rclpy.node', 'rclpy.qos',
    'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
    'sensor_msgs', 'sensor_msgs.msg',
    'rcl_interfaces', 'rcl_interfaces.msg', 'rcl_interfaces.srv',
    'std_msgs', 'std_msgs.msg',
    'tf2_ros',
    'ament_index_python', 'ament_index_python.packages',
    'fake_tag_interfaces', 'fake_tag_interfaces.msg',
]:
    if mod not in sys.modules:
        sys.modules[mod] = MagicMock()

class MockNode:
    def __init__(self, node_name='localization_node'):
        self.node_name = node_name
        self.params = {}
    def declare_parameter(self, name, default=None, *args, **kwargs):
        self.params[name] = default
        p = MagicMock()
        p.value = default
        return p
    def get_parameter(self, name):
        p = MagicMock()
        p.value = self.params.get(name)
        return p
    def create_publisher(self, *args, **kwargs): return MagicMock()
    def create_subscription(self, *args, **kwargs): return MagicMock()
    def create_timer(self, *args, **kwargs): return MagicMock()
    def create_client(self, *args, **kwargs): return MagicMock()
    def get_logger(self):
        logger = MagicMock()
        logger.info = MagicMock()
        logger.warn = MagicMock()
        logger.error = MagicMock()
        return logger
    def get_clock(self):
        clk = MagicMock()
        clk.now.return_value.seconds_nanoseconds.return_value = (100, 0)
        return clk
    def destroy_node(self): pass

sys.modules['rclpy.node'].Node = MockNode
import rclpy
from localization_node import LocalizationNode, WebServerHandler

@pytest.fixture
def mock_node(tmp_path):

    # Create temporary tags_config.yaml
    cfg_file = tmp_path / "tags_config.yaml"
    cfg_data = {
        "config_epoch": 1,
        "tag_map_revision": 1,
        "anchor_tag_id": 17,
        "anchor_confirmed": False,
        "default_marker_size_mm": 100.0,
        "tags": {
            "tag_17": {
                "enabled": True,
                "state": "confirmed",
                "marker_size_m": 0.100,
                "pose": {"x": 0.0, "y": 0.0, "z": 2.5, "roll": 3.1416, "pitch": 0.0, "yaw": 0.0}
            }
        }
    }
    import yaml
    with open(cfg_file, "w", encoding="utf-8") as f:
        yaml.dump(cfg_data, f)

    with patch.object(LocalizationNode, 'start_web_server'):
        with patch.object(LocalizationNode, 'init_robot_api'):
            node = LocalizationNode()
            node.tag_registry = TagRegistry(str(cfg_file))
            node.tags_db = node.tag_registry.get_active_confirmed_tags()
            yield node
            node.destroy_node()

def test_dynamic_api_health(mock_node):
    # Setup mock server and handler
    handler = WebServerHandler.__new__(WebServerHandler)
    mock_server = MagicMock()
    mock_server.node = mock_node
    handler.server = mock_server

    # Set mock variables
    mock_node.git_commit = "b4ee324"
    mock_node.firmware_version = "FastAccelStepper-v2.0"
    mock_node.camera_extrinsics_status = "unverified"
    mock_node.is_nav_locked = False

    # Simulate GET /api/health
    sent_responses = []
    handler.send_response = lambda code: sent_responses.append(code)
    handler.send_header = lambda k, v: None
    handler.end_headers = lambda: None
    
    written_data = []
    handler.wfile = MagicMock()
    handler.wfile.write = lambda d: written_data.append(d)
    
    handler.path = "/api/health"
    handler.do_GET()

    assert sent_responses[-1] == 200
    payload = json.loads(written_data[-1].decode('utf-8'))

    assert payload["git_commit"] == "b4ee324"
    assert payload["firmware_version"] == "FastAccelStepper-v2.0"
    assert payload["camera_extrinsics"]["status"] == "unverified"
    assert payload["config_epoch"] == 1
    assert payload["tag_map_revision"] == 1
    assert payload["anchor_tag_id"] == 17
    assert payload["anchor_confirmed"] is False
    assert payload["is_nav_locked"] is False

def test_api_tags_crud_and_optimistic_locking(mock_node):
    handler = WebServerHandler.__new__(WebServerHandler)
    mock_server = MagicMock()
    mock_server.node = mock_node
    handler.server = mock_server

    sent_responses = []
    written_data = []
    handler.send_response = lambda code: sent_responses.append(code)
    handler.send_header = lambda k, v: None
    handler.end_headers = lambda: None
    handler.wfile = MagicMock()
    handler.wfile.write = lambda d: written_data.append(d)

    # 1. Save valid tag 25
    tag_data = {
        "enabled": True,
        "state": "confirmed",
        "marker_size_m": 0.100,
        "pose": {"x": 1.5, "y": 2.0, "z": 2.5, "roll": 3.1416, "pitch": 0.0, "yaw": 0.0}
    }
    save_payload = json.dumps({
        "tag_id": 25,
        "tag_data": tag_data,
        "expected_revision": 1
    }).encode('utf-8')

    handler.rfile = MagicMock()
    handler.rfile.read = lambda n: save_payload
    handler.headers = {'Content-Length': str(len(save_payload))}
    handler.path = "/api/tags/save"
    handler.do_POST()

    assert sent_responses[-1] == 200
    res = json.loads(written_data[-1].decode('utf-8'))
    assert res["status"] == "ok"
    assert res["revision"] == 2

    # 2. Conflict test: attempt save with outdated expected_revision=1 -> 409 Conflict
    conflict_payload = json.dumps({
        "tag_id": 25,
        "tag_data": tag_data,
        "expected_revision": 1
    }).encode('utf-8')
    handler.rfile.read = lambda n: conflict_payload
    handler.headers = {'Content-Length': str(len(conflict_payload))}
    handler.do_POST()

    assert sent_responses[-1] == 409
    err_res = json.loads(written_data[-1].decode('utf-8'))
    assert "conflict" in err_res["error"].lower()

    # 3. Anchor protection test: cannot delete confirmed anchor
    mock_node.tag_registry.set_anchor_tag(17, confirm=True)
    del_payload = json.dumps({
        "tag_id": 17,
        "expected_revision": mock_node.tag_registry.revision
    }).encode('utf-8')
    handler.rfile.read = lambda n: del_payload
    handler.headers = {'Content-Length': str(len(del_payload))}
    handler.path = "/api/tags/delete"
    handler.do_POST()

    assert sent_responses[-1] == 400
    del_err = json.loads(written_data[-1].decode('utf-8'))
    assert "cannot delete confirmed anchor" in del_err["error"].lower()

def test_stop_and_reanchor_procedure(mock_node):
    # When robot is moving and a large visual jump occurs (>0.15m),
    # navigation must lock and robot must halt without jumping dead reckoning!
    mock_node.autopilot_active = True
    mock_node.map_odom_initialized = True
    mock_node.delta_map_odom_x = 0.0
    mock_node.delta_map_odom_y = 0.0
    mock_node.fused_x = 0.0
    mock_node.fused_y = 0.0

    # Robot commanded to drive
    drives = []
    mock_node.drive_robot = lambda vx, vy, w: drives.append((vx, vy, w))

    # Mock detection resulting in visual pose at (1.0, 0.0) -> jump of 1.0m > 0.15m
    mock_fusion_res = {
        "status": "single_tag_ok",
        "fused_base_pose": (1.0, 0.0, 0.0),
        "inlier_ids": [17],
        "multi_tag_used": False
    }
    mock_node.fusion.process_frame = MagicMock(return_value=mock_fusion_res)

    mock_msg = MagicMock()
    mock_msg.header.frame_id = "camera_link"
    mock_msg.header.stamp.sec = 100
    mock_msg.header.stamp.nanosec = 0
    det = MagicMock()
    det.tag_id = 17
    det.pose.position.x = 0.0
    det.pose.position.y = 0.0
    det.pose.position.z = 2.5
    det.pose.orientation.x = 0.0
    det.pose.orientation.y = 0.0
    det.pose.orientation.z = 0.0
    det.pose.orientation.w = 1.0
    mock_msg.detections = [det]

    mock_node.tag_callback(mock_msg)

    # Check Stop-and-Reanchor initiated:
    assert mock_node.is_nav_locked is True
    assert mock_node.visual_jump_pending is True
    assert drives[-1] == (0.0, 0.0, 0.0)  # Robot halted
    assert mock_node.delta_map_odom_x == 0.0  # Dead reckoning held!

def test_exact_se2_fusion_cycle(mock_node):
    # Odometry reports robot at (1.0, 0.0, yaw=pi/2)
    mock_node.fused_initialized = True
    # map->odom translation (2.0, 3.0, yaw=0)
    mock_node.delta_map_odom_x = 2.0
    mock_node.delta_map_odom_y = 3.0
    mock_node.delta_map_odom_yaw = 0.0

    from collections import namedtuple
    OdomStub = namedtuple('Odom', ['timestamp', 'x', 'y', 'theta'])
    mock_node.odom_queue.put(OdomStub(timestamp=1.0, x=1.0, y=0.5, theta=0.1))

    mock_node.fusion_cycle()

    # Exact SE(2): x_mb = x_mo + x_ob, y_mb = y_mo + y_ob when yaw_mo = 0
    assert abs(mock_node.fused_x - 3.0) < 1e-4
    assert abs(mock_node.fused_y - 3.5) < 1e-4
    assert abs(mock_node.fused_yaw - 0.1) < 1e-4


def test_api_settings_get_and_post(mock_node, tmp_path):
    handler = WebServerHandler.__new__(WebServerHandler)
    mock_server = MagicMock()
    mock_server.node = mock_node
    handler.server = mock_server

    sent_responses = []
    written_data = []
    handler.send_response = lambda code: sent_responses.append(code)
    handler.send_header = lambda k, v: None
    handler.end_headers = lambda: None
    handler.wfile = MagicMock()
    handler.wfile.write = lambda d: written_data.append(d)

    # 1. GET /api/settings
    handler.path = "/api/settings"
    handler.do_GET()
    assert sent_responses[-1] == 200
    res = json.loads(written_data[-1].decode('utf-8'))
    assert res["status"] == "ok"
    assert "filter_alpha" in res["settings"]

    # 2. POST /api/settings
    update_payload = json.dumps({
        "settings": {
            "filter_alpha": 0.28,
            "ap_cruise_speed": 0.12
        }
    }).encode('utf-8')
    handler.rfile = MagicMock()
    handler.rfile.read = lambda n: update_payload
    handler.headers = {'Content-Length': str(len(update_payload))}
    handler.path = "/api/settings"
    handler.do_POST()

    assert sent_responses[-1] == 200
    res_post = json.loads(written_data[-1].decode('utf-8'))
    assert res_post["status"] == "ok"
    assert abs(res_post["settings"]["filter_alpha"] - 0.28) < 1e-4
    assert abs(mock_node.filter_alpha - 0.28) < 1e-4
    assert abs(mock_node.ap_cruise_speed - 0.12) < 1e-4

def test_api_anchor_wizard_status_and_confirm(mock_node):
    handler = WebServerHandler.__new__(WebServerHandler)
    mock_server = MagicMock()
    mock_server.node = mock_node
    handler.server = mock_server

    sent_responses = []
    written_data = []
    handler.send_response = lambda code: sent_responses.append(code)
    handler.send_header = lambda k, v: None
    handler.end_headers = lambda: None
    handler.wfile = MagicMock()
    handler.wfile.write = lambda d: written_data.append(d)

    # Mock visible detection
    mock_node.latest_detections = [{
        "tag_id": 17,
        "distance_m": 2.45,
        "reproj_err": 0.42,
        "viewing_angle_deg": 4.5
    }]
    mock_node.camera_extrinsics_status = "verified"

    # 1. GET /api/anchor/wizard_status
    handler.path = "/api/anchor/wizard_status"
    handler.do_GET()
    assert sent_responses[-1] == 200
    res = json.loads(written_data[-1].decode('utf-8'))
    assert res["status"] == "ok"
    assert res["anchor_tag_id"] == "17"
    assert res["anchor_confirmed"] is False
    assert res["ready_for_confirm"] is True
    assert len(res["visible_candidates"]) == 1

    # 2. POST /api/anchor/confirm
    confirm_payload = json.dumps({
        "anchor_tag_id": 17,
        "size_mm": 100.0,
        "ceiling_z_m": 2.5
    }).encode('utf-8')
    handler.rfile = MagicMock()
    handler.rfile.read = lambda n: confirm_payload
    handler.headers = {'Content-Length': str(len(confirm_payload))}
    handler.path = "/api/anchor/confirm"
    handler.do_POST()

    assert sent_responses[-1] == 200
    res_conf = json.loads(written_data[-1].decode('utf-8'))
    assert res_conf["status"] == "ok"
    assert mock_node.tag_registry.anchor_confirmed is True

def test_api_calibration_abort_and_confirm(mock_node):
    handler = WebServerHandler.__new__(WebServerHandler)
    mock_server = MagicMock()
    mock_server.node = mock_node
    handler.server = mock_server

    sent_responses = []
    written_data = []
    handler.send_response = lambda code: sent_responses.append(code)
    handler.send_header = lambda k, v: None
    handler.end_headers = lambda: None
    handler.wfile = MagicMock()
    handler.wfile.write = lambda d: written_data.append(d)

    from tag_calibration_wizard import WizardState, MotionAuthorityMode
    mock_node.wizard.start(25, 0.100)
    assert mock_node.wizard.state != WizardState.IDLE

    # POST /api/calibration/abort
    handler.path = "/api/calibration/abort"
    handler.rfile = MagicMock()
    handler.rfile.read = lambda n: b""
    handler.headers = {'Content-Length': '0'}
    handler.do_POST()

    assert sent_responses[-1] == 200
    assert mock_node.wizard.state == WizardState.ABORTED
    assert mock_node.motion_mgr.current_mode == MotionAuthorityMode.IDLE

    # Simulate reaching REVIEW state and confirm
    mock_node.wizard.state = WizardState.REVIEW
    mock_node.wizard.calibrated_tag_result = {
        "tag_id": 25,
        "state": "provisional",
        "enabled": True,
        "marker_size_m": 0.100,
        "pose": {"x": 1.0, "y": 2.0, "z": 2.5, "roll": 3.1416, "pitch": 0.0, "yaw": 0.0}
    }
    handler.path = "/api/calibration/confirm"
    handler.do_POST()
    assert sent_responses[-1] == 200
    assert mock_node.wizard.state == WizardState.COMPLETED
    assert "25" in mock_node.tag_registry.get_all_tags()

def test_publish_camera_tf(mock_node):
    mock_node.tf_broadcaster = MagicMock()
    mock_node.publish_camera_tf()
    assert mock_node.tf_broadcaster.sendTransform.called
    call_args = mock_node.tf_broadcaster.sendTransform.call_args[0][0]
    assert call_args.header.frame_id == "base_link"
    assert call_args.child_frame_id == "camera_link"
