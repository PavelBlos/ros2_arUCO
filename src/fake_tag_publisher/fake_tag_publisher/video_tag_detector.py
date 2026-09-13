import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from fake_tag_interfaces.msg import TagDetection, TagDetectionArray, TagMapUpdate, TagMapAck
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import os
import yaml
import time
import threading
import math
from scipy.spatial.transform import Rotation as R

try:
    from .single_tag_pnp import solve_single_tag_ippe, validate_camera_calibration
    from .tag_registry import TagRegistry
    from .geometry_transforms import compute_midpoint_stamp, compute_latency_ms
except ImportError:
    from single_tag_pnp import solve_single_tag_ippe, validate_camera_calibration
    from tag_registry import TagRegistry
    from geometry_transforms import compute_midpoint_stamp, compute_latency_ms

class VideoTagDetector(Node):
    def __init__(self):
        super().__init__('video_tag_detector')
        
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('video_path', 'config/robot_drive.mp4', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('calibration_path', '/home/raspberry/arUco_termit/shared_config/camera_info.yaml', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('tag_map_path', '/home/raspberry/arUco_termit/shared_config/tags_config.yaml', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('marker_length', 0.100, ParameterDescriptor(dynamic_typing=True))  # Default 100 mm (0.100 m)
        self.declare_parameter('detection_rate', 30.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_100', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('loop_video', True, ParameterDescriptor(dynamic_typing=True))

        video_param = self.get_parameter('video_path')
        self.video_path = str(video_param.value) if video_param.value is not None else 'config/robot_drive.mp4'

        calib_param = self.get_parameter('calibration_path')
        self.calibration_path = str(calib_param.value) if calib_param.value is not None else '/home/raspberry/arUco_termit/shared_config/camera_info.yaml'

        tag_map_param = self.get_parameter('tag_map_path')
        self.tag_map_path = str(tag_map_param.value) if tag_map_param.value is not None else '/home/raspberry/arUco_termit/shared_config/tags_config.yaml'

        marker_param = self.get_parameter('marker_length')
        try:
            self.marker_length = float(marker_param.value)
        except (TypeError, ValueError):
            self.marker_length = 0.100

        rate_param = self.get_parameter('detection_rate')
        try:
            self.detection_rate = float(rate_param.value)
        except (TypeError, ValueError):
            self.detection_rate = 30.0

        aruco_dict_param = self.get_parameter('aruco_dictionary')
        self.aruco_dict_name = str(aruco_dict_param.value) if aruco_dict_param.value is not None else 'DICT_4X4_100'

        loop_param = self.get_parameter('loop_video')
        if loop_param.type_ == rclpy.Parameter.Type.STRING:
            self.loop_video = loop_param.value.lower() in ['true', '1', 'yes']
        elif loop_param.value is not None:
            self.loop_video = bool(loop_param.value)
        else:
            self.loop_video = True

        # Resolve relative video path
        if not os.path.isabs(self.video_path) and not str(self.video_path).isdigit() and not self.video_path.startswith('/dev/'):
            try:
                share_dir = get_package_share_directory('fake_tag_publisher')
                full_video_path = os.path.join(share_dir, self.video_path)
                if os.path.exists(full_video_path):
                    self.video_path = full_video_path
            except Exception as e:
                self.get_logger().warn(f"Could not resolve video path in share directory: {str(e)}")

        self.get_logger().info('Video Tag Detector node starting...')
        self.get_logger().info(f"Params: video={self.video_path}, calib={self.calibration_path}, tag_map={self.tag_map_path}, marker_size={self.marker_length}m")

        # 1. Load Tag Registry
        self.registry = None
        self.config_epoch = 1
        self.tag_map_revision = 1
        self.tag_map_sha256 = ""
        self.load_tag_registry()

        # 2. Camera Calibration (Strict)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_calibration_invalid = True
        self.load_calibration()

        # 3. Initialize OpenCV ArUco detector
        self.init_aruco_detector()

        # 4. Open video source
        self.cap = None
        self.open_source()

        # 5. ROS Publishers & Subscribers
        self.publisher_ = self.create_publisher(TagDetectionArray, '/fake_tag', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/camera/annotated_image/compressed', 10)

        # Transient-local QoS for receiving tag map updates and sending acks
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.ack_pub = self.create_publisher(TagMapAck, '/tag_map/ack', transient_qos)
        self.update_sub = self.create_subscription(
            TagMapUpdate, '/tag_map/updated', self.tag_map_update_callback, transient_qos
        )
        self.calibration_update_sub = self.create_subscription(
            String, '/camera_calibration/updated', self.camera_calibration_update_callback, 10
        )

        timer_period = 1.0 / self.detection_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def load_tag_registry(self):
        resolved_path = self.tag_map_path
        if not os.path.exists(resolved_path):
            candidates = [
                '/home/raspberry/arUco_termit/shared_config/tags_config.yaml',
                '/home/raspberry/arUco_termit/shared_config/tag_map.yaml'
            ]
            try:
                share_dir = get_package_share_directory('fake_tag_publisher')
                candidates.extend([
                    os.path.join(share_dir, 'config', 'tags_config.yaml'),
                    os.path.join(share_dir, 'config', 'tag_map.yaml')
                ])
            except Exception:
                pass
            for c in candidates:
                if os.path.exists(c):
                    resolved_path = c
                    break
        
        if os.path.exists(resolved_path):
            try:
                self.registry = TagRegistry(resolved_path)
                self.config_epoch = self.registry.config_epoch
                self.tag_map_revision = self.registry.revision
                self.tag_map_sha256 = self.registry.sha256
                self.aruco_dict_name = self.registry.dictionary_name
                self.get_logger().info(f"Loaded tag registry: rev={self.tag_map_revision}, sha={self.tag_map_sha256[:10]}")
            except Exception as e:
                self.get_logger().error(f"Failed to load tag registry from {resolved_path}: {e}")
        else:
            self.get_logger().warn(f"Tag registry file not found at {resolved_path}. Using default parameters.")

    def tag_map_update_callback(self, msg: TagMapUpdate):
        self.get_logger().info(f"Received TagMapUpdate: rev={msg.revision}, epoch={msg.config_epoch}, sha={msg.sha256[:10]}")
        try:
            self.load_tag_registry()
            # Send ACK
            ack_msg = TagMapAck()
            ack_msg.stamp = self.get_clock().now().to_msg()
            ack_msg.config_epoch = self.config_epoch
            ack_msg.detector_revision = self.tag_map_revision
            ack_msg.detector_sha256 = self.tag_map_sha256
            ack_msg.node_name = "video_tag_detector"
            ack_msg.status = "ok"
            ack_msg.error_message = ""
            self.ack_pub.publish(ack_msg)
            self.get_logger().info(f"Published TagMapAck for rev {self.tag_map_revision}")
        except Exception as e:
            ack_msg = TagMapAck()
            ack_msg.stamp = self.get_clock().now().to_msg()
            ack_msg.config_epoch = self.config_epoch
            ack_msg.detector_revision = self.tag_map_revision
            ack_msg.detector_sha256 = self.tag_map_sha256
            ack_msg.node_name = "video_tag_detector"
            ack_msg.status = "error"
            ack_msg.error_message = str(e)
            self.ack_pub.publish(ack_msg)

    def camera_calibration_update_callback(self, msg: String):
        self.get_logger().info(f"Reloading camera calibration from {msg.data}")
        self.load_calibration()

    def load_calibration(self):
        resolved_path = self.calibration_path
        if not os.path.exists(resolved_path):
            try:
                share_dir = get_package_share_directory('fake_tag_publisher')
                candidate = os.path.join(share_dir, 'config', 'camera_info.yaml')
                if os.path.exists(candidate):
                    resolved_path = candidate
            except Exception:
                pass

        if not os.path.exists(resolved_path):
            self.get_logger().error(f"Camera calibration file NOT FOUND: {resolved_path}. Live localization poses will be BLOCKED!")
            self.camera_calibration_invalid = True
            return

        try:
            with open(resolved_path, 'r', encoding='utf-8') as f:
                calib_data = yaml.safe_load(f)
            self.camera_matrix = np.array(calib_data['camera_matrix'], dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(calib_data['distortion_coefficients'], dtype=np.float64)
            
            w = calib_data.get('image_width', 640)
            h = calib_data.get('image_height', 480)
            ok, reason = validate_camera_calibration(self.camera_matrix, self.dist_coeffs, w, h)
            if ok:
                self.camera_calibration_invalid = False
                self.get_logger().info(f"Successfully loaded valid camera calibration from: {resolved_path}")
            else:
                self.camera_calibration_invalid = True
                self.get_logger().error(f"Camera calibration validation FAILED ({reason}) in {resolved_path}")
        except Exception as e:
            self.camera_calibration_invalid = True
            self.get_logger().error(f"Failed to load camera calibration: {e}")

    def init_aruco_detector(self):
        dict_id = getattr(cv2.aruco, self.aruco_dict_name, cv2.aruco.DICT_4X4_100)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.dictionary = cv2.aruco.Dictionary_get(dict_id)
            self.parameters = cv2.aruco.DetectorParameters_create()
            if hasattr(self.parameters, 'minMarkerPerimeterRate'):
                self.parameters.minMarkerPerimeterRate = 0.04
            if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detect_func = lambda img: cv2.aruco.detectMarkers(img, self.dictionary, parameters=self.parameters)
            self.get_logger().info(f"Initialized OpenCV Legacy ArUco detector ({self.aruco_dict_name})")
        else:
            self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            self.parameters = cv2.aruco.DetectorParameters()
            if hasattr(self.parameters, 'minMarkerPerimeterRate'):
                self.parameters.minMarkerPerimeterRate = 0.04
            if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            self.detect_func = lambda img: self.detector.detectMarkers(img)
            self.get_logger().info(f"Initialized OpenCV 4.7+ ArUco detector ({self.aruco_dict_name})")

    def open_source(self):
        import re
        match_dev = re.match(r'^/dev/video(\d+)$', str(self.video_path))
        is_cam = str(self.video_path).isdigit() or match_dev is not None or self.video_path in ['/dev/video0', '0']
        opened = False
        
        if is_cam:
            cam_idx = int(self.video_path) if str(self.video_path).isdigit() else (int(match_dev.group(1)) if match_dev else 0)
            self.get_logger().info(f"Opening camera index {cam_idx}...")
            self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(cam_idx)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                for _ in range(10):
                    ret, f = self.cap.read()
                    if ret and f is not None:
                        opened = True
                    time.sleep(0.03)
                if opened:
                    self.is_live = True
                    self.get_logger().info("Physical camera opened successfully")
                else:
                    self.cap.release()
                    self.cap = None
        else:
            self.cap = cv2.VideoCapture(self.video_path)
            if self.cap.isOpened():
                ret, test_f = self.cap.read()
                if ret and test_f is not None:
                    opened = True
                    self.is_live = False
                    self.get_logger().info(f"Opened video file: {self.video_path}")

        if not opened:
            self.get_logger().warn(f"Source {self.video_path} unavailable. Falling back to robot_drive.mp4")
            self.is_live = False
            self.loop_video = True
            fb_path = 'config/robot_drive.mp4'
            try:
                share_dir = get_package_share_directory('fake_tag_publisher')
                fb_full = os.path.join(share_dir, fb_path)
                if os.path.exists(fb_full):
                    fb_path = fb_full
            except Exception:
                pass
            self.cap = cv2.VideoCapture(fb_path)

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        # Record capture timestamps around blocking read
        t_before = time.time()
        ret, frame = self.cap.read()
        t_after = time.time()
        capture_stamp = compute_midpoint_stamp(t_before, t_after)

        if not ret or frame is None:
            if not getattr(self, 'is_live', False):
                if self.loop_video:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                if not ret or frame is None:
                    if not self.loop_video:
                        self.get_logger().info("Video finished.")
                        array_msg = TagDetectionArray()
                        array_msg.header.stamp = self.get_clock().now().to_msg()
                        array_msg.header.frame_id = "finished"
                        self.publisher_.publish(array_msg)
                        self.timer.cancel()
                    return
            else:
                return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detect_func(gray)

        # Deduplicate by tag_id keeping largest perimeter
        valid_corners = []
        valid_ids = []
        if ids is not None:
            best_by_id = {}
            for i in range(len(ids)):
                tag_id = int(ids[i][0])
                pts = corners[i][0]
                perim = cv2.arcLength(pts, True)
                if perim >= 40.0:
                    if tag_id not in best_by_id or perim > best_by_id[tag_id][1]:
                        best_by_id[tag_id] = (corners[i], perim)
            for tid, (c, _) in best_by_id.items():
                valid_corners.append(c)
                valid_ids.append([tid])

        # Prepare TagDetectionArray message
        array_msg = TagDetectionArray()
        # Header stamp: exact capture stamp converted to ROS Time
        sec = int(capture_stamp)
        nanosec = int((capture_stamp - sec) * 1e9)
        array_msg.header.stamp.sec = sec
        array_msg.header.stamp.nanosec = nanosec
        array_msg.header.frame_id = "camera_link"
        array_msg.config_epoch = self.config_epoch
        array_msg.tag_map_revision = self.tag_map_revision
        array_msg.tag_map_sha256 = self.tag_map_sha256

        detected_tag_infos = []

        if valid_ids:
            for i in range(len(valid_ids)):
                tag_id = int(valid_ids[i][0])
                corner_pts = valid_corners[i][0]

                # Determine tag status from registry
                tag_entry = self.registry.get_tag(tag_id) if self.registry else None
                if tag_entry:
                    tag_state = tag_entry.get("state", "unconfirmed")
                    tag_enabled = tag_entry.get("enabled", False)
                    marker_len = self.registry.get_marker_size_m(tag_id)
                else:
                    tag_state = "unknown"
                    tag_enabled = False
                    marker_len = (self.registry.default_marker_size_mm / 1000.0) if self.registry else self.marker_length

                # Solve IPPE PnP
                if not self.camera_calibration_invalid:
                    res = solve_single_tag_ippe(
                        corner_pts, marker_len, self.camera_matrix, self.dist_coeffs
                    )
                else:
                    res = {
                        "pose_valid": False,
                        "rejection_reason": "camera_calibration_invalid",
                        "reproj_err": 999.0,
                        "distance_m": 0.0,
                        "viewing_angle_deg": 90.0,
                        "ambiguity_status": 2,
                        "ambiguity_ratio": 1.0,
                        "marker_area_px": float(cv2.contourArea(corner_pts)),
                        "marker_perimeter_px": float(cv2.arcLength(corner_pts, True)),
                        "T_cameraRos_tag": np.eye(4)
                    }

                detection = TagDetection()
                detection.tag_id = tag_id
                detection.pose_valid = bool(res["pose_valid"])
                detection.rejection_reason = str(res["rejection_reason"])
                detection.reproj_err = float(res["reproj_err"])
                detection.marker_area_px = float(res["marker_area_px"])
                detection.marker_perimeter_px = float(res["marker_perimeter_px"])
                detection.distance_m = float(res["distance_m"])
                detection.viewing_angle_deg = float(res["viewing_angle_deg"])
                detection.ambiguity_status = int(res["ambiguity_status"])
                detection.ambiguity_ratio = float(res["ambiguity_ratio"])
                detection.marker_size_mm = float(marker_len * 1000.0)
                detection.corners_px = [float(val) for val in corner_pts.flatten()]

                if res["pose_valid"]:
                    T_ros = res["T_cameraRos_tag"]
                    pos = T_ros[:3, 3]
                    rot = R.from_matrix(T_ros[:3, :3]).as_quat()
                    detection.pose.position.x = float(pos[0])
                    detection.pose.position.y = float(pos[1])
                    detection.pose.position.z = float(pos[2])
                    detection.pose.orientation.x = float(rot[0])
                    detection.pose.orientation.y = float(rot[1])
                    detection.pose.orientation.z = float(rot[2])
                    detection.pose.orientation.w = float(rot[3])

                array_msg.detections.append(detection)
                detected_tag_infos.append({
                    "id": tag_id,
                    "state": tag_state,
                    "enabled": tag_enabled,
                    "pose_valid": res["pose_valid"],
                    "corners": corner_pts,
                    "dist": res["distance_m"],
                    "err": res["reproj_err"],
                    "reason": res["rejection_reason"]
                })

        # Calculate processing latency
        array_msg.processing_latency_ms = compute_latency_ms(capture_stamp)

        # Publish detections
        try:
            self.publisher_.publish(array_msg)
        except Exception:
            return

        # Annotate video frame with 5 colors
        for info in detected_tag_infos:
            pts = info["corners"].astype(np.int32).reshape((-1, 1, 2))
            
            # Color logic:
            if info["state"] == "unknown":
                color = (0, 0, 255)      # Red: unknown, independent of PnP quality
                status_text = "NEW"
            elif not info["pose_valid"]:
                color = (128, 128, 128)  # Gray: quality rejected
                status_text = f"REJ:{info['reason'][:10]}"
            elif info["state"] == "confirmed" and info["enabled"]:
                color = (0, 255, 0)      # Green: confirmed & enabled
                status_text = "CONF"
            elif info["state"] == "disabled" or not info["enabled"]:
                color = (0, 255, 255)    # Yellow: disabled
                status_text = "DIS"
            else:
                color = (255, 255, 0)    # Cyan: provisional/unconfirmed
                status_text = "PROV"

            cv2.polylines(frame, [pts], True, color, 2)
            c0 = info["corners"][0]
            label = f"ID:{info['id']} [{status_text}] d={info['dist']:.2f}m e={info['err']:.1f}px"
            cv2.putText(frame, label, (int(c0[0]), max(15, int(c0[1]) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        # Compress and publish annotated image
        ret_enc, jpeg_buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ret_enc:
            img_msg = CompressedImage()
            img_msg.header.stamp = array_msg.header.stamp
            img_msg.header.frame_id = "camera_link"
            img_msg.format = "jpeg"
            img_msg.data = jpeg_buf.tobytes()
            try:
                self.image_pub.publish(img_msg)
            except Exception:
                pass

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = VideoTagDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
