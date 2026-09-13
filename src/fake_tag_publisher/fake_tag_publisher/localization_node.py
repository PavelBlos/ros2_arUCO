import rclpy
from rclpy.node import Node
try:
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
except (ImportError, AttributeError):
    QoSProfile = None
    DurabilityPolicy = None
    ReliabilityPolicy = None
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
try:
    from fake_tag_interfaces.msg import TagDetection, TagDetectionArray, TagMapUpdate, TagMapAck
except ImportError:
    from fake_tag_interfaces.msg import TagDetectionArray
    TagDetection = None
    TagMapUpdate = None
    TagMapAck = None

try:
    from .geometry_transforms import (
        normalize_angle, invert_transform, pose_to_matrix, matrix_to_pose,
        compute_map_to_odom_se2, compute_fused_pose_se2, smooth_map_to_odom_se2,
        optical_to_ros_matrix, ros_to_optical_matrix, map_velocity_to_body
    )
    from .tag_registry import TagRegistry
    from .multi_tag_fusion import MultiTagFusion, propagate_odometry_covariance
    from .tag_calibration_wizard import TagCalibrationWizard, MotionAuthorityManager, MotionAuthorityMode, WizardState
    from .covisibility_graph import CovisibilityGraph
except ImportError:
    from geometry_transforms import (
        normalize_angle, invert_transform, pose_to_matrix, matrix_to_pose,
        compute_map_to_odom_se2, compute_fused_pose_se2, smooth_map_to_odom_se2,
        optical_to_ros_matrix, ros_to_optical_matrix, map_velocity_to_body
    )
    from tag_registry import TagRegistry
    from multi_tag_fusion import MultiTagFusion, propagate_odometry_covariance
    from tag_calibration_wizard import TagCalibrationWizard, MotionAuthorityManager, MotionAuthorityMode, WizardState
    from covisibility_graph import CovisibilityGraph
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String, Empty
import tf2_ros
from ament_index_python.packages import get_package_share_directory
import os
import time
import math
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import threading
import json
import queue
from http.server import SimpleHTTPRequestHandler, HTTPServer
import socketserver

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # Буфер и слушатель TF для получения смещения камеры base_link -> camera_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Динамический транслятор TF для публикации map -> base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Загрузка базы данных меток из конфигурационного файла tags_config.yaml
        self.tags_db = {}
        self.tag_registry = None
        self.load_tags_config()

        # Camera Intrinsics and Extrinsics
        self.camera_matrix = np.array([[794.108, 0.0, 317.316], [0.0, 798.507, 293.119], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.dist_coeffs = np.array([-0.400, -0.0655, -0.00376, 0.00322, 1.105], dtype=np.float64)
        self.load_camera_calibration()

        self.camera_extrinsics_status = "unverified"
        # The original working mounting has image bottom toward robot +X.
        self.T_base_cam = pose_to_matrix(0.0, 0.0, 0.0, 0.0, -np.pi/2.0, 0.0)
        self.load_camera_extrinsics()

        # Fusion, Motion Mutex, Wizard, and Co-Visibility Graph
        self.fusion = MultiTagFusion()
        self.motion_mgr = MotionAuthorityManager()
        self.wizard = TagCalibrationWizard(
            motion_manager=self.motion_mgr,
            cx=float(self.camera_matrix[0, 2]),
            cy=float(self.camera_matrix[1, 2]),
        )
        self.covis_graph = CovisibilityGraph()

        # Detections caching and rate limits
        self.latest_detections = []
        self.latest_detections_stamp = 0.0
        self._last_conflict_warn_time = 0.0
        self.detector_revision = 0

        # Timers: 25 Hz for Calibration Wizard FSM, 20 Hz for Dynamic Camera TF broadcast
        self.wizard_timer = self.create_timer(0.04, self.wizard_timer_tick)
        self.camera_tf_timer = self.create_timer(0.05, self.publish_camera_tf)

        # Hot-reload Handshake
        if QoSProfile and DurabilityPolicy and ReliabilityPolicy:
            transient_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE
            )
        else:
            transient_qos = 10

        if TagMapUpdate:
            self.tag_map_pub = self.create_publisher(TagMapUpdate, '/tag_map/updated', transient_qos)
        else:
            self.tag_map_pub = None

        if TagMapAck:
            self.tag_map_ack_sub = self.create_subscription(TagMapAck, '/tag_map/ack', self.tag_map_ack_callback, transient_qos)
        else:
            self.tag_map_ack_sub = None
        self.last_tag_map_ack = None
        # TRANSIENT_LOCAL retains this startup announcement until the detector
        # subscribes, establishing an explicit epoch/revision/SHA handshake.
        self.publish_tag_map_update()

        # Safety and Versioning
        self.is_nav_locked = False
        self.visual_jump_pending = False
        self.firmware_version = "FastAccelStepper-v2.0"
        self.git_commit = self._get_git_commit()
        
        # Подписка на топик /fake_tag (сообщения типа TagDetectionArray)
        self.tag_sub = self.create_subscription(
            TagDetectionArray, '/fake_tag', self.tag_callback, 10)

        # Буфер и подписка на сжатое видео с камеры
        self.latest_jpeg_frame = None
        self.latest_frame_lock = threading.Lock()
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/annotated_image/compressed', self.image_callback, 10)

        # Публикатор оцененного положения робота в топик /estimated_pose
        self.pose_pub = self.create_publisher(
            PoseStamped, '/estimated_pose', 10)

        # Публикатор cmd_vel для тестового движения и переменные калибровки
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.test_drive_thread = None
        self.test_drive_active = False

        # Публикатор траектории маршрута (план)
        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.start_work_pub = self.create_publisher(Empty, '/start_work', 10)
        self.status_sub = self.create_subscription(String, '/follower_status', self.status_callback, 10)
        self.follower_status = "idle"

        # Клиент для динамического изменения параметров автопилота
        self.param_client = self.create_client(SetParameters, '/path_follower/set_parameters')

        # Состояние слияния одометрии и меток (Комплементарный фильтр)
        self.fused_x = 0.0
        self.fused_y = 0.0
        self.fused_z = 0.0
        self.fused_yaw = 0.0
        self.fused_initialized = False
        self.last_odom_msg_time = None
        self.last_detected_tags = []

        # Очередь и кольцевой буфер одометрии (Single UART owner -> ROS queue fusion)
        import queue
        import collections
        self.odom_queue = queue.Queue(maxsize=200)
        self.odom_history = collections.deque(maxlen=200)  # [(timestamp, odom_x, odom_y, odom_yaw)]
        self.delta_map_odom_x = 0.0
        self.delta_map_odom_y = 0.0
        self.delta_map_odom_yaw = 0.0
        self.map_odom_initialized = False
        self.outlier_count = 0
        self.odom_covariance = np.diag([0.02 ** 2, 0.02 ** 2, math.radians(2.0) ** 2])
        self._cov_odom_pose = None
        self._cov_odom_time = None

        # Таймер высокочастотного слияния одометрии и публикации позы (50 Гц)
        self.fusion_timer = self.create_timer(0.02, self.fusion_cycle)

        # Буферы для записи траекторий (сырая и отфильтрованная)
        self.raw_trajectory_x = []
        self.raw_trajectory_y = []
        self.raw_trajectory_z = []
        
        self.filtered_trajectory_x = []
        self.filtered_trajectory_y = []
        self.filtered_trajectory_z = []
        
        self.trajectory_timestamps = []

        # Состояние фильтра низких частот (EMA / Slerp)
        self.last_pos = None
        self.last_rot = None

        # Коэффициент фильтрации (EMA alpha)
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('filter_alpha', 0.15, ParameterDescriptor(dynamic_typing=True))
        alpha_param = self.get_parameter('filter_alpha')
        try:
            self.filter_alpha = float(alpha_param.value)
        except (TypeError, ValueError):
            self.filter_alpha = 0.15

        # Накопленная длина пути в реальном времени
        self.raw_path_length = 0.0
        self.filtered_path_length = 0.0

        # Состояние автопилота и маршрутизатора
        self.route_waypoints = []
        self.route_state = "idle"  # idle, running, paused, finished
        self.current_wp_idx = 0
        self.autopilot_thread = None
        self.autopilot_active = False

        # Параметры векторного контроллера (Cross-track follower & Corner speed profile)
        self.ap_cruise_speed = 0.14       # Крейсерская скорость по прямой (м/с)
        self.ap_max_lin = 0.14
        self.ap_min_lin = 0.03            # Минимальная скорость движения (м/с)
        self.ap_goal_tol = 0.03           # Радиус попадания в цель (м)
        self.ap_wp_tol = 0.05             # Радиус прохождения вершины угла для переключения сегмента (м)
        self.ap_kp_cross = 1.20           # Пропорциональный коэффициент возврата на траекторию (1/с)
        self.ap_v_cross_max = 0.08        # Максимальная скорость боковой коррекции (м/с)
        self.ap_brake_accel = 0.25        # Тормозное кинематическое замедление (м/с^2)
        self.ap_turn_factor = 0.65        # Множитель скорости прохождения углов
        self.ap_max_ang = 0.60            # Предельная угловая скорость вращения (рад/с)
        self.ap_kp_ang = 1.80             # Пропорциональный коэффициент ориентации
        self.ap_yaw_mode = "HOLD_INITIAL" # FREE, HOLD_INITIAL, PATH_TANGENT, FINAL_YAW
        self.ap_final_yaw = 0.0
        self.current_seg_idx = 0
        self.path_s_accum = [0.0]
        self.path_corner_speeds = []

        # Persistent physical settings. Load only after all defaults exist so the
        # file values are not silently overwritten later in __init__.
        self.wheel_diameter_mm = 70.0
        self.default_marker_size_mm = 100.0
        self.ceiling_height_m = 2.5
        self.load_runtime_settings()

        # Состояние слияния и фильтрации выбросов
        self.outlier_count = 0
        self.last_valid_tag_time = 0.0
        self.tracking_mode = "dead_reckoning"

        # Состояние питания обмоток шаговых двигателей (enabled / disabled)
        self.motor_power_state = "enabled"
        self.last_motion_cmd_time = time.time()
        
        # Таймер автоматического снятия тока с обмоток при отсутствии команд 2 секунды (5 Гц)
        self.power_watchdog_timer = self.create_timer(0.2, self.check_motor_power_watchdog)

        # Телеметрия и мониторинг свежести данных
        self.last_esp32_odom_time = 0.0
        self.last_pose_publish_time = 0.0
        
        # Журнал забегов (Run Logger в JSONL)
        self.run_logger_lock = threading.Lock()
        self.active_log_file = None
        self.active_run_id = None
        self.active_log_path = None

        # Аппаратное подключение к ESP32 для прямого управления моторами
        self.robot = None
        self.init_robot_api()

        # Запуск веб-сервера для real-time визуализации траектории
        self.web_port = 8080
        self.start_web_server()

        self.get_logger().info('Localization node started successfully (Multi-tag Data Fusion + ESP32 API + Autopilot Engine)')

    def start_run_logging(self, run_id=None):
        with self.run_logger_lock:
            if self.active_log_file:
                try:
                    self.active_log_file.close()
                except Exception:
                    pass
            if not run_id:
                run_id = f"run_{int(time.time())}"
            self.active_run_id = run_id
            log_dir = "/home/raspberry/arUco_termit/logs"
            os.makedirs(log_dir, exist_ok=True)
            self.active_log_path = os.path.join(log_dir, f"{run_id}.jsonl")
            self.active_log_file = open(self.active_log_path, "a", encoding="utf-8")
            self.get_logger().info(f"📝 Запись лога забега активна: {self.active_log_path}")
            return self.active_run_id

    def stop_run_logging(self):
        with self.run_logger_lock:
            if self.active_log_file:
                try:
                    self.active_log_file.flush()
                    self.active_log_file.close()
                except Exception:
                    pass
                p = self.active_log_path
                self.active_log_file = None
                self.get_logger().info(f"💾 Лог забега сохранен: {p}")
                return p
            return None

    def log_record(self, rec: dict):
        with self.run_logger_lock:
            if self.active_log_file:
                try:
                    self.active_log_file.write(json.dumps(rec) + "\n")
                    self.active_log_file.flush()
                except Exception:
                    pass

    def clear_trajectory_history(self):
        self.raw_trajectory_x.clear()
        self.raw_trajectory_y.clear()
        self.raw_trajectory_z.clear()
        self.filtered_trajectory_x.clear()
        self.filtered_trajectory_y.clear()
        self.filtered_trajectory_z.clear()
        self.trajectory_timestamps.clear()
        self.raw_path_length = 0.0
        self.filtered_path_length = 0.0
        self.notify_ui_event()
        self.get_logger().info("🧹 История траекторий на сервере очищена.")

    def _get_git_commit(self) -> str:
        if "ROBOT_RELEASE_COMMIT" in os.environ:
            return os.environ["ROBOT_RELEASE_COMMIT"]
        cand_manifests = [
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "manifest.json"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "manifest.json"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "manifest.json")
        ]
        for manifest_path in cand_manifests:
            if os.path.exists(manifest_path):
                try:
                    with open(manifest_path, "r", encoding="utf-8") as f:
                        m = json.load(f)
                        return m.get("git_commit", "unknown")
                except Exception:
                    pass
        try:
            import subprocess
            res = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=2)
            if res.returncode == 0 and res.stdout.strip():
                return res.stdout.strip()
        except Exception:
            pass
        return "unknown"

    def publish_camera_tf(self):
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'camera_link'
            t.transform.translation.x = float(self.T_base_cam[0, 3])
            t.transform.translation.y = float(self.T_base_cam[1, 3])
            t.transform.translation.z = float(self.T_base_cam[2, 3])
            q = R.from_matrix(self.T_base_cam[:3, :3]).as_quat()
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            self.tf_broadcaster.sendTransform(t)
        except Exception:
            pass

    def load_runtime_settings(self):
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        cand_paths = [
            '/home/raspberry/arUco_termit/shared_config/runtime_settings.yaml',
            os.path.join(curr_dir, 'runtime_settings.yaml'),
            os.path.join(curr_dir, '..', '..', '..', 'runtime_settings.yaml'),
            os.path.join(curr_dir, 'config', 'runtime_settings.yaml'),
            os.path.join(curr_dir, '..', 'config', 'runtime_settings.yaml'),
        ]
        self.runtime_settings_path = None
        for p in cand_paths:
            if os.path.exists(p):
                self.runtime_settings_path = os.path.abspath(p)
                break

        if not self.runtime_settings_path:
            if os.path.exists('/home/raspberry/arUco_termit/shared_config'):
                self.runtime_settings_path = '/home/raspberry/arUco_termit/shared_config/runtime_settings.yaml'
            else:
                self.runtime_settings_path = os.path.join(curr_dir, 'runtime_settings.yaml')

        if os.path.exists(self.runtime_settings_path):
            try:
                with open(self.runtime_settings_path, 'r', encoding='utf-8') as f:
                    cfg = yaml.safe_load(f) or {}
                if "filter_alpha" in cfg: self.filter_alpha = float(cfg["filter_alpha"])
                if "ap_cruise_speed" in cfg: self.ap_cruise_speed = float(cfg["ap_cruise_speed"])
                if "ap_max_lin" in cfg: self.ap_max_lin = float(cfg["ap_max_lin"])
                if "ap_min_lin" in cfg: self.ap_min_lin = float(cfg["ap_min_lin"])
                if "ap_goal_tol" in cfg: self.ap_goal_tol = float(cfg["ap_goal_tol"])
                if "ap_wp_tol" in cfg: self.ap_wp_tol = float(cfg["ap_wp_tol"])
                if "ap_kp_cross" in cfg: self.ap_kp_cross = float(cfg["ap_kp_cross"])
                if "ap_v_cross_max" in cfg: self.ap_v_cross_max = float(cfg["ap_v_cross_max"])
                if "ap_turn_factor" in cfg: self.ap_turn_factor = float(cfg["ap_turn_factor"])
                if "ap_max_ang" in cfg: self.ap_max_ang = float(cfg["ap_max_ang"])
                if "ap_kp_ang" in cfg: self.ap_kp_ang = float(cfg["ap_kp_ang"])
                if "ap_yaw_mode" in cfg: self.ap_yaw_mode = str(cfg["ap_yaw_mode"])
                if "wheel_diameter_mm" in cfg: self.wheel_diameter_mm = float(cfg["wheel_diameter_mm"])
                if "default_marker_size_mm" in cfg: self.default_marker_size_mm = float(cfg["default_marker_size_mm"])
                if "ceiling_height_m" in cfg: self.ceiling_height_m = float(cfg["ceiling_height_m"])
                self.get_logger().info(f"Loaded runtime settings from {self.runtime_settings_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed loading runtime settings: {e}")

    def save_runtime_settings(self):
        cfg = self.get_runtime_settings()
        try:
            target = getattr(self, 'runtime_settings_path', None)
            if not target:
                curr_dir = os.path.dirname(os.path.abspath(__file__))
                target = os.path.join(curr_dir, 'runtime_settings.yaml')
            os.makedirs(os.path.dirname(os.path.abspath(target)), exist_ok=True)
            tmp = target + '.tmp'
            with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
                yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=True)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, target)
            self.get_logger().info(f"Saved runtime settings to {target}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed saving runtime settings: {e}")
            return False

    def get_runtime_settings(self) -> dict:
        return {
            "filter_alpha": float(getattr(self, 'filter_alpha', 0.15)),
            "ap_cruise_speed": float(getattr(self, 'ap_cruise_speed', 0.14)),
            "ap_max_lin": float(getattr(self, 'ap_max_lin', 0.14)),
            "ap_min_lin": float(getattr(self, 'ap_min_lin', 0.03)),
            "ap_goal_tol": float(getattr(self, 'ap_goal_tol', 0.03)),
            "ap_wp_tol": float(getattr(self, 'ap_wp_tol', 0.05)),
            "ap_kp_cross": float(getattr(self, 'ap_kp_cross', 1.20)),
            "ap_v_cross_max": float(getattr(self, 'ap_v_cross_max', 0.08)),
            "ap_turn_factor": float(getattr(self, 'ap_turn_factor', 0.65)),
            "ap_max_ang": float(getattr(self, 'ap_max_ang', 0.60)),
            "ap_kp_ang": float(getattr(self, 'ap_kp_ang', 1.80)),
            "ap_yaw_mode": str(getattr(self, 'ap_yaw_mode', 'HOLD_INITIAL')),
            "wheel_diameter_mm": float(getattr(self, 'wheel_diameter_mm', 70.0)),
            "default_marker_size_mm": float(getattr(self, 'default_marker_size_mm', 100.0)),
            "ceiling_height_m": float(getattr(self, 'ceiling_height_m', 2.5)),
        }

    def update_runtime_settings(self, new_settings: dict):
        if not isinstance(new_settings, dict):
            raise ValueError("settings must be an object")
        if "settings" in new_settings and isinstance(new_settings["settings"], dict):
            new_settings = new_settings["settings"]

        float_keys = {
            'filter_alpha', 'ap_cruise_speed', 'ap_max_lin', 'ap_min_lin',
            'ap_goal_tol', 'ap_wp_tol', 'ap_kp_cross', 'ap_v_cross_max',
            'ap_turn_factor', 'ap_max_ang', 'ap_kp_ang', 'wheel_diameter_mm',
            'default_marker_size_mm', 'ceiling_height_m'
        }
        str_keys = {'ap_yaw_mode'}

        validated = {}
        for k, v in new_settings.items():
            if k in float_keys:
                val = float(v)
                if not math.isfinite(val):
                    raise ValueError(f"{k} must be finite")
                limits = {
                    'wheel_diameter_mm': (20.0, 300.0),
                    'default_marker_size_mm': (20.0, 1000.0),
                    'ceiling_height_m': (0.2, 20.0),
                    'filter_alpha': (0.01, 1.0),
                }
                if k in limits and not (limits[k][0] <= val <= limits[k][1]):
                    raise ValueError(f'{k} outside allowed range {limits[k]}')
                validated[k] = val
            elif k in str_keys:
                validated[k] = str(v)
        old_wheel = self.wheel_diameter_mm
        for k, val in validated.items():
            setattr(self, k, val)
        if self.robot and self.wheel_diameter_mm != old_wheel:
            self.robot.stop()
            self.robot.set_wheel_diameter_mm(self.wheel_diameter_mm)
            self.map_odom_initialized = False
            self.odom_covariance = np.diag([0.02 ** 2, 0.02 ** 2, math.radians(2.0) ** 2])
        if not self.save_runtime_settings():
            raise OSError("could not persist runtime settings")
        if self.tag_registry and (
            'default_marker_size_mm' in validated or 'ceiling_height_m' in validated
        ):
            self.tag_registry.update_defaults(self.default_marker_size_mm, self.ceiling_height_m)
            self.tags_db = self.tag_registry.get_active_confirmed_tags()
            self.publish_tag_map_update()
        return self.get_runtime_settings()

    def wizard_timer_tick(self):
        """25 Hz periodic tick driving the Calibration Wizard FSM independently of camera frames."""
        if not self.wizard or self.wizard.state in (WizardState.IDLE, WizardState.COMPLETED, WizardState.ABORTED):
            return

        now_t = time.monotonic()

        if (now_t - self.latest_detections_stamp) < 0.25:
            current_detections = self.latest_detections
        else:
            current_detections = []

        wheels_stopped = True
        if self.autopilot_active or self.test_drive_active:
            wheels_stopped = False
        elif self.robot and getattr(self.robot, 'is_connected', False):
            sp1 = abs(getattr(self.robot, 'current_speed_m1', 0))
            sp2 = abs(getattr(self.robot, 'current_speed_m2', 0))
            sp3 = abs(getattr(self.robot, 'current_speed_m3', 0))
            if sp1 > 15 or sp2 > 15 or sp3 > 15:
                wheels_stopped = False

        # camera_extrinsics.yaml is the only source of this transform. Looking
        # it up from TF here can pick up an old publisher and ignore new values.
        T_base_camera = self.T_base_cam

        wiz_state, cmd_vel_tuple, status_msg = self.wizard.update(
            current_detections,
            wheels_stopped,
            (self.fused_x, self.fused_y, self.fused_yaw),
            self.camera_matrix,
            self.dist_coeffs,
            T_base_camera,
            now=now_t
        )

        if cmd_vel_tuple is not None:
            self.drive_robot(cmd_vel_tuple[0], cmd_vel_tuple[1], cmd_vel_tuple[2], source_mode=MotionAuthorityMode.CALIBRATION)

        if wiz_state == WizardState.COMPLETED and self.wizard.calibrated_tag_result:
            res_tag = self.wizard.calibrated_tag_result
            if self.tag_registry:
                self.tag_registry.set_tag(res_tag["tag_id"], res_tag)
                self.tags_db = self.tag_registry.get_active_confirmed_tags()
                self.publish_tag_map_update()
                self.notify_ui_event()
                self.get_logger().info(f"🎉 Calibration Wizard successfully registered tag {res_tag['tag_id']}")

    def load_camera_calibration(self):
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        cand_paths = [
            '/home/raspberry/arUco_termit/shared_config/camera_info.yaml',
            os.path.join(curr_dir, 'config', 'camera_info.yaml'),
            os.path.join(curr_dir, '..', 'config', 'camera_info.yaml'),
            os.path.join(curr_dir, 'camera_info.yaml'),
            os.path.join(curr_dir, 'camera_calibration.yaml')
        ]
        for p in cand_paths:
            if os.path.exists(p):
                try:
                    with open(p, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                    self.camera_matrix = np.array(data['camera_matrix'], dtype=np.float64).reshape(3, 3)
                    self.dist_coeffs = np.array(data['distortion_coefficients'], dtype=np.float64)
                    if self.camera_matrix.shape != (3, 3) or not np.all(np.isfinite(self.camera_matrix)):
                        raise ValueError("camera_matrix must be a finite 3x3 matrix")
                    self.camera_calibration_path = os.path.abspath(p)
                    self.camera_calibration_valid = True
                    self.get_logger().info(f"Loaded camera calibration from {p}")
                    return
                except Exception as e:
                    self.get_logger().warn(f"Failed loading camera calibration from {p}: {e}")
        self.camera_calibration_path = "built-in fallback"
        self.camera_calibration_valid = False
        self.get_logger().error("No valid camera_info.yaml found; tag calibration is blocked")

    def load_camera_extrinsics(self):
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        cand_paths = [
            '/home/raspberry/arUco_termit/shared_config/camera_extrinsics.yaml',
            os.path.join(curr_dir, 'camera_extrinsics.yaml'),
            os.path.join(curr_dir, '..', '..', '..', 'camera_extrinsics.yaml'),
            os.path.join(curr_dir, 'config', 'camera_extrinsics.yaml'),
            os.path.join(curr_dir, '..', 'config', 'camera_extrinsics.yaml'),
        ]
        ext_path = None
        for p in cand_paths:
            if os.path.exists(p):
                ext_path = os.path.abspath(p)
                break

        if ext_path and os.path.exists(ext_path):
            try:
                with open(ext_path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                self.camera_extrinsics_status = data.get("status", "unverified")
                trans = data.get("translation", {}) if isinstance(data.get("translation"), dict) else {}
                rot = data.get("rotation_rpy_rad", {}) if isinstance(data.get("rotation_rpy_rad"), dict) else {}
                x = float(data.get("x", trans.get("x", 0.0)))
                y = float(data.get("y", trans.get("y", 0.0)))
                z = float(data.get("z", trans.get("z", 0.0)))
                roll = float(data.get("roll", rot.get("roll", 0.0)))
                pitch = float(data.get("pitch", rot.get("pitch", -np.pi / 2.0)))
                yaw = float(data.get("yaw", rot.get("yaw", np.pi / 2.0)))
                self.T_base_cam = pose_to_matrix(x, y, z, roll, pitch, yaw)
                self.camera_extrinsics_path = ext_path
                self.get_logger().info(f"Loaded camera extrinsics from {ext_path} (status: {self.camera_extrinsics_status})")
                return
            except Exception as e:
                self.get_logger().warn(f"Failed to parse camera extrinsics: {e}")

        self.camera_extrinsics_status = "unverified"
        self.T_base_cam = pose_to_matrix(0.0, 0.0, 0.0, 0.0, -np.pi / 2.0, np.pi / 2.0)
        self.camera_extrinsics_path = os.path.join(curr_dir, 'camera_extrinsics.yaml')

    def publish_tag_map_update(self):
        if getattr(self, 'tag_map_pub', None) and TagMapUpdate:
            try:
                msg = TagMapUpdate()
                now_t = time.time()
                sec = int(now_t)
                nanosec = int((now_t - sec) * 1e9)
                msg.stamp.sec = sec
                msg.stamp.nanosec = nanosec
                msg.config_epoch = int(self.tag_registry.config_epoch)
                msg.revision = int(self.tag_registry.revision)
                msg.sha256 = str(self.tag_registry.sha256)
                msg.node_name = "localization_node"
                msg.config_type = "tags_config"
                self.tag_map_pub.publish(msg)
                self.get_logger().info(f"Published /tag_map/updated (rev: {msg.revision})")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish /tag_map/updated: {e}")

    def tag_map_ack_callback(self, msg):
        rev = getattr(msg, 'detector_revision', getattr(msg, 'tag_map_revision', 0))
        status = getattr(msg, 'status', getattr(msg, 'ack_status', 'unknown'))
        err = getattr(msg, 'error_message', getattr(msg, 'error_msg', ''))
        self.last_tag_map_ack = {
            "config_epoch": getattr(msg, 'config_epoch', 0),
            "detector_revision": rev,
            "detector_sha256": getattr(msg, 'detector_sha256', ''),
            "status": status,
            "error_message": err
        }
        self.get_logger().info(f"Received /tag_map/ack (rev: {rev}, status: {status})")

    def load_tags_config(self):
        try:
            curr_dir = os.path.dirname(os.path.abspath(__file__))
            cand_paths = [
                '/home/raspberry/arUco_termit/shared_config/tags_config.yaml',
                os.path.join(curr_dir, 'tags_config.yaml'),
                os.path.join(curr_dir, '..', '..', '..', 'tags_config.yaml'),
                os.path.join(curr_dir, 'config', 'tags_config.yaml'),
                os.path.join(curr_dir, '..', 'config', 'tags_config.yaml'),
            ]
            try:
                share_dir = get_package_share_directory('fake_tag_publisher')
                cand_paths.append(os.path.join(share_dir, 'config', 'tags_config.yaml'))
            except Exception:
                pass

            config_path = None
            for p in cand_paths:
                if os.path.exists(p):
                    config_path = os.path.abspath(p)
                    break

            if not config_path:
                self.get_logger().error("tags_config.yaml not found!")
                return

            self.tag_registry = TagRegistry(config_path)
            self.tags_db = self.tag_registry.get_active_confirmed_tags()
            self.get_logger().info(f"Loaded {len(self.tags_db)} active tags from {config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {str(e)}")

    def image_callback(self, msg):
        with self.latest_frame_lock:
            self.latest_jpeg_frame = bytes(msg.data)

    def tag_callback(self, msg):
        try:
            # Check finished signal
            if msg.header.frame_id == "finished":
                self.get_logger().info("Received finished signal. Saving trajectory plot and shutting down...")
                self.save_trajectory_and_shutdown()
                return

            detections = msg.detections
            if not detections:
                return

            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Lookup odometry at exact frame midpoint capture stamp
            odom_at_cam = self.lookup_odom_at(stamp_sec)
            if odom_at_cam is None:
                odom_x_c, odom_y_c, odom_yaw_c = self.fused_x, self.fused_y, self.fused_yaw
            else:
                odom_x_c, odom_y_c, odom_yaw_c = odom_at_cam

            # Use the validated persistent extrinsics directly. This node also
            # publishes the same matrix to TF for other ROS consumers.
            T_base_camera = self.T_base_cam

            # Convert detections to standardized dicts
            det_dicts = []
            for d in detections:
                corners = list(d.corners_px) if hasattr(d, 'corners_px') else []
                pos = [d.pose.position.x, d.pose.position.y, d.pose.position.z]
                rot = [d.pose.orientation.x, d.pose.orientation.y, d.pose.orientation.z, d.pose.orientation.w]
                T_ros = np.eye(4, dtype=np.float64)
                T_ros[:3, :3] = R.from_quat(rot).as_matrix()
                T_ros[:3, 3] = pos

                det_dicts.append({
                    "tag_id": int(d.tag_id),
                    "pose_valid": bool(getattr(d, 'pose_valid', True)),
                    "rejection_reason": getattr(d, 'rejection_reason', ''),
                    "reproj_err": float(getattr(d, 'reproj_err', 0.5)),
                    "marker_area_px": float(getattr(d, 'marker_area_px', 1000.0)),
                    "marker_perimeter_px": float(getattr(d, 'marker_perimeter_px', 120.0)),
                    "distance_m": float(getattr(d, 'distance_m', 2.5)),
                    "viewing_angle_deg": float(getattr(d, 'viewing_angle_deg', 0.0)),
                    "ambiguity_status": int(getattr(d, 'ambiguity_status', 0)),
                    "marker_size_mm": float(getattr(d, 'marker_size_mm', 100.0)),
                    "corners_px": corners,
                    "pose_position": pos,
                    "pose_orientation": rot,
                    "T_cameraRos_tag": T_ros
                })

            # Cache latest detections for 25 Hz Calibration Wizard timer
            self.latest_detections = det_dicts
            self.latest_detections_stamp = time.monotonic()

            # Check robot motion state
            is_moving = self.autopilot_active or self.test_drive_active
            if self.robot and getattr(self.robot, 'is_connected', False):
                sp1 = abs(getattr(self.robot, 'current_speed_m1', 0))
                sp2 = abs(getattr(self.robot, 'current_speed_m2', 0))
                sp3 = abs(getattr(self.robot, 'current_speed_m3', 0))
                if sp1 > 15 or sp2 > 15 or sp3 > 15:
                    is_moving = True

            # Process through MultiTagFusion with revision handshake
            active_tags = self.tag_registry.get_active_confirmed_tags() if self.tag_registry else self.tags_db
            frame_epoch = int(getattr(msg, 'config_epoch', 0))
            frame_rev = int(getattr(msg, 'tag_map_revision', 0))
            frame_sha = str(getattr(msg, 'tag_map_sha256', ''))
            det_rev = frame_rev
            if det_rev is not None:
                try:
                    self.detector_revision = int(det_rev)
                except Exception:
                    pass

            fusion_res = self.fusion.process_frame(
                det_dicts,
                active_tags,
                self.camera_matrix,
                self.dist_coeffs,
                T_base_camera,
                (odom_x_c, odom_y_c, odom_yaw_c),
                pred_odom_cov=self.odom_covariance.copy(),
                expected_epoch=self.tag_registry.config_epoch if self.tag_registry else None,
                expected_revision=self.tag_registry.revision if self.tag_registry else None,
                expected_sha256=self.tag_registry.sha256 if self.tag_registry else None,
                frame_epoch=frame_epoch,
                frame_revision=frame_rev,
                frame_sha256=frame_sha
            )

            # Record in Co-Visibility Graph
            self.covis_graph.record_frame_observations(
                det_dicts, (self.fused_x, self.fused_y, self.fused_yaw), stamp_sec
            )

            # Handle Fusion Result
            if fusion_res.get("fused_base_pose") is not None:
                vis_x, vis_y, vis_yaw = fusion_res["fused_base_pose"]
                jump_dist = float(math.hypot(vis_x - self.fused_x, vis_y - self.fused_y))

                # Stop-and-Reanchor gate
                if jump_dist > 0.15 and is_moving and self.map_odom_initialized:
                    self.is_nav_locked = True
                    self.visual_jump_pending = True
                    self.drive_robot(0.0, 0.0, 0.0)
                    self.get_logger().warn(
                        f"⚠️ Visual jump {jump_dist:.2f}m > 0.15m while in motion! "
                        f"Locking navigation and halting robot for Stop-and-Reanchor."
                    )
                    return
                else:
                    target_x_mo, target_y_mo, target_yaw_mo = compute_map_to_odom_se2(
                        vis_x, vis_y, vis_yaw, odom_x_c, odom_y_c, odom_yaw_c
                    )
                    if not self.map_odom_initialized:
                        self.delta_map_odom_x = target_x_mo
                        self.delta_map_odom_y = target_y_mo
                        self.delta_map_odom_yaw = target_yaw_mo
                        self.map_odom_initialized = True
                    else:
                        self.delta_map_odom_x, self.delta_map_odom_y, self.delta_map_odom_yaw = smooth_map_to_odom_se2(
                            self.delta_map_odom_x, self.delta_map_odom_y, self.delta_map_odom_yaw,
                            target_x_mo, target_y_mo, target_yaw_mo,
                            alpha=self.filter_alpha
                        )

                    if self.visual_jump_pending and not is_moving:
                        self.visual_jump_pending = False
                        self.is_nav_locked = False
                        self.get_logger().info("✅ Stop-and-Reanchor completed cleanly at zero velocity. Navigation unlocked.")

                self.tracking_mode = "aruco_multi" if fusion_res.get("multi_tag_used") else "aruco_single"
                self.odom_covariance = np.asarray(fusion_res.get("covariance", self.odom_covariance), dtype=float)
                self.last_valid_tag_time = time.time()

                # Update trajectory
                if len(self.raw_trajectory_x) > 0:
                    dx = vis_x - self.raw_trajectory_x[-1]
                    dy = vis_y - self.raw_trajectory_y[-1]
                    self.raw_path_length += math.hypot(dx, dy)
                self.raw_trajectory_x.append(vis_x)
                self.raw_trajectory_y.append(vis_y)
                self.raw_trajectory_z.append(0.0)
                self.trajectory_timestamps.append(stamp_sec)

                self.last_detected_tags = fusion_res.get("inlier_ids", [d.tag_id for d in detections])
                self.publish_fused_pose(msg.header.stamp)

            elif fusion_res.get("status") == "multi_tag_conflict":
                now_t = time.time()
                if now_t - self._last_conflict_warn_time >= 1.0:
                    self.get_logger().warn("⚠️ Multi-tag conflict detected between visible tags! Holding dead reckoning.")
                    self._last_conflict_warn_time = now_t

        except Exception as e:
            self.get_logger().error(f"Error in tag_callback: {str(e)}")

    def start_web_server(self):
        try:
            self.server = ThreadedHTTPServer(('0.0.0.0', self.web_port), WebServerHandler, self)
            self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.server_thread.start()
            self.get_logger().info(f"🌐 Real-time Web UI started at http://localhost:{self.web_port}/")
        except Exception as e:
            self.get_logger().error(f"Failed to start Web UI server: {str(e)}")

    def init_robot_api(self):
        """Прямая аппаратная интеграция с ESP32 через TermitRobotAPI"""
        try:
            try:
                from .termit_api import TermitRobotAPI, RobotConfig, HoldMode
            except (ImportError, ValueError):
                from termit_api import TermitRobotAPI, RobotConfig, HoldMode
                
            config = RobotConfig(
                wheel_radius=float(self.wheel_diameter_mm) / 2000.0,
                base_radius=0.122,
                steps_per_rev=1600,
                watchdog_timeout_ms=1500,
                max_linear_speed=0.15,
                max_motor_speed_steps=1100,
                min_start_speed_steps=120.0,
                max_wheel_accel_steps=1600.0
            )
            self.robot = TermitRobotAPI(config)
            hardware_by_id = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
            ports = self.robot.list_available_ports()
            if os.path.exists(hardware_by_id):
                port = hardware_by_id
            elif ports:
                port = '/dev/ttyUSB0' if '/dev/ttyUSB0' in ports else ports[0]
            else:
                port = None

            if port:
                self.get_logger().info(f"Connecting TermitRobotAPI to ESP32 on {port}...")
                self.robot.connect(port=port)
                self.robot.set_holding_mode(HoldMode.CONTINUOUS_HOLD)
                self.robot.add_odometry_callback(self.on_esp32_odometry)
                self.get_logger().info(f"✅ TermitRobotAPI successfully connected to ESP32 on {port} (Continuous Hold enabled)")
            else:
                self.get_logger().warn("No serial ports found for ESP32. Running with ROS cmd_vel fallback.")
        except Exception as e:
            self.get_logger().warn(f"TermitRobotAPI initialization note: {str(e)}")
            self.robot = None

    def on_esp32_odometry(self, odom):
        """Коллбэк прямой одометрии шаговых двигателей от TermitRobotAPI (UART-поток -> неблокирующая очередь)"""
        self.last_esp32_odom_time = time.time()
        self.last_esp32_odom = odom
        try:
            self.odom_queue.put_nowait(odom)
        except Exception:
            pass

    def lookup_odom_at(self, target_time: float):
        """Интерполяция позы одометрии по кольцевому буферу на момент времени target_time кадра камеры."""
        if not self.odom_history:
            return None
        if target_time <= self.odom_history[0][0]:
            return self.odom_history[0][1], self.odom_history[0][2], self.odom_history[0][3]
        if target_time >= self.odom_history[-1][0]:
            return self.odom_history[-1][1], self.odom_history[-1][2], self.odom_history[-1][3]

        hist = list(self.odom_history)
        for i in range(len(hist) - 1):
            t1, x1, y1, th1 = hist[i]
            t2, x2, y2, th2 = hist[i + 1]
            if t1 <= target_time <= t2:
                ratio = (target_time - t1) / max(1e-6, t2 - t1)
                ix = x1 + ratio * (x2 - x1)
                iy = y1 + ratio * (y2 - y1)
                dth = (th2 - th1 + np.pi) % (2.0 * np.pi) - np.pi
                ith = (th1 + ratio * dth + np.pi) % (2.0 * np.pi) - np.pi
                return ix, iy, ith
        return hist[-1][1], hist[-1][2], hist[-1][3]

    def fusion_cycle(self):
        """Высокочастотный цикл обработки одометрии и слияния с картой (50 Гц) в главном потоке ROS."""
        while not self.odom_queue.empty():
            try:
                odom = self.odom_queue.get_nowait()
                self.odom_history.append((odom.timestamp, odom.x, odom.y, odom.theta))
            except Exception:
                break

        if not self.odom_history:
            return

        latest_odom = self.odom_history[-1]
        raw_odom_x = latest_odom[1]
        raw_odom_y = latest_odom[2]
        raw_odom_yaw = latest_odom[3]

        cov_time = float(latest_odom[0])
        if self._cov_odom_pose is not None:
            px, py, pyaw = self._cov_odom_pose
            self.odom_covariance = propagate_odometry_covariance(
                self.odom_covariance,
                raw_odom_x - px,
                raw_odom_y - py,
                normalize_angle(raw_odom_yaw - pyaw),
                pyaw,
                max(0.0, cov_time - (self._cov_odom_time or cov_time)),
            )
        self._cov_odom_pose = (raw_odom_x, raw_odom_y, raw_odom_yaw)
        self._cov_odom_time = cov_time

        if not self.fused_initialized:
            self.fused_x = raw_odom_x
            self.fused_y = raw_odom_y
            self.fused_yaw = raw_odom_yaw
            self.delta_map_odom_x = 0.0
            self.delta_map_odom_y = 0.0
            self.delta_map_odom_yaw = 0.0
            self.fused_initialized = True
            self.map_odom_initialized = True
        else:
            # Преобразование SE(2): T_map_base = T_map_odom @ T_odom_base
            self.fused_x, self.fused_y, self.fused_yaw = compute_fused_pose_se2(
                self.delta_map_odom_x, self.delta_map_odom_y, self.delta_map_odom_yaw,
                raw_odom_x, raw_odom_y, raw_odom_yaw
            )

        if time.time() - self.last_valid_tag_time > 0.6:
            self.tracking_mode = "dead_reckoning"

        now = self.get_clock().now()
        self.publish_fused_pose(now.to_msg())

    def check_motor_power_watchdog(self):
        """Автоматическое снятие тока с обмоток при отсутствии команд более 2.0 секунд"""
        if self.motor_power_state == "enabled":
            # Не снимаем ток, если активно автономное движение по маршруту или калибровочный тест
            if not self.autopilot_active and not self.test_drive_active:
                if time.time() - self.last_motion_cmd_time >= 2.0:
                    self.set_motor_power("disable")
                    self.get_logger().info("💤 Нет команд 2 секунды: ток с обмоток моторов снят автоматически")

    def set_motor_power(self, state: str) -> bool:
        """
        Управление питанием обмоток шаговых двигателей:
        state == 'enable'  -> включить ток (удержание валов, HoldMode.CONTINUOUS_HOLD)
        state == 'disable' -> снять ток с обмоток (HoldMode.DISABLED, валы свободны, 0 Вт, охлаждение)
        """
        try:
            try:
                from .termit_api import HoldMode
            except (ImportError, ValueError):
                from termit_api import HoldMode
        except Exception:
            HoldMode = None

        if state == "disable":
            self.motor_power_state = "disabled"
            if self.robot and self.robot.is_connected and HoldMode:
                self.robot.set_holding_mode(HoldMode.DISABLED)
            self.notify_ui_event()
            self.get_logger().info("💤 Ток с обмоток снят (валы свободны, нагрев 0)")
            return True
        elif state == "enable":
            self.motor_power_state = "enabled"
            self.last_motion_cmd_time = time.time()
            if self.robot and self.robot.is_connected and HoldMode:
                self.robot.set_holding_mode(HoldMode.CONTINUOUS_HOLD)
            self.notify_ui_event()
            self.get_logger().info("⚡ Ток подан на обмотки моторов (удержание активно)")
            return True
        return False

    def drive_robot(self, forward, strafe, w, source_mode=MotionAuthorityMode.MANUAL):
        """
        Прямое аппаратное управление моторами через ESP32 API + публикация Twist в /cmd_vel.
        Проверяет полномочия через MotionAuthorityManager и соблюдает блокировки E-STOP и Stop-and-Reanchor.
        
        Координаты REP-103:
          forward: vx (вперед > 0, назад < 0) в м/с
          strafe:  vy (влево > 0, вправо < 0) в м/с
          w:       omega (против часовой > 0, по часовой < 0) в рад/с
        """
        is_motion = (abs(forward) > 0.001 or abs(strafe) > 0.001 or abs(w) > 0.001)

        # 1. Проверка E-STOP и полномочий
        if hasattr(self, 'motion_mgr') and self.motion_mgr:
            if self.motion_mgr.current_mode == MotionAuthorityMode.ESTOP:
                self.is_nav_locked = True
                if self.robot and self.robot.is_connected:
                    try:
                        self.robot.emergency_stop()
                    except Exception:
                        pass
                msg = Twist()
                self.cmd_vel_pub.publish(msg)
                return False

            if is_motion:
                ok, reason = self.motion_mgr.request_lease(source_mode)
                if not ok:
                    self.get_logger().warn(f"Motion rejected for {source_mode.value}: {reason}")
                    return False

        # 2. Блокировка навигации (Stop-and-Reanchor)
        if self.is_nav_locked and source_mode not in (MotionAuthorityMode.CALIBRATION, MotionAuthorityMode.MANUAL):
            return False

        # Автоматическое включение питания обмоток и обновление таймера простоя при команде движения
        if is_motion:
            self.last_motion_cmd_time = time.time()
            if self.motor_power_state == "disabled":
                self.set_motor_power("enable")

        # 3. Hardware adapter. The proven TermitRobotAPI convention is
        # vx=right, vy=forward; convert once from REP-103 (+y=left).
        if self.robot and self.robot.is_connected:
            try:
                if not is_motion:
                    self.robot.stop()
                else:
                    self.robot.drive(vx=float(-strafe), vy=float(forward), omega=float(w))
            except Exception as e:
                self.get_logger().error(f"ESP32 motor drive error: {str(e)}")

        # 4. Публикация в ROS 2 топик /cmd_vel для совместимости
        msg = Twist()
        msg.linear.x = float(forward)
        msg.linear.y = float(strafe)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)
        if not is_motion and source_mode == MotionAuthorityMode.MANUAL and self.motion_mgr:
            self.motion_mgr.release_lease(MotionAuthorityMode.MANUAL)
        return True

    def set_path_plan(self, waypoints):
        """Сохранение путевых точек маршрута, расчет кумулятивных длин и скоростей в углах"""
        self.route_waypoints = [list(pt) for pt in waypoints]
        self.current_seg_idx = 0
        self.current_wp_idx = 0
        self.route_state = "idle"
        self.path_s_accum = [0.0]
        self.path_corner_speeds = [self.ap_cruise_speed] * len(self.route_waypoints)
        
        for i in range(len(self.route_waypoints) - 1):
            p1 = self.route_waypoints[i]
            p2 = self.route_waypoints[i+1]
            seg_len = float(np.hypot(p2[0] - p1[0], p2[1] - p1[1]))
            self.path_s_accum.append(self.path_s_accum[-1] + seg_len)

        for i in range(1, len(self.route_waypoints) - 1):
            p_prev = np.array(self.route_waypoints[i-1])
            p_curr = np.array(self.route_waypoints[i])
            p_next = np.array(self.route_waypoints[i+1])
            
            v_in = p_curr - p_prev
            v_out = p_next - p_curr
            l_in = float(np.linalg.norm(v_in))
            l_out = float(np.linalg.norm(v_out))
            
            if l_in > 1e-4 and l_out > 1e-4:
                u_in = v_in / l_in
                u_out = v_out / l_out
                cos_turn = float(np.clip(np.dot(u_in, u_out), -1.0, 1.0))
                turn_angle = float(np.arccos(cos_turn))
                if turn_angle > np.radians(15.0):
                    v_c = self.ap_cruise_speed * np.cos(turn_angle / 2.0) * self.ap_turn_factor
                    self.path_corner_speeds[i] = max(self.ap_min_lin, float(v_c))

        self.publish_plan(waypoints)
        self.notify_ui_event()
        self.get_logger().info(f"Загружен маршрут: {len(waypoints)} точек, общая длина {self.path_s_accum[-1]:.2f}м")

    def start_route(self):
        """Запуск автономного движения по маршруту"""
        if not self.route_waypoints or len(self.route_waypoints) < 2:
            self.get_logger().warn("Невозможно запустить маршрут: список точек пуст или содержит менее 2 точек!")
            return False
            
        ok, reason = self.motion_mgr.request_lease(MotionAuthorityMode.ROUTE)
        if not ok:
            self.get_logger().warn(reason)
            return False
        self.set_motor_power("enable")
        self.route_state = "running"
        self.autopilot_active = True
        
        if self.autopilot_thread is None or not self.autopilot_thread.is_alive():
            self.autopilot_thread = threading.Thread(target=self.autopilot_loop, daemon=True, name="Autopilot")
            self.autopilot_thread.start()
            
        self.notify_ui_event()
        self.get_logger().info(f"▶ Старт векторного автопилота: сегмент 1/{len(self.route_waypoints) - 1}")
        return True

    def pause_route(self):
        """Пауза / Снятие с паузы автопилота"""
        if self.route_state == "running":
            self.route_state = "paused"
            self.autopilot_active = False
            self.drive_robot(0.0, 0.0, 0.0, source_mode=MotionAuthorityMode.ROUTE)
            self.motion_mgr.release_lease(MotionAuthorityMode.ROUTE)
            self.notify_ui_event()
            self.get_logger().info(f"⏸ Автопилот на паузе (точка {self.current_wp_idx + 1}/{len(self.route_waypoints)})")
            return True
        elif self.route_state == "paused":
            return self.start_route()
        return False

    def stop_route(self):
        """Полная остановка и сброс маршрута с автоматическим снятием тока"""
        self.route_state = "idle"
        self.autopilot_active = False
        self.current_wp_idx = 0
        self.current_seg_idx = 0
        self.drive_robot(0.0, 0.0, 0.0, source_mode=MotionAuthorityMode.ROUTE)
        self.motion_mgr.release_lease(MotionAuthorityMode.ROUTE)
        self.last_motion_cmd_time = time.time()
        self.notify_ui_event()
        self.get_logger().info("⏹ Маршрут сброшен; плавная остановка, затем авто-снятие тока.")
        return True

    def clear_waypoints(self):
        """Очистка путевых точек"""
        self.stop_route()
        self.route_waypoints = []
        self.publish_plan([])
        self.notify_ui_event()
        self.get_logger().info("🗑 Путевые точки очищены.")
        return True

    def return_to_origin(self):
        """Автоматическое построение гладкого маршрута и возврат робота в начало координат (0,0)"""
        rx = float(self.fused_x)
        ry = float(self.fused_y)
        dist = np.sqrt(rx*rx + ry*ry)
        num_pts = max(3, int(np.ceil(dist / 0.05)))
        points = []
        for i in range(num_pts + 1):
            t = i / float(num_pts)
            points.append([float(rx * (1.0 - t)), float(ry * (1.0 - t))])
        self.set_path_plan(points)
        self.get_logger().info(f"🎯 Построен маршрут возврата в (0,0): {len(points)} точек, дистанция {dist:.2f}м")
        return self.start_route()

    def autopilot_loop(self):
        """
        Высокоточный векторный контроллер следования по траектории (20 Гц).
        - Посегментное продвижение без срезания углов и без застревания на вершинах
        - Плавное кинематическое замедление перед каждым углом и перед финишем
        - Активная компенсация бокового сноса v_cross_cmd
        - Строгая кинематика Omni REP-103
        """
        import time as pytime

        if not self.route_waypoints or len(self.route_waypoints) < 2:
            self.route_state = "idle"
            self.autopilot_active = False
            self.motion_mgr.release_lease(MotionAuthorityMode.ROUTE)
            return

        total_path_len = self.path_s_accum[-1]
        total_wps = len(self.route_waypoints)
        seg_idx = 0
        self.current_seg_idx = 0
        initial_yaw = float(self.fused_yaw)

        smooth_forward = 0.0
        smooth_strafe = 0.0
        smooth_w = 0.0
        alpha = 0.40

        self.get_logger().info(f"▶ Старт векторного контроллера: длина пути {total_path_len:.2f}м, режим курса: {self.ap_yaw_mode}")

        while self.autopilot_active and self.route_state == "running":
            t_loop_start = pytime.time()
            self.last_motion_cmd_time = t_loop_start

            rx = float(self.fused_x)
            ry = float(self.fused_y)
            ryaw = float(self.fused_yaw)

            # 1. Текущий сегмент пути
            p_a = np.array(self.route_waypoints[seg_idx])
            p_b = np.array(self.route_waypoints[seg_idx + 1])
            ab = p_b - p_a
            seg_len = float(np.linalg.norm(ab))
            if seg_len < 1e-6:
                seg_len = 1e-6
            tangent = ab / seg_len
            normal = np.array([-tangent[1], tangent[0]])

            ap = np.array([rx, ry]) - p_a
            t_param = float(np.dot(ap, tangent) / seg_len)
            dist_to_next_wp = float(np.hypot(p_b[0] - rx, p_b[1] - ry))

            # 2. Продвижение на следующий сегмент
            if seg_idx < total_wps - 2:
                if t_param >= 0.95 or dist_to_next_wp <= self.ap_wp_tol:
                    seg_idx += 1
                    self.current_seg_idx = seg_idx
                    self.current_wp_idx = seg_idx
                    self.notify_ui_event()
                    self.get_logger().info(f"📍 Вершина пройдена! Переход на сегмент {seg_idx + 1}/{total_wps - 1}")
                    p_a = np.array(self.route_waypoints[seg_idx])
                    p_b = np.array(self.route_waypoints[seg_idx + 1])
                    ab = p_b - p_a
                    seg_len = float(np.linalg.norm(ab))
                    if seg_len < 1e-6:
                        seg_len = 1e-6
                    tangent = ab / seg_len
                    normal = np.array([-tangent[1], tangent[0]])
                    ap = np.array([rx, ry]) - p_a
                    t_param = float(np.dot(ap, tangent) / seg_len)

            self.current_wp_idx = seg_idx + 1

            # 3. Проекция на текущий сегмент и боковая ошибка e_cross
            t_clamped = float(np.clip(t_param, 0.0, 1.0))
            proj_pt = p_a + t_clamped * ab
            current_s = self.path_s_accum[seg_idx] + t_clamped * seg_len
            e_cross = float(np.dot(np.array([rx, ry]) - proj_pt, normal))

            # 4. Проверка достижения финиша
            dist_to_finish = max(0.0, total_path_len - current_s)
            fx, fy = self.route_waypoints[-1]
            finish_euclid = float(np.hypot(fx - rx, fy - ry))

            if seg_idx >= total_wps - 2:
                if dist_to_finish <= self.ap_goal_tol or finish_euclid <= self.ap_goal_tol:
                    self.drive_robot(0.0, 0.0, 0.0, source_mode=MotionAuthorityMode.ROUTE)
                    pytime.sleep(0.4)
                    self.route_state = "finished"
                    self.autopilot_active = False
                    self.notify_ui_event()
                    self.get_logger().info(f"🎉 Маршрут полностью выполнен! Финиш достигнут (ошибка {finish_euclid*100:.1f} см)")
                    self.last_motion_cmd_time = pytime.time()
                    break

            # 5. Профиль скорости вдоль траектории (along-track velocity)
            if dist_to_finish <= self.ap_goal_tol:
                v_finish = self.ap_min_lin
            else:
                v_finish = np.sqrt(max(0.0, 2.0 * self.ap_brake_accel * max(0.0, dist_to_finish - self.ap_goal_tol))) + self.ap_min_lin

            v_corners = []
            for i in range(seg_idx + 1, total_wps - 1):
                d_to_corner = self.path_s_accum[i] - current_s
                if d_to_corner > 0:
                    v_max_c = self.path_corner_speeds[i]
                    v_brake_c = np.sqrt(v_max_c**2 + 2.0 * self.ap_brake_accel * d_to_corner)
                    v_corners.append(v_brake_c)

            v_along_target = min([self.ap_cruise_speed, v_finish] + v_corners)
            v_along_target = float(np.clip(v_along_target, self.ap_min_lin, self.ap_cruise_speed))

            # 6. Векторное боковое управление (cross-track velocity)
            v_cross_cmd = -float(np.clip(self.ap_kp_cross * e_cross, -self.ap_v_cross_max, self.ap_v_cross_max))

            # 7. Результирующий вектор скорости в СК карты
            v_map = v_along_target * tangent + v_cross_cmd * normal
            v_norm = float(np.linalg.norm(v_map))
            if v_norm > self.ap_cruise_speed:
                v_map = v_map * (self.ap_cruise_speed / v_norm)

            # 8. Проекция скорости карты в REP-103 оси робота:
            # +X вперед, +Y влево.
            v_forward, v_strafe_left = map_velocity_to_body(v_map[0], v_map[1], ryaw)

            # 9. Контроллер ориентации (4 режима yaw)
            if self.ap_yaw_mode == "HOLD_INITIAL":
                target_yaw = initial_yaw
            elif self.ap_yaw_mode == "PATH_TANGENT":
                target_yaw = float(np.arctan2(tangent[1], tangent[0]))
            elif self.ap_yaw_mode == "FINAL_YAW":
                if dist_to_finish < 0.20:
                    target_yaw = self.ap_final_yaw
                else:
                    target_yaw = float(np.arctan2(tangent[1], tangent[0]))
            else:
                target_yaw = ryaw

            yaw_err = float((target_yaw - ryaw + np.pi) % (2.0 * np.pi) - np.pi)
            if abs(yaw_err) < 0.03:
                w = 0.0
            else:
                w = float(np.clip(self.ap_kp_ang * yaw_err, -self.ap_max_ang, self.ap_max_ang))

            # 10. Плавная фильтрация скоростей (EMA)
            smooth_forward = smooth_forward * (1.0 - alpha) + v_forward * alpha
            smooth_strafe = smooth_strafe * (1.0 - alpha) + v_strafe_left * alpha
            smooth_w = smooth_w * (1.0 - alpha) + w * alpha

            # 11. Отправка команды движения
            self.drive_robot(smooth_forward, smooth_strafe, smooth_w, source_mode=MotionAuthorityMode.ROUTE)

            # 12. Логирование телеметрии забега
            rec = {
                "t": t_loop_start,
                "run_id": self.active_run_id,
                "s": round(float(current_s), 4),
                "total_s": round(float(total_path_len), 4),
                "seg_idx": int(seg_idx),
                "wp_idx": int(self.current_wp_idx),
                "rx": round(rx, 4),
                "ry": round(ry, 4),
                "ryaw": round(ryaw, 4),
                "proj_x": round(float(proj_pt[0]), 4),
                "proj_y": round(float(proj_pt[1]), 4),
                "e_cross": round(float(e_cross), 4),
                "dist_finish": round(float(dist_to_finish), 4),
                "v_along": round(float(v_along_target), 4),
                "v_cross": round(float(v_cross_cmd), 4),
                "cmd_fwd": round(float(smooth_forward), 4),
                "cmd_strafe": round(float(smooth_strafe), 4),
                "cmd_w": round(float(smooth_w), 4),
                "mode": self.tracking_mode
            }
            self.log_record(rec)

            elapsed = pytime.time() - t_loop_start
            pytime.sleep(max(0.01, 0.05 - elapsed))

        self.drive_robot(0.0, 0.0, 0.0, source_mode=MotionAuthorityMode.ROUTE)
        self.motion_mgr.release_lease(MotionAuthorityMode.ROUTE)

    def notify_ui_event(self):
        """Отправка немедленного обновления состояния маршрута в SSE"""
        web_data = {
            "type": "route_status",
            "route_state": self.route_state,
            "current_wp": int(self.current_wp_idx),
            "total_wps": int(len(self.route_waypoints)),
            "tracking_mode": self.tracking_mode,
            "esp32_connected": bool(self.robot and self.robot.is_connected),
            "motor_power": self.motor_power_state
        }
        with sse_clients_lock:
            for q in sse_clients:
                try:
                    q.put_nowait(web_data)
                except:
                    pass

    def get_latest_frame(self):
        """Получение последнего кадра с камеры для MJPEG стрима"""
        with self.latest_frame_lock:
            if self.latest_jpeg_frame is not None:
                return self.latest_jpeg_frame
        return self.generate_placeholder_frame()

    def generate_placeholder_frame(self):
        """Генерация заглушки при отсутствии активного видеопотока"""
        if hasattr(self, '_placeholder_jpeg') and self._placeholder_jpeg is not None:
            return self._placeholder_jpeg
        try:
            import cv2
            img = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(img, "TERMiT Camera Feed", (35, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 200), 2)
            cv2.putText(img, "Waiting for /video_feed...", (40, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
            ret, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if ret:
                self._placeholder_jpeg = buf.tobytes()
                return self._placeholder_jpeg
        except:
            pass
        return None

    def destroy_node(self):
        self.autopilot_active = False
        if hasattr(self, 'robot') and self.robot:
            try:
                self.robot.stop()
                self.robot.disconnect()
            except Exception:
                pass
        if hasattr(self, 'server'):
            self.get_logger().info("Stopping web server...")
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception as e:
                self.get_logger().warn(f"Error closing web server: {str(e)}")
        super().destroy_node()

    def set_bridge_parameters(self, wheel_mult, base_mult):
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue

        self.get_logger().info(f"Setting bridge parameters: wheel_mult={wheel_mult}, base_mult={base_mult}")
        
        client = self.create_client(SetParameters, '/esp32_bridge/set_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("esp32_bridge parameter service not available!")
            return False
            
        req = SetParameters.Request()
        
        val_wheel = ParameterValue(type=2, double_value=float(wheel_mult))
        req.parameters.append(Parameter(name='wheel_radius_multiplier', value=val_wheel))
        
        val_base = ParameterValue(type=2, double_value=float(base_mult))
        req.parameters.append(Parameter(name='base_radius_multiplier', value=val_base))
        
        client.call_async(req)
        return True

    def run_test_drive(self, drive_type):
        self.get_logger().info(f"Triggering test drive: {drive_type}")
        
        self.test_drive_active = False
        if self.test_drive_thread and self.test_drive_thread.is_alive():
            self.test_drive_thread.join()
            
        if drive_type == 'stop':
            self.test_drive_active = False
            self.autopilot_active = False
            self.drive_robot(0.0, 0.0, 0.0, source_mode=MotionAuthorityMode.MANUAL)
            if hasattr(self, 'motion_mgr') and self.motion_mgr:
                self.motion_mgr.release_lease(MotionAuthorityMode.TEST)
                self.motion_mgr.release_lease(MotionAuthorityMode.ROUTE)
            self.set_motor_power("disable")
            return True
            
        ok, reason = self.motion_mgr.request_lease(MotionAuthorityMode.TEST)
        if not ok:
            self.get_logger().warn(reason)
            return False
        self.set_motor_power("enable")
        self.test_drive_active = True
        import time as pytime
        self.test_drive_thread = threading.Thread(target=self.test_drive_loop, args=(drive_type,), daemon=True)
        self.test_drive_thread.start()
        return True

    def test_drive_loop(self, drive_type):
        import time as pytime
        start_time = pytime.time()
        
        if drive_type == 'forward':
            duration = 1.0 / 0.15 # 6.67 seconds to drive 1m
            forward = 0.15
            strafe = 0.0
            w = 0.0
        elif drive_type == 'rotate':
            duration = (2.0 * np.pi) / 0.5 # 12.57 seconds to rotate 360 degrees
            forward = 0.0
            strafe = 0.0
            w = 0.5
        else:
            return
            
        self.get_logger().info(f"Starting test motion '{drive_type}' for {duration:.2f} seconds")
        
        while self.test_drive_active and (pytime.time() - start_time < duration):
            self.drive_robot(forward, strafe, w, source_mode=MotionAuthorityMode.TEST)
            pytime.sleep(0.05)
            
        self.drive_robot(0.0, 0.0, 0.0, source_mode=MotionAuthorityMode.TEST)
        self.test_drive_active = False
        if hasattr(self, 'motion_mgr') and self.motion_mgr:
            self.motion_mgr.release_lease(MotionAuthorityMode.TEST)
        self.get_logger().info("Test motion finished, robot stopped.")
        pytime.sleep(0.3)
        self.set_motor_power("disable")



    def publish_fused_pose(self, stamp):
        self.last_pose_publish_time = time.time()
        if not self.fused_initialized:
            return
            
        robot_pose = PoseStamped()
        robot_pose.header.stamp = stamp
        robot_pose.header.frame_id = 'map'
        
        robot_pose.pose.position.x = self.fused_x
        robot_pose.pose.position.y = self.fused_y
        robot_pose.pose.position.z = self.fused_z
        
        q_arr = self.yaw_to_quaternion_as_array(self.fused_yaw)
        robot_pose.pose.orientation.x = q_arr[0]
        robot_pose.pose.orientation.y = q_arr[1]
        robot_pose.pose.orientation.z = q_arr[2]
        robot_pose.pose.orientation.w = q_arr[3]
        
        self.pose_pub.publish(robot_pose)

        t_msg = TransformStamped()
        t_msg.header.stamp = stamp
        t_msg.header.frame_id = 'map'
        t_msg.child_frame_id = 'base_link'
        
        t_msg.transform.translation.x = self.fused_x
        t_msg.transform.translation.y = self.fused_y
        t_msg.transform.translation.z = self.fused_z
        t_msg.transform.rotation.x = q_arr[0]
        t_msg.transform.rotation.y = q_arr[1]
        t_msg.transform.rotation.z = q_arr[2]
        t_msg.transform.rotation.w = q_arr[3]
        
        self.tf_broadcaster.sendTransform(t_msg)

        if len(self.filtered_trajectory_x) > 0:
            dx = self.fused_x - self.filtered_trajectory_x[-1]
            dy = self.fused_y - self.filtered_trajectory_y[-1]
            dz = self.fused_z - self.filtered_trajectory_z[-1]
            self.filtered_path_length += np.sqrt(dx*dx + dy*dy + dz*dz)

        self.filtered_trajectory_x.append(self.fused_x)
        self.filtered_trajectory_y.append(self.fused_y)
        self.filtered_trajectory_z.append(self.fused_z)

        # Отправляем обновление позы на веб-интерфейс (SSE)
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        web_data = {
            "type": "pose",
            "x": float(self.fused_x),
            "y": float(self.fused_y),
            "z": float(self.fused_z),
            "yaw": float(np.degrees(self.fused_yaw)),
            "raw_x": float(self.raw_trajectory_x[-1]) if len(self.raw_trajectory_x) > 0 else float(self.fused_x),
            "raw_y": float(self.raw_trajectory_y[-1]) if len(self.raw_trajectory_y) > 0 else float(self.fused_y),
            "distance_raw": float(self.raw_path_length),
            "distance_filtered": float(self.filtered_path_length),
            "timestamp": float(stamp_sec),
            "detected_tags": self.last_detected_tags,
            "follower_status": self.route_state,
            "route_state": self.route_state,
            "current_wp": int(self.current_wp_idx),
            "total_wps": int(len(self.route_waypoints)),
            "tracking_mode": self.tracking_mode,
            "esp32_connected": bool(self.robot and self.robot.is_connected),
            "motor_power": self.motor_power_state
        }
        with sse_clients_lock:
            for q in sse_clients:
                try:
                    q.put_nowait(web_data)
                except:
                    pass

        if self.active_log_file:
            rec = {
                "t": float(stamp_sec),
                "run_id": self.active_run_id,
                "rx": float(self.fused_x),
                "ry": float(self.fused_y),
                "rz": float(self.fused_z),
                "ryaw": float(self.fused_yaw),
                "mode": self.tracking_mode,
                "dist_filtered": float(self.filtered_path_length),
                "auto_active": bool(self.autopilot_active),
                "wp_idx": int(self.current_wp_idx) if self.autopilot_active else None,
            }
            if hasattr(self, 'last_esp32_odom') and self.last_esp32_odom:
                rec["odom_x"] = float(self.last_esp32_odom.x)
                rec["odom_y"] = float(self.last_esp32_odom.y)
                rec["odom_theta"] = float(self.last_esp32_odom.theta)
                rec["odom_vx"] = float(self.last_esp32_odom.vx)
                rec["odom_vy"] = float(self.last_esp32_odom.vy)
                rec["odom_w"] = float(self.last_esp32_odom.omega)
            self.log_record(rec)

    def publish_plan(self, points):
        self.get_logger().info(f"Publishing new path plan with {len(points)} waypoints")
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for pt in points:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            path_msg.poses.append(pose)
            
        self.plan_pub.publish(path_msg)

    def status_callback(self, msg):
        self.follower_status = msg.data

    def set_follower_parameters(self, look_ahead, max_lin, max_ang, kp_lin, kp_ang, goal_tol, decel_dist, min_lin, yaw_deadzone, wp_tol, turn_decel):
        """Обновление параметров автопилота внутри ноды и в ROS"""
        self.ap_look_ahead = float(look_ahead)
        self.ap_max_lin = float(max_lin)
        self.ap_max_ang = float(max_ang)
        self.ap_kp_lin = float(kp_lin)
        self.ap_kp_ang = float(kp_ang)
        self.ap_goal_tol = float(goal_tol)
        self.ap_decel_dist = float(decel_dist)
        self.ap_min_lin = float(min_lin)
        self.ap_yaw_deadzone = float(yaw_deadzone)
        self.ap_wp_tol = float(wp_tol)
        self.ap_turn_decel = float(turn_decel)
        
        if self.param_client.service_is_ready():
            req = SetParameters.Request()
            req.parameters.append(Parameter(name='look_ahead_distance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(look_ahead))))
            req.parameters.append(Parameter(name='max_linear_velocity', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(max_lin))))
            req.parameters.append(Parameter(name='max_angular_velocity', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(max_ang))))
            req.parameters.append(Parameter(name='kp_linear', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp_lin))))
            req.parameters.append(Parameter(name='kp_angular', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp_ang))))
            req.parameters.append(Parameter(name='goal_tolerance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(goal_tol))))
            req.parameters.append(Parameter(name='decel_dist', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(decel_dist))))
            req.parameters.append(Parameter(name='min_linear_velocity', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(min_lin))))
            req.parameters.append(Parameter(name='yaw_deadzone_dist', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(yaw_deadzone))))
            req.parameters.append(Parameter(name='waypoint_tolerance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(wp_tol))))
            req.parameters.append(Parameter(name='kp_turn_decel', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(turn_decel))))
            self.param_client.call_async(req)

        self.get_logger().info(f"Updated autopilot parameters: max_lin={max_lin}, max_ang={max_ang}, goal_tol={goal_tol}")
        return True

    def quaternion_to_yaw_from_quat(self, q):
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        return np.arctan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion_as_array(self, yaw):
        qw = np.cos(yaw / 2.0)
        qx = 0.0
        qy = 0.0
        qz = np.sin(yaw / 2.0)
        return np.array([qx, qy, qz, qw])

    def save_trajectory_and_shutdown(self):
        if not self.raw_trajectory_x:
            self.get_logger().warn("Trajectory is empty. Cannot generate plot or statistics.")
            return

        # 1. Расчет статистики для сырых и отфильтрованных данных
        raw_path_length = 0.0
        for i in range(len(self.raw_trajectory_x) - 1):
            dx = self.raw_trajectory_x[i+1] - self.raw_trajectory_x[i]
            dy = self.raw_trajectory_y[i+1] - self.raw_trajectory_y[i]
            dz = self.raw_trajectory_z[i+1] - self.raw_trajectory_z[i]
            raw_path_length += np.sqrt(dx*dx + dy*dy + dz*dz)

        filtered_path_length = 0.0
        for i in range(len(self.filtered_trajectory_x) - 1):
            dx = self.filtered_trajectory_x[i+1] - self.filtered_trajectory_x[i]
            dy = self.filtered_trajectory_y[i+1] - self.filtered_trajectory_y[i]
            dz = self.filtered_trajectory_z[i+1] - self.filtered_trajectory_z[i]
            filtered_path_length += np.sqrt(dx*dx + dy*dy + dz*dz)

        t_start = self.trajectory_timestamps[0]
        t_end = self.trajectory_timestamps[-1]
        duration = t_end - t_start
        
        avg_speed_raw = raw_path_length / duration if duration > 0 else 0.0
        avg_speed_filtered = filtered_path_length / duration if duration > 0 else 0.0

        self.get_logger().info("\n" + "="*50 + "\n" +
                               "📊 СТАТИСТИКА ПОЕЗДКИ РОБОТА:\n" +
                               f"🔹 Длительность поездки: {duration:.2f} сек\n" +
                               f"🔹 Количество точек траектории: {len(self.raw_trajectory_x)}\n" +
                               f"🔸 Длина пути (СЫРАЯ с шумом): {raw_path_length:.3f} м (Ср. скорость: {avg_speed_raw:.3f} м/с)\n" +
                               f"🔸 Длина пути (ФИЛЬТРОВАННАЯ): {filtered_path_length:.3f} м (Ср. скорость: {avg_speed_filtered:.3f} м/с)\n" +
                               f"🔹 Коэффициент сглаживания filter_alpha: {self.filter_alpha}\n" +
                               f"🔹 Диапазон X (фильтр): [{min(self.filtered_trajectory_x):.3f}, {max(self.filtered_trajectory_x):.3f}] м\n" +
                               f"🔹 Диапазон Y (фильтр): [{min(self.filtered_trajectory_y):.3f}, {max(self.filtered_trajectory_y):.3f}] м\n" +
                               f"🔹 Диапазон Z (высота камеры): [{min(self.filtered_trajectory_z):.3f}, {max(self.filtered_trajectory_z):.3f}] м\n" +
                               "="*50)

        # 2. Построение графиков с помощью matplotlib
        try:
            import matplotlib
            matplotlib.use('Agg') # Для работы без графического интерфейса
            import matplotlib.pyplot as plt

            plt.figure(figsize=(10, 8))
            
            # Рисуем сырой путь (серая пунктирная линия с точками)
            plt.plot(self.raw_trajectory_x, self.raw_trajectory_y, color='gray', linestyle=':', alpha=0.5, label='Сырой путь (шум ArUco)')
            
            # Рисуем отфильтрованный путь (сплошная синяя линия)
            plt.plot(self.filtered_trajectory_x, self.filtered_trajectory_y, color='royalblue', linewidth=2.5, label='Отфильтрованный путь (EMA/Slerp)')
            
            # Точки старта и финиша (по отфильтрованному пути)
            plt.scatter(self.filtered_trajectory_x[0], self.filtered_trajectory_y[0], color='limegreen', s=150, zorder=5, label='Старт')
            plt.scatter(self.filtered_trajectory_x[-1], self.filtered_trajectory_y[-1], color='crimson', s=150, zorder=5, label='Финиш')

            # Рисуем потолочные метки ArUco
            tags_x = []
            tags_y = []
            tags_labels = []
            for tag_name, tag_info in self.tags_db.items():
                tags_x.append(tag_info['x'])
                tags_y.append(tag_info['y'])
                tags_labels.append(tag_name)
            
            if tags_x:
                plt.scatter(tags_x, tags_y, color='darkorange', marker='s', s=100, zorder=4, label='Потолочные метки (ArUco)')
                for i, txt in enumerate(tags_labels):
                    plt.annotate(txt, (tags_x[i], tags_y[i]), textcoords="offset points", xytext=(0,10), ha='center', fontweight='bold')

            # Сетка и подписи осей
            plt.grid(True, linestyle='--', alpha=0.5)
            plt.xlabel('Координата X (метры)', fontsize=12)
            plt.ylabel('Координата Y (метры)', fontsize=12)
            plt.title('Сравнение сырой и отфильтрованной траектории движения робота', fontsize=14, fontweight='bold')
            plt.axis('equal')
            plt.legend(loc='best', fontsize=10)

            # Текст со статистикой на графике
            stats_text = (
                f"Сырой путь: {raw_path_length:.2f} м\n"
                f"Фильтрованный: {filtered_path_length:.2f} м\n"
                f"Время поездки: {duration:.1f} с\n"
                f"Ср. скорость: {avg_speed_filtered:.2f} м/с"
            )
            plt.gcf().text(0.15, 0.15, stats_text, fontsize=10, bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.5'))

            # Путь для сохранения графика
            share_dir = get_package_share_directory('fake_tag_publisher')
            plot_name = 'trajectory_plot.png'
            share_plot_path = os.path.join(share_dir, 'config', plot_name)
            
            os.makedirs(os.path.dirname(share_plot_path), exist_ok=True)
            plt.savefig(share_plot_path, dpi=150, bbox_inches='tight')
            self.get_logger().info(f"Saved trajectory plot to installed share: {share_plot_path}")
            
            ws_path = os.path.abspath(os.path.join(share_dir, '../../../../'))
            src_plot_dir = os.path.join(ws_path, 'src', 'fake_tag_publisher', 'config')
            if os.path.exists(src_plot_dir):
                src_plot_path = os.path.join(src_plot_dir, plot_name)
                plt.savefig(src_plot_path, dpi=150, bbox_inches='tight')
                self.get_logger().info(f"Saved trajectory plot to source src directory: {src_plot_path}")
            
            plt.close()
        except Exception as e:
            self.get_logger().error(f"Failed to generate trajectory plot: {str(e)}")

# --- ВЕБ-СЕРВЕР ДЛЯ ОТОБРАЖЕНИЯ В РЕАЛЬНОМ ВРЕМЕНИ ---

sse_clients = []
sse_clients_lock = threading.Lock()

class WebServerHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        # Отключаем логирование запросов, чтобы не мусорить в консоли
        pass

    def do_HEAD(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()

    def do_POST(self):
        node = self.server.node
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length) if content_length > 0 else b""
        payload = {}
        if post_data:
            try:
                payload = json.loads(post_data.decode('utf-8'))
            except Exception:
                pass

        from urllib.parse import urlparse, parse_qs
        parsed_url = urlparse(self.path)
        query = parse_qs(parsed_url.query)
        request_path = parsed_url.path

        if self.path.startswith('/api/tags/save'):
            reg = getattr(node, 'tag_registry', None)
            if not reg:
                self._send_json(500, {"error": "Registry not initialized"})
                return

            tag_id = payload.get('tag_id', query.get('id', [None])[0])
            tag_data = payload.get('tag_data', {})
            expected_rev = payload.get('expected_revision', query.get('expected_revision', [None])[0])
            if expected_rev is not None:
                try:
                    expected_rev = int(expected_rev)
                except ValueError:
                    expected_rev = None

            if tag_id is None:
                self._send_json(400, {"error": "Missing tag_id"})
                return

            try:
                rev, sha = reg.set_tag(int(tag_id), tag_data, expected_revision=expected_rev)
                node.tags_db = reg.get_active_confirmed_tags()
                node.publish_tag_map_update()
                node.notify_ui_event()
                self._send_json(200, {"status": "ok", "message": f"Tag {tag_id} saved", "revision": rev, "sha256": sha})
            except Exception as e:
                err_str = str(e)
                code = 409 if "conflict" in err_str.lower() else 400
                self._send_json(code, {"error": err_str})

        elif self.path.startswith('/api/tags/delete'):
            reg = getattr(node, 'tag_registry', None)
            if not reg:
                self._send_json(500, {"error": "Registry not initialized"})
                return

            tag_id = payload.get('tag_id', query.get('id', [None])[0])
            expected_rev = payload.get('expected_revision', query.get('expected_revision', [None])[0])
            allow_anchor_delete = bool(payload.get('allow_anchor_delete', False))
            if expected_rev is not None:
                try:
                    expected_rev = int(expected_rev)
                except ValueError:
                    expected_rev = None

            if tag_id is None:
                self._send_json(400, {"error": "Missing tag_id"})
                return

            try:
                deleting_anchor = str(int(tag_id)) == reg.anchor_tag_id
                if deleting_anchor and allow_anchor_delete:
                    if node.wizard and node.wizard.state not in (WizardState.IDLE, WizardState.COMPLETED, WizardState.ABORTED):
                        node.wizard.abort("Anchor tag deleted")
                    node.stop_route()
                    node.drive_robot(0.0, 0.0, 0.0, source_mode=MotionAuthorityMode.MANUAL)
                    node.set_motor_power("disable")

                rev, sha = reg.delete_tag(
                    int(tag_id), expected_revision=expected_rev,
                    allow_anchor_delete=allow_anchor_delete
                )
                node.tags_db = reg.get_active_confirmed_tags()
                if deleting_anchor:
                    node.map_odom_initialized = False
                    node.is_nav_locked = True
                    node.visual_jump_pending = False
                node.publish_tag_map_update()
                node.notify_ui_event()
                self._send_json(200, {
                    "status": "ok", "message": f"Tag {tag_id} deleted",
                    "anchor_cleared": deleting_anchor,
                    "navigation_locked": deleting_anchor,
                    "revision": rev, "sha256": sha
                })
            except Exception as e:
                err_str = str(e)
                code = 409 if "conflict" in err_str.lower() else 400
                self._send_json(code, {"error": err_str})

        elif request_path == '/api/anchor/set':
            self._send_json(410, {"error": "Use /api/anchor/confirm; anchor confirmation requires live visual evidence"})

        elif self.path.startswith('/api/calibration/start'):
            wiz = getattr(node, 'wizard', None)
            tid = payload.get('tag_id', query.get('id', [None])[0])
            size_m = float(payload.get('marker_size_m', query.get('size', [node.default_marker_size_mm / 1000.0])[0]))

            if tid is None:
                self._send_json(400, {"error": "Missing tag_id"})
                return

            target_id = int(tid)
            visible = next((d for d in node.latest_detections if int(d.get('tag_id', -1)) == target_id and d.get('pose_valid', False)), None)
            blockers = []
            if not getattr(node, 'camera_calibration_valid', False): blockers.append("camera intrinsics are unavailable")
            if node.camera_extrinsics_status != 'verified': blockers.append("camera extrinsics are unverified")
            if not node.tag_registry.anchor_confirmed: blockers.append("anchor tag is not confirmed")
            if not (node.robot and node.robot.is_connected): blockers.append("ESP32 is disconnected")
            if not node.map_odom_initialized: blockers.append("robot pose has not been initialized from a known tag")
            if time.time() - node.last_esp32_odom_time > 0.5: blockers.append("wheel odometry is stale")
            ack = node.last_tag_map_ack or {}
            if not (ack.get('status') == 'ok'
                    and ack.get('config_epoch') == node.tag_registry.config_epoch
                    and ack.get('detector_revision') == node.tag_registry.revision
                    and ack.get('detector_sha256') == node.tag_registry.sha256):
                blockers.append("tag map is not synchronized with detector")
            if time.monotonic() - node.latest_detections_stamp > 0.25 or visible is None: blockers.append("target tag is not visible with a valid pose")
            if node.is_nav_locked: blockers.append("navigation is locked")
            if blockers:
                self._send_json(409, {"error": "; ".join(blockers), "blocking_reasons": blockers})
                return
            ok, msg = wiz.start(target_id, marker_size_m=size_m)
            if not ok:
                self._send_json(409, {"error": msg})
                return
            self._send_json(200, {"status": "ok", "message": msg})

        elif self.path.startswith('/api/calibration/heartbeat'):
            wiz = getattr(node, 'wizard', None)
            ok, msg = wiz.heartbeat() if wiz else (False, "Wizard not initialized")
            if not ok:
                self._send_json(400, {"error": msg})
                return
            self._send_json(200, {"status": "ok", "message": msg})

        elif self.path.startswith('/api/calibration/abort'):
            wiz = getattr(node, 'wizard', None)
            if wiz:
                wiz.abort("User requested abort via API")
            node.drive_robot(0.0, 0.0, 0.0, source_mode=MotionAuthorityMode.CALIBRATION)
            if hasattr(node, 'motion_mgr') and node.motion_mgr:
                node.motion_mgr.release_lease(MotionAuthorityMode.CALIBRATION)
            self._send_json(200, {"status": "ok", "message": "Calibration aborted"})

        elif self.path.startswith('/api/calibration/confirm'):
            wiz = getattr(node, 'wizard', None)
            if not wiz:
                self._send_json(500, {"error": "Wizard not initialized"})
                return
            ok, msg = wiz.confirm_review()
            if not ok:
                self._send_json(400, {"error": msg})
                return
            if wiz.calibrated_tag_result:
                res_tag = dict(wiz.calibrated_tag_result)
                res_tag["state"] = "confirmed"
                res_tag["source"] = "anchor_wizard"
                wiz.calibrated_tag_result = res_tag
                reg = getattr(node, 'tag_registry', None)
                if reg:
                    rev, sha = reg.set_tag(res_tag["tag_id"], res_tag)
                    node.tags_db = reg.get_active_confirmed_tags()
                    node.publish_tag_map_update()
                    node.notify_ui_event()
            self._send_json(200, {"status": "ok", "message": msg, "tag": wiz.calibrated_tag_result})

        elif self.path.startswith('/api/anchor/confirm'):
            reg = getattr(node, 'tag_registry', None)
            if not reg:
                self._send_json(500, {"error": "Registry not initialized"})
                return
            tag_id = payload.get('anchor_tag_id', payload.get('tag_id', query.get('id', [None])[0]))
            size_mm = float(payload.get('size_mm', query.get('size_mm', [node.default_marker_size_mm])[0]))
            ceiling_z_m = float(payload.get('ceiling_z_m', query.get('ceiling_z_m', [node.ceiling_height_m])[0]))
            expected_rev = payload.get('expected_revision', query.get('expected_revision', [None])[0])
            if expected_rev is not None:
                try:
                    expected_rev = int(expected_rev)
                except ValueError:
                    expected_rev = None

            if tag_id is None:
                self._send_json(400, {"error": "Missing tag_id"})
                return

            target_id = int(tag_id)
            fresh = time.monotonic() - node.latest_detections_stamp <= 0.25
            visible = next((d for d in node.latest_detections if int(d.get('tag_id', -1)) == target_id and d.get('pose_valid', False)), None)
            blockers = []
            if not getattr(node, 'camera_calibration_valid', False): blockers.append("camera intrinsics are unavailable")
            if node.camera_extrinsics_status != 'verified': blockers.append("camera extrinsics are unverified")
            if not fresh or visible is None: blockers.append("anchor tag is not visible with a valid pose")
            if not 20.0 <= size_mm <= 1000.0: blockers.append("marker size must be 20..1000 mm")
            if blockers:
                self._send_json(409, {"error": "; ".join(blockers), "blocking_reasons": blockers})
                return

            try:
                rev, sha = reg.set_anchor_tag(int(tag_id), confirm=True, size_mm=size_mm, ceiling_z_m=ceiling_z_m, expected_revision=expected_rev)
                node.tags_db = reg.get_active_confirmed_tags()
                node.publish_tag_map_update()
                node.notify_ui_event()
                self._send_json(200, {"status": "ok", "message": f"Anchor tag {tag_id} confirmed", "revision": rev, "sha256": sha})
            except Exception as e:
                err_str = str(e)
                code = 409 if "conflict" in err_str.lower() else 400
                self._send_json(code, {"error": err_str})

        elif self.path.startswith('/api/settings'):
            new_settings = payload.get('settings', payload)
            try:
                settings = node.update_runtime_settings(new_settings)
                self._send_json(200, {"status": "ok", "settings": settings})
            except (TypeError, ValueError) as e:
                self._send_json(400, {"error": str(e)})
            except Exception as e:
                self._send_json(500, {"error": str(e)})

        elif self.path.startswith('/api/extrinsics'):
            # Save extrinsics to file
            try:
                status = payload.get('status')
                if status not in ('verified', 'unverified'):
                    raise ValueError("status must be explicitly set to verified or unverified")
                data = {
                    "status": status,
                    "x": float(payload.get('x', 0.0)),
                    "y": float(payload.get('y', 0.0)),
                    "z": float(payload.get('z', 0.0)),
                    "roll": float(payload.get('roll', 0.0)),
                    "pitch": float(payload.get('pitch', -np.pi/2.0)),
                    "yaw": float(payload.get('yaw', 0.0))
                }
                if not all(math.isfinite(v) for k, v in data.items() if k != 'status'):
                    raise ValueError("extrinsics values must be finite")
                matrix = pose_to_matrix(data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw'])
                target = node.camera_extrinsics_path
                tmp = target + '.tmp'
                with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
                    yaml.safe_dump(data, f, sort_keys=True)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp, target)
                node.T_base_cam = matrix
                node.camera_extrinsics_status = status
                node.map_odom_initialized = False
                self._send_json(200, {"status": "ok", "message": "Extrinsics saved"})
            except (TypeError, ValueError) as e:
                self._send_json(400, {"error": str(e)})
            except Exception as e:
                self._send_json(500, {"error": str(e)})
        else:
            self._send_json(404, {"error": "Not Found"})

    def _send_json(self, code: int, data: dict):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.end_headers()
            self.wfile.write(HTML_TEMPLATE.encode('utf-8'))
        elif self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    frame = self.server.node.get_latest_frame()
                    if frame is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.04)
                    else:
                        time.sleep(0.08)
            except (ConnectionResetError, BrokenPipeError):
                pass
        elif self.path == '/config':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            # Отправляем конфигурацию потолочных меток
            tags_data = self.server.node.tags_db
            self.wfile.write(json.dumps(tags_data).encode('utf-8'))
        elif self.path == '/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            import queue
            q = queue.Queue()
            with sse_clients_lock:
                sse_clients.append(q)
            
            try:
                # Отправляем событие успешного подключения
                self.wfile.write(b"data: {\"type\": \"connected\"}\n\n")
                self.wfile.flush()
                
                while True:
                    try:
                        data = q.get(timeout=2.0)
                        event_str = f"data: {json.dumps(data)}\n\n"
                        self.wfile.write(event_str.encode('utf-8'))
                        self.wfile.flush()
                    except queue.Empty:
                        # Отправка пинга для поддержания активности
                        self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
            except (ConnectionResetError, BrokenPipeError):
                pass
            finally:
                with sse_clients_lock:
                    if q in sse_clients:
                        sse_clients.remove(q)
        elif self.path.startswith('/set_calib'):
            from urllib.parse import urlparse, parse_qs
            parsed_url = urlparse(self.path)
            query = parse_qs(parsed_url.query)
            
            wheel_mult = float(query.get('wheel_mult', [1.0])[0])
            base_mult = float(query.get('base_mult', [1.0])[0])
            
            self.server.node.set_bridge_parameters(wheel_mult, base_mult)
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success"}).encode('utf-8'))
            
        elif self.path.startswith('/test_drive'):
            from urllib.parse import urlparse, parse_qs
            parsed_url = urlparse(self.path)
            query = parse_qs(parsed_url.query)
            
            drive_type = query.get('type', ['stop'])[0]
            self.server.node.run_test_drive(drive_type)
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success", "type": drive_type}).encode('utf-8'))
            
        elif self.path.startswith('/set_path'):
            from urllib.parse import urlparse, parse_qs
            parsed_url = urlparse(self.path)
            query = parse_qs(parsed_url.query)
            
            points_str = query.get('points', [''])[0]
            points = []
            if points_str:
                for pt_str in points_str.split(';'):
                    if ',' in pt_str:
                        coords = pt_str.split(',')
                        try:
                            points.append([float(coords[0]), float(coords[1])])
                        except ValueError:
                            pass
            
            self.server.node.set_path_plan(points)
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success", "count": len(points)}).encode('utf-8'))

        elif self.path.startswith('/start_route'):
            success = self.server.node.start_route()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success" if success else "error", "route_state": self.server.node.route_state}).encode('utf-8'))

        elif self.path.startswith('/pause_route'):
            success = self.server.node.pause_route()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success" if success else "error", "route_state": self.server.node.route_state}).encode('utf-8'))

        elif self.path.startswith('/stop_route'):
            self.server.node.stop_route()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success", "route_state": "idle"}).encode('utf-8'))

        elif self.path.startswith('/clear_waypoints'):
            self.server.node.clear_waypoints()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success"}).encode('utf-8'))

        elif self.path.startswith('/return_home'):
            success = self.server.node.return_to_origin()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success" if success else "error"}).encode('utf-8'))
            
        elif self.path.startswith('/set_follower_params'):
            from urllib.parse import urlparse, parse_qs
            parsed_url = urlparse(self.path)
            query = parse_qs(parsed_url.query)
            
            look_ahead = float(query.get('look_ahead', [0.15])[0])
            max_lin = float(query.get('max_lin', [0.14])[0])
            max_ang = float(query.get('max_ang', [0.70])[0])
            kp_lin = float(query.get('kp_lin', [0.80])[0])
            kp_ang = float(query.get('kp_ang', [1.50])[0])
            goal_tol = float(query.get('goal_tol', [0.03])[0])
            decel_dist = float(query.get('decel_dist', [0.30])[0])
            min_lin = float(query.get('min_lin', [0.03])[0])
            yaw_deadzone = float(query.get('yaw_deadzone', [0.05])[0])
            wp_tol = float(query.get('wp_tol', [0.06])[0])
            turn_decel = float(query.get('turn_decel', [0.35])[0])
            
            self.server.node.set_follower_parameters(
                look_ahead, max_lin, max_ang, kp_lin, kp_ang, goal_tol, decel_dist, min_lin, yaw_deadzone, wp_tol, turn_decel
            )
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success"}).encode('utf-8'))
            
        elif self.path.startswith('/start_work'):
            self.server.node.start_route()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success"}).encode('utf-8'))
            
        elif self.path.startswith('/drive'):
            from urllib.parse import urlparse, parse_qs
            parsed_url = urlparse(self.path)
            query = parse_qs(parsed_url.query)
            
            vx = float(query.get('vx', [0.0])[0])
            vy = float(query.get('vy', [0.0])[0])
            w = float(query.get('w', [0.0])[0])
            
            self.server.node.drive_robot(vx, vy, w, source_mode=MotionAuthorityMode.MANUAL)
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success", "vx": vx, "vy": vy, "w": w}).encode('utf-8'))

        elif self.path.startswith('/set_motor_power'):
            from urllib.parse import urlparse, parse_qs
            parsed_url = urlparse(self.path)
            query = parse_qs(parsed_url.query)
            state = query.get('state', ['enable'])[0]
            success = self.server.node.set_motor_power(state)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "success" if success else "error",
                "motor_power": self.server.node.motor_power_state
            }).encode('utf-8'))

        elif self.path.startswith('/esp32_status'):
            connected = bool(self.server.node.robot and self.server.node.robot.is_connected)
            port_str = self.server.node.robot._port_name if self.server.node.robot else "none"
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"connected": connected, "port": port_str}).encode('utf-8'))

        elif self.path.startswith('/api/settings'):
            node = self.server.node
            settings_data = node.get_runtime_settings() if hasattr(node, 'get_runtime_settings') else {}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "settings": settings_data}).encode('utf-8'))

        elif self.path.startswith('/api/anchor/wizard_status'):
            node = self.server.node
            reg = getattr(node, 'tag_registry', None)
            fresh = time.monotonic() - getattr(node, 'latest_detections_stamp', 0.0) <= 0.25
            cands = []
            for d in ((getattr(node, 'latest_detections', []) or []) if fresh else []):
                cands.append({
                    "tag_id": d.get("tag_id"),
                    "distance_m": d.get("distance_m", 0.0),
                    "reproj_err": d.get("reproj_err", 0.0),
                    "viewing_angle_deg": d.get("viewing_angle_deg", 0.0)
                    ,"pose_valid": bool(d.get("pose_valid", False))
                })
            ext_status = getattr(node, 'camera_extrinsics_status', 'unverified')
            anchor_conf = reg.anchor_confirmed if reg else False
            aid = reg.anchor_tag_id if reg else None
            resp = {
                "status": "ok",
                "anchor_tag_id": aid,
                "anchor_confirmed": anchor_conf,
                "camera_extrinsics_status": ext_status,
                "visible_candidates": cands,
                "ready_for_confirm": (
                    any(c["pose_valid"] for c in cands)
                    and ext_status == "verified"
                    and getattr(node, 'camera_calibration_valid', False)
                )
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode('utf-8'))

        elif self.path.startswith('/api/health'):
            now_t = time.time()
            node = self.server.node
            connected = bool(node.robot and node.robot.is_connected)
            esp_odom_fresh = (now_t - node.last_esp32_odom_time < 0.5) if node.last_esp32_odom_time > 0 else False
            pose_fresh = (now_t - node.last_pose_publish_time < 0.5) if node.last_pose_publish_time > 0 else False
            tag_fresh = (now_t - node.last_valid_tag_time < 1.0) if node.last_valid_tag_time > 0 else False

            reg = getattr(node, 'tag_registry', None)
            anchor_confirmed = reg.anchor_confirmed if reg else False
            camera_ext_status = getattr(node, 'camera_extrinsics_status', 'unverified')
            is_nav_locked = getattr(node, 'is_nav_locked', False)
            estop_active = (node.motion_mgr.current_mode == MotionAuthorityMode.ESTOP) if hasattr(node, 'motion_mgr') else False

            blocking_reasons = []
            if estop_active:
                blocking_reasons.append("E-STOP is active")
            if is_nav_locked:
                blocking_reasons.append("Navigation locked: visual jump pending or safety lock")
            if not anchor_confirmed:
                blocking_reasons.append("Anchor tag unconfirmed")
            if camera_ext_status != "verified":
                blocking_reasons.append("Camera extrinsics unverified")
            if not getattr(node, 'camera_calibration_valid', False):
                blocking_reasons.append("Camera intrinsics unavailable")
            if not connected:
                blocking_reasons.append("ESP32 controller not connected")

            if estop_active or is_nav_locked or not anchor_confirmed:
                overall_status = "blocked"
            elif (not connected or not esp_odom_fresh or camera_ext_status != "verified"
                  or not getattr(node, 'camera_calibration_valid', False)):
                overall_status = "degraded"
            else:
                overall_status = "ok"

            last_ack = getattr(node, 'last_tag_map_ack', None) or {}
            detector_rev = last_ack.get("detector_revision", getattr(node, 'detector_revision', reg.revision if reg else 0))
            hot_reload_synced = bool(reg is None or (
                last_ack.get("status") == "ok"
                and last_ack.get("config_epoch") == reg.config_epoch
                and detector_rev == reg.revision
                and last_ack.get("detector_sha256") == reg.sha256
            ))
            if not hot_reload_synced:
                blocking_reasons.append("Tag-map update has not been acknowledged by detector")
                if overall_status == "ok": overall_status = "degraded"

            wheels_stopped = True
            if node.robot and getattr(node.robot, 'is_connected', False):
                sp1 = abs(getattr(node.robot, 'current_speed_m1', 0))
                sp2 = abs(getattr(node.robot, 'current_speed_m2', 0))
                sp3 = abs(getattr(node.robot, 'current_speed_m3', 0))
                if sp1 > 15 or sp2 > 15 or sp3 > 15:
                    wheels_stopped = False
            calib_ready = (
                getattr(node, 'camera_calibration_valid', False)
                and camera_ext_status == "verified"
                and anchor_confirmed
                and connected
                and esp_odom_fresh
                and hot_reload_synced
                and not estop_active
                and not is_nav_locked
                and wheels_stopped
            )

            health_data = {
                "status": overall_status,
                "blocking_reasons": blocking_reasons,
                "calibration_ready": calib_ready,
                "git_commit": getattr(node, "git_commit", "b4ee324"),
                "firmware_version": getattr(node, "firmware_version", "FastAccelStepper-v2.0"),
                "detector_revision": detector_rev,
                "hot_reload_synced": hot_reload_synced,
                "esp32_connected": connected,
                "esp32_port": node.robot._port_name if node.robot else None,
                "esp32_odom_fresh": esp_odom_fresh,
                "esp32_odom_age_s": round(now_t - node.last_esp32_odom_time, 3) if node.last_esp32_odom_time > 0 else None,
                "pose_fresh": pose_fresh,
                "pose_age_s": round(now_t - node.last_pose_publish_time, 3) if node.last_pose_publish_time > 0 else None,
                "tag_fresh": tag_fresh,
                "tag_age_s": round(now_t - node.last_valid_tag_time, 3) if node.last_valid_tag_time > 0 else None,
                "active_run_id": node.active_run_id,
                "motor_power": node.motor_power_state,
                "tracking_mode": node.tracking_mode,
                "active_tags_count": len(node.tags_db),
                "config_epoch": reg.config_epoch if reg else 0,
                "tag_map_revision": reg.revision if reg else 0,
                "tag_map_sha256": reg.sha256 if reg else "",
                "anchor_tag_id": int(reg.anchor_tag_id) if (reg and reg.anchor_tag_id is not None) else None,
                "anchor_confirmed": anchor_confirmed,
                "camera_extrinsics": {
                    "status": camera_ext_status,
                    "path": getattr(node, 'camera_extrinsics_path', '')
                },
                "camera_intrinsics": {
                    "valid": getattr(node, 'camera_calibration_valid', False),
                    "path": getattr(node, 'camera_calibration_path', '')
                },
                "is_nav_locked": is_nav_locked,
                "visual_jump_pending": getattr(node, 'visual_jump_pending', False)
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(health_data).encode('utf-8'))

        elif self.path.startswith('/api/tags'):
            node = self.server.node
            reg = getattr(node, 'tag_registry', None)
            if not reg:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(json.dumps({"error": "Tag registry not initialized"}).encode('utf-8'))
                return

            resp = {
                "status": "ok",
                "config_epoch": reg.config_epoch,
                "tag_map_revision": reg.revision,
                "tag_map_sha256": reg.sha256,
                "anchor_tag_id": reg.anchor_tag_id,
                "anchor_confirmed": reg.anchor_confirmed,
                "tags": reg.get_all_tags()
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode('utf-8'))

        elif self.path.startswith('/api/calibration/status'):
            node = self.server.node
            wiz = getattr(node, 'wizard', None)
            if not wiz:
                resp = {"state": "IDLE"}
            else:
                resp = {
                    "state": wiz.state.value,
                    "target_tag_id": wiz.target_tag_id,
                    "elapsed_s": round(max(0.0, time.monotonic() - wiz.state_enter_time), 2),
                    "samples_count": len(wiz.collected_samples),
                    "abort_reason": wiz.abort_reason,
                    "centering_axis": wiz.centering_axis,
                    "centering_error_px": wiz.centering_error_px,
                    "centering_axis_error_px": wiz.centering_axis_error_px,
                    "centering_command": list(wiz.centering_command),
                    "calibrated_result": wiz.calibrated_tag_result
                }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode('utf-8'))

        elif self.path.startswith('/api/covisibility'):
            node = self.server.node
            covis = getattr(node, 'covis_graph', None)
            resp = covis.get_diagnostics() if covis else {}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode('utf-8'))

        elif self.path.startswith('/api/extrinsics'):
            node = self.server.node
            resp = {
                "status": getattr(node, 'camera_extrinsics_status', 'unverified'),
                "matrix": node.T_base_cam.tolist()
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode('utf-8'))

        elif self.path.startswith('/api/clear_history'):
            self.server.node.clear_trajectory_history()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))

        elif self.path.startswith('/api/start_log'):
            from urllib.parse import urlparse, parse_qs
            parsed_url = urlparse(self.path)
            query = parse_qs(parsed_url.query)
            run_id = query.get('run_id', [None])[0]
            actual_run_id = self.server.node.start_run_logging(run_id)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "run_id": actual_run_id, "path": self.server.node.active_log_path}).encode('utf-8'))

        elif self.path.startswith('/api/stop_log'):
            log_path = self.server.node.stop_run_logging()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "saved_path": log_path}).encode('utf-8'))
        else:
            self.send_error(404, "File not found")

class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    def __init__(self, server_address, RequestHandlerClass, node):
        super().__init__(server_address, RequestHandlerClass)
        self.node = node

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Локализация Робота в реальном времени</title>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }
        body {
            font-family: 'Outfit', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background-color: #0b0c10;
            color: #c5c6c7;
            display: flex;
            height: 100vh;
            overflow: hidden;
        }
        #sidebar {
            width: 360px;
            background: rgba(20, 24, 33, 0.85);
            backdrop-filter: blur(10px);
            border-right: 1px solid rgba(255, 255, 255, 0.08);
            display: flex;
            flex-direction: column;
            padding: 24px;
            z-index: 10;
            box-shadow: 4px 0 24px rgba(0, 0, 0, 0.5);
            flex-shrink: 0;
            overflow-y: auto;
            max-height: 100vh;
        }
        #sidebar::-webkit-scrollbar {
            width: 6px;
        }
        #sidebar::-webkit-scrollbar-track {
            background: rgba(0, 0, 0, 0.1);
        }
        #sidebar::-webkit-scrollbar-thumb {
            background: rgba(255, 255, 255, 0.15);
            border-radius: 3px;
        }
        #sidebar::-webkit-scrollbar-thumb:hover {
            background: rgba(102, 252, 241, 0.4);
        }
        #map-container {
            flex: 1;
            position: relative;
            background-color: #0f1015;
            display: flex;
            align-items: center;
            justify-content: center;
            overflow: hidden;
        }
        @media (max-width: 768px) {
            body {
                flex-direction: column;
                height: 100vh;
                overflow: hidden;
            }
            #map-container {
                height: 40vh;
                width: 100%;
                flex-shrink: 0;
            }
            #sidebar {
                width: 100%;
                height: 60vh;
                border-right: none;
                border-top: 1px solid rgba(255, 255, 255, 0.08);
                padding: 16px;
                overflow-y: auto;
                max-height: none;
            }
        }
        canvas {
            display: block;
            cursor: grab;
            width: 100%;
            height: 100%;
        }
        canvas:active {
            cursor: grabbing;
        }
        h1 {
            font-size: 22px;
            font-weight: 700;
            color: #fff;
            margin-bottom: 4px;
            background: linear-gradient(45deg, #66fcf1, #45a29e);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .subtitle {
            font-size: 11px;
            color: #8b9bb4;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 16px;
        }
        .status-container {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 13px;
            margin-bottom: 24px;
            color: #8b9bb4;
            background: rgba(255, 255, 255, 0.03);
            padding: 8px 12px;
            border-radius: 8px;
            border: 1px solid rgba(255, 255, 255, 0.05);
            align-self: flex-start;
        }
        .status-dot {
            width: 8px;
            height: 8px;
            background-color: #ff4d4d;
            border-radius: 50%;
            display: inline-block;
            box-shadow: 0 0 8px #ff4d4d;
        }
        .status-dot.connected {
            background-color: #2ecc71;
            box-shadow: 0 0 10px #2ecc71;
            animation: pulse 2s infinite;
        }
        @keyframes pulse {
            0% { box-shadow: 0 0 0 0 rgba(46, 204, 113, 0.7); }
            70% { box-shadow: 0 0 0 10px rgba(46, 204, 113, 0); }
            100% { box-shadow: 0 0 0 0 rgba(46, 204, 113, 0); }
        }
        .section-title {
            font-size: 11px;
            text-transform: uppercase;
            letter-spacing: 1.5px;
            color: #66fcf1;
            margin-bottom: 12px;
            font-weight: 700;
        }
        .card {
            background: rgba(255, 255, 255, 0.02);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 20px;
        }
        .coords-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
        }
        .coord-box {
            display: flex;
            flex-direction: column;
            background: rgba(255, 255, 255, 0.02);
            padding: 8px 12px;
            border-radius: 8px;
            border: 1px solid rgba(255, 255, 255, 0.03);
        }
        .coord-label {
            font-size: 11px;
            color: #8b9bb4;
            margin-bottom: 4px;
            font-weight: 500;
        }
        .coord-val {
            font-size: 18px;
            font-weight: 700;
            color: #fff;
            font-family: 'Outfit', monospace;
        }
        .coord-val span {
            font-size: 12px;
            color: #8b9bb4;
            font-weight: 400;
            margin-left: 2px;
        }
        .stat-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            font-size: 13px;
            padding: 8px 0;
            border-bottom: 1px solid rgba(255, 255, 255, 0.03);
        }
        .stat-item:last-child {
            border-bottom: none;
        }
        .stat-label {
            color: #8b9bb4;
        }
        .stat-val {
            color: #fff;
            font-weight: 600;
        }
        .btn {
            background: #1f2833;
            border: 1px solid #45a29e;
            color: #66fcf1;
            padding: 12px;
            border-radius: 8px;
            cursor: pointer;
            font-weight: 600;
            font-family: inherit;
            transition: all 0.3s;
            width: 100%;
            text-align: center;
            margin-bottom: 10px;
            font-size: 13px;
        }
        .btn:hover {
            background: #66fcf1;
            color: #0b0c10;
            box-shadow: 0 0 15px rgba(102, 252, 241, 0.4);
        }
        .btn-secondary {
            background: transparent;
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: #8b9bb4;
        }
        .btn-secondary:hover {
            background: rgba(255, 255, 255, 0.05);
            color: #fff;
            box-shadow: none;
        }
        .active-tags-grid {
            display: flex;
            flex-wrap: wrap;
            gap: 6px;
            margin-top: 8px;
        }
        .tag-badge {
            background: rgba(255, 165, 0, 0.1);
            border: 1px solid rgba(255, 165, 0, 0.3);
            color: #ffa500;
            padding: 2px 8px;
            border-radius: 12px;
            font-size: 11px;
            font-weight: 600;
        }
        .video-panel {
            position: absolute;
            top: 20px;
            right: 20px;
            display: flex;
            flex-direction: column;
            align-items: flex-end;
            gap: 8px;
            z-index: 10;
        }
        .video-box {
            width: 320px;
            background: rgba(20, 24, 33, 0.85);
            border: 1px solid rgba(255, 255, 255, 0.12);
            border-radius: 10px;
            overflow: hidden;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.6);
            backdrop-filter: blur(12px);
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }
        .video-box.minimized .video-content {
            display: none;
        }
        .video-box-header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 7px 12px;
            background: rgba(0, 0, 0, 0.35);
            border-bottom: 1px solid rgba(255, 255, 255, 0.06);
            user-select: none;
        }
        .video-title {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 11px;
            font-weight: 700;
            letter-spacing: 0.8px;
            color: #66fcf1;
            text-transform: uppercase;
        }
        .video-dot {
            width: 7px;
            height: 7px;
            border-radius: 50%;
            background: #2ea043;
            box-shadow: 0 0 8px #2ea043;
        }
        .video-toggle-btn {
            background: transparent;
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: #8b9bb4;
            border-radius: 4px;
            cursor: pointer;
            font-size: 11px;
            padding: 2px 6px;
            line-height: 1;
            transition: all 0.2s;
        }
        .video-toggle-btn:hover {
            color: #fff;
            border-color: #66fcf1;
        }
        .video-content {
            position: relative;
            background: #000;
            width: 100%;
            line-height: 0;
        }
        #camera-stream {
            width: 100%;
            height: auto;
            max-height: 240px;
            object-fit: contain;
            display: block;
        }
        .control-panel {
            display: flex;
            gap: 8px;
            justify-content: flex-end;
            width: 100%;
        }
        .icon-btn {
            background: rgba(20, 24, 33, 0.85);
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: #fff;
            padding: 8px 16px;
            border-radius: 8px;
            cursor: pointer;
            backdrop-filter: blur(10px);
            transition: all 0.2s;
            font-family: inherit;
            font-size: 12px;
            font-weight: 500;
        }
        .icon-btn:hover {
            border-color: #66fcf1;
            color: #66fcf1;
        }
        .icon-btn.active {
            background: #66fcf1;
            color: #0b0c10;
            border-color: #66fcf1;
        }
        .calib-container {
            background: rgba(255, 255, 255, 0.02);
            border: 1px solid rgba(255, 255, 255, 0.05);
            padding: 12px;
            border-radius: 8px;
            margin-bottom: 16px;
        }
        .calib-row {
            display: flex;
            align-items: center;
            justify-content: space-between;
            margin-bottom: 8px;
        }
        .calib-input {
            width: 80px;
            background: rgba(0, 0, 0, 0.3);
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: #fff;
            padding: 4px 8px;
            border-radius: 4px;
            font-family: inherit;
            text-align: right;
        }
        .calib-input:focus {
            outline: none;
            border-color: #66fcf1;
        }
        .tag-form {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 7px;
        }
        .tag-form input, .tag-form button { min-width: 0; width: 100%; }
        .tag-form .tag-id { grid-column: span 1; }
        .tag-form .tag-save { grid-column: 1 / -1; }
        .wizard-controls { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
        .wizard-controls button { min-width: 0; width: 100%; }
        .control-box {
            background: rgba(255, 255, 255, 0.02);
            border: 1px solid rgba(255, 255, 255, 0.05);
            padding: 16px;
            border-radius: 8px;
            margin-bottom: 16px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .joystick-zone {
            width: 160px;
            height: 160px;
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
        }
        #joystick-base {
            width: 120px;
            height: 120px;
            background: rgba(255, 255, 255, 0.03);
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 50%;
            position: relative;
            touch-action: none;
        }
        #joystick-handle {
            width: 40px;
            height: 40px;
            background: #66fcf1;
            border-radius: 50%;
            position: absolute;
            top: 38px;
            left: 38px;
            box-shadow: 0 0 15px rgba(102, 252, 241, 0.6);
            cursor: pointer;
            transition: transform 0.05s ease;
        }
        #terminal-log {
            margin-top: auto;
            background: rgba(0, 0, 0, 0.3);
            border: 1px solid rgba(255, 255, 255, 0.03);
            border-radius: 8px;
            padding: 12px;
            font-family: monospace;
            font-size: 11px;
            height: 120px;
            overflow-y: auto;
            color: #8b9bb4;
        }
        .log-entry {
            margin-bottom: 4px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.01);
            padding-bottom: 2px;
        }
        .log-time {
            color: #45a29e;
            margin-right: 6px;
        }
    </style>
</head>
<body>
    <div id="sidebar">
        <h1>ЛОКАЛИЗАЦИЯ</h1>
        <div class="subtitle">Multi-tag Data Fusion</div>
        
        <div class="status-container">
            <span class="status-dot" id="status-dot"></span>
            <span id="status-text">ПОДКЛЮЧЕНИЕ...</span>
        </div>

        <div class="sync-badge-container" id="sync-container" style="background: rgba(0,0,0,0.4); border: 1px solid rgba(255,255,255,0.08); border-radius: 6px; padding: 8px 10px; margin-bottom: 12px; font-size: 11px;">
            <div style="display:flex; justify-content:space-between; margin-bottom: 4px;">
                <span style="color: #8b9bb4;">Map Rev / Epoch:</span>
                <span id="sync-rev-epoch" style="color: #66fcf1; font-weight:600;">rev 0 | epoch 0</span>
            </div>
            <div style="display:flex; justify-content:space-between; margin-bottom: 4px;">
                <span style="color: #8b9bb4;">Anchor Tag:</span>
                <span id="sync-anchor-badge" style="color: #ffa500; font-weight:600;">⚠️ None</span>
            </div>
            <div style="display:flex; justify-content:space-between;">
                <span style="color: #8b9bb4;">Camera Extrinsics:</span>
                <span id="sync-extrinsics-badge" style="color: #e74c3c; font-weight:600;">UNVERIFIED</span>
            </div>
        </div>

        <div class="section-title">Текущие Координаты</div>
        <div class="card coords-grid">
            <div class="coord-box">
                <span class="coord-label">Ось X</span>
                <span class="coord-val" id="val-x">0.000<span>m</span></span>
            </div>
            <div class="coord-box">
                <span class="coord-label">Ось Y</span>
                <span class="coord-val" id="val-y">0.000<span>m</span></span>
            </div>
            <div class="coord-box">
                <span class="coord-label">Высота Z</span>
                <span class="coord-val" id="val-z">0.000<span>m</span></span>
            </div>
            <div class="coord-box">
                <span class="coord-label">Угол Yaw</span>
                <span class="coord-val" id="val-yaw">0.0<span>°</span></span>
            </div>
        </div>

        <div class="section-title">Статистика Движения</div>
        <div class="card">
            <div class="stat-item">
                <span class="stat-label">Сырой путь (с шумом)</span>
                <span class="stat-val" id="stat-dist-raw">0.00 m</span>
            </div>
            <div class="stat-item">
                <span class="stat-label">Сглаженный путь (фильтр)</span>
                <span class="stat-val" id="stat-dist-filt">0.00 m</span>
            </div>
            <div class="stat-item" style="flex-direction: column; align-items: flex-start; border: none; padding-bottom: 0;">
                <span class="stat-label" style="margin-bottom: 6px;">Видимые метки</span>
                <div class="active-tags-grid" id="active-tags">
                    <span style="color: #666; font-style: italic; font-size: 11px;">Нет видимых меток</span>
                </div>
            </div>
        </div>

        <button class="btn" id="btn-autocenter">Автоцентрирование: ВКЛ</button>
        <button class="btn btn-secondary" id="btn-reset">Сбросить Траекторию & Вид</button>

        <div class="section-title" style="margin-top: 15px;">Карта меток ArUco (Schema v2)</div>
        <div class="calib-container" style="display: flex; flex-direction: column; gap: 8px; margin-bottom: 12px;">
            <div style="display: flex; justify-content: space-between; align-items: center;">
                <span style="font-size: 12px; color: #8b9bb4;">Метки в реестре:</span>
                <button class="btn btn-secondary" id="btn-refresh-tags" style="font-size: 11px; padding: 4px 8px; margin: 0;">Обновить</button>
            </div>
            <div id="tag-registry-table-container" style="max-height: 150px; overflow-y: auto; font-size: 11px; background: rgba(0,0,0,0.25); border-radius: 4px; padding: 4px;">
                <div style="color: #666; font-style: italic; padding: 4px;">Загрузка меток...</div>
            </div>
            <div class="tag-form">
                <input class="calib-input tag-id" type="number" id="input-new-tag-id" placeholder="ID" min="0" max="99">
                <input class="calib-input" type="number" id="input-new-tag-size" placeholder="Сторона (мм)" step="1" min="20" max="1000">
                <input class="calib-input" type="number" id="input-new-tag-x" placeholder="X (м)" step="0.01">
                <input class="calib-input" type="number" id="input-new-tag-y" placeholder="Y (м)" step="0.01">
                <button class="btn tag-save" id="btn-add-tag-save" style="font-size: 11px; padding: 7px 8px; margin: 0;">💾 Сохранить координаты</button>
            </div>
        </div>

        <div class="section-title" style="margin-top: 15px;">Мастер автокалибровки метки</div>
        <div class="calib-container" style="display: flex; flex-direction: column; gap: 8px; margin-bottom: 12px;">
            <div style="display:flex; align-items:center; justify-content:space-between;">
                <span style="font-size:12px; color:#8b9bb4;">Целевой ID:</span>
                <input type="number" id="wizard-target-tag" class="calib-input" value="17" min="0" max="99" style="width:50px; text-align:center;">
                <span id="wizard-status-badge" style="font-weight:600; color:#45a29e; font-size:11px;">IDLE</span>
            </div>
            <div class="wizard-controls">
                <button class="btn" id="btn-wizard-start" style="flex: 1; font-size: 11px; padding: 8px 4px; background-color: #2ecc71; border-color: #2ecc71; margin: 0;">Центрировать & Обучить</button>
                <button class="btn btn-secondary" id="btn-wizard-confirm" style="display:none; flex: 1; font-size: 11px; padding: 8px 4px; background-color: #3498db; border-color: #3498db; color: white; margin: 0;">✅ Подтвердить</button>
                <button class="btn btn-secondary" id="btn-wizard-abort" style="flex: 1; font-size: 11px; padding: 8px 4px; background-color: #e74c3c; border-color: #e74c3c; color: white; margin: 0;">Стоп</button>
            </div>
            <div id="wizard-diagnostics" style="font-size: 11px; color: #8b9bb4; line-height: 1.3; background: rgba(0,0,0,0.2); border-radius: 4px; padding: 4px;">
                Готов к запуску.
            </div>
        </div>

        <div class="section-title" style="margin-top: 15px;">Физические параметры</div>
        <div class="calib-container">
            <div class="calib-row"><span class="stat-label">Диаметр колеса (мм):</span><input type="number" id="input-wheel-diameter" class="calib-input" min="20" max="300" step="0.1"></div>
            <div class="calib-row"><span class="stat-label">Сторона метки (мм):</span><input type="number" id="input-default-tag-size" class="calib-input" min="20" max="1000" step="1"></div>
            <div class="calib-row"><span class="stat-label">Высота потолка (м):</span><input type="number" id="input-ceiling-height" class="calib-input" min="0.2" max="20" step="0.01"></div>
            <button class="btn" id="btn-save-physical" style="margin: 4px 0 0;">Сохранить параметры</button>
        </div>

        <div class="section-title" style="margin-top: 20px;">Калибровка моторов</div>
        <div class="calib-container">
            <div class="calib-row">
                <span class="stat-label">Колеса (Wheel Mult):</span>
                <input type="number" id="input-wheel-mult" class="calib-input" value="1.000" step="0.005" min="0.5" max="1.5">
            </div>
            <div class="calib-row">
                <span class="stat-label">База (Base Mult):</span>
                <input type="number" id="input-base-mult" class="calib-input" value="1.000" step="0.005" min="0.5" max="1.5">
            </div>
            <button class="btn" id="btn-set-calib" style="margin-top: 8px; font-size: 13px;">Применить коэффициенты</button>
        </div>

        <div class="section-title">Тестовые движения</div>
        <div style="display: flex; gap: 8px; margin-bottom: 8px;">
            <button class="btn btn-secondary" id="btn-test-forward" style="flex: 1; font-size: 12px; padding: 10px 4px;">1м Вперед</button>
            <button class="btn btn-secondary" id="btn-test-rotate" style="flex: 1; font-size: 12px; padding: 10px 4px;">Поворот 360°</button>
        </div>
        <button class="btn" id="btn-test-stop" style="background-color: #ff4d4d; color: white; box-shadow: 0 0 10px rgba(255, 77, 77, 0.3); border-color: #ff4d4d; margin-bottom: 12px; font-size: 13px;">ОСТАНОВИТЬ ДВИЖЕНИЕ</button>

        <div class="section-title" style="margin-top: 15px;">Питание моторов (Ток)</div>
        <div class="calib-container" style="display: flex; flex-direction: column; gap: 8px; margin-bottom: 12px;">
            <div style="display: flex; align-items: center; justify-content: space-between; padding: 6px 10px; background: rgba(0,0,0,0.3); border-radius: 6px; font-size: 12px;">
                <span style="color: #8b9bb4;">Обмотки моторов:</span>
                <span id="motor-power-badge" style="font-weight: 600; color: #2ecc71;">⚡ ПОД ТОКОМ (УДЕРЖАНИЕ)</span>
            </div>
            <div style="display: flex; gap: 8px;">
                <button class="btn" id="btn-power-on" style="flex: 1; margin-bottom: 0; font-size: 12px; padding: 10px 4px; background-color: #2ecc71; color: white; border-color: #2ecc71; box-shadow: 0 0 10px rgba(46, 204, 113, 0.3);">⚡ Подать ток</button>
                <button class="btn btn-secondary" id="btn-power-off" style="flex: 1; margin-bottom: 0; font-size: 12px; padding: 10px 4px; background-color: #34495e; color: #ecf0f1; border-color: #7f8c8d;">💤 Снять ток</button>
            </div>
            <div style="font-size: 11px; color: #8b9bb4; line-height: 1.3;">
                Ток снимается автоматически через 2 секунды после последней команды или при финише маршрута.
            </div>
        </div>

        <div class="section-title" style="margin-top: 15px;">Автопилот (Маршруты)</div>
        <div class="calib-container" style="display: flex; flex-direction: column; gap: 8px;">
            <div style="display: flex; align-items: center; justify-content: space-between; padding: 6px 10px; background: rgba(0,0,0,0.3); border-radius: 6px; font-size: 12px;">
                <span style="color: #8b9bb4;">Статус:</span>
                <span id="route-status-badge" style="font-weight: 600; color: #45a29e;">ОЖИДАНИЕ</span>
            </div>
            <div style="display: flex; align-items: center; justify-content: space-between; padding: 2px 4px; font-size: 11px; color: #8b9bb4;">
                <span>Прогресс: <span id="route-progress-text" style="color: #fff; font-weight: 500;">0 / 0</span></span>
                <span>Режим: <span id="tracking-mode-badge" style="color: #2ecc71; font-weight: 500;">ArUco Fusion</span></span>
            </div>
            
            <button class="btn btn-secondary" id="btn-draw-mode" style="margin-bottom: 0; font-size: 13px;">✏️ Режим рисования: ВЫКЛ</button>
            
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 8px;">
                <button class="btn" id="btn-route-start" style="margin-bottom: 0; font-size: 13px; padding: 10px 4px; background-color: #2ecc71; color: white; box-shadow: 0 0 10px rgba(46, 204, 113, 0.3); border-color: #2ecc71;">▶ Старт</button>
                <button class="btn btn-secondary" id="btn-route-pause" style="margin-bottom: 0; font-size: 13px; padding: 10px 4px; background-color: #f39c12; color: white; border-color: #f39c12;">⏸ Пауза</button>
                <button class="btn btn-secondary" id="btn-route-stop" style="margin-bottom: 0; font-size: 13px; padding: 10px 4px; background-color: #e74c3c; color: white; border-color: #e74c3c;">⏹ Стоп</button>
                <button class="btn btn-secondary" id="btn-clear-plan" style="margin-bottom: 0; font-size: 13px; padding: 10px 4px;">🗑 Очистить</button>
            </div>
            
            <button class="btn btn-secondary" id="btn-return-home" style="margin-bottom: 0; font-size: 13px; padding: 9px 4px; background-color: #3498db; color: white; border-color: #3498db; box-shadow: 0 0 10px rgba(52, 152, 219, 0.25);">🎯 Приехать в ноль (0, 0)</button>
            
            <div style="border-top: 1px solid rgba(255,255,255,0.08); padding-top: 8px; margin-top: 4px;">
                <div class="calib-row" style="margin-bottom: 8px;">
                    <span class="stat-label" style="font-size: 11px;">Шаг точек (м):</span>
                    <input type="number" id="input-gen-spacing" class="calib-input" value="0.05" step="0.01" min="0.01" max="0.50" style="font-size: 11px; padding: 1px 4px; width: 60px;">
                </div>
                <div style="display: flex; gap: 6px;">
                    <button class="btn btn-secondary" id="btn-gen-square" style="flex: 1; font-size: 11px; padding: 6px 2px; margin-bottom: 0;">Квадрат 1м</button>
                    <button class="btn btn-secondary" id="btn-gen-circle" style="flex: 1; font-size: 11px; padding: 6px 2px; margin-bottom: 0;">Круг R=1м</button>
                </div>
            </div>
        </div>

        <div class="section-title">Настройки автопилота</div>
        <div class="calib-container">
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Look-ahead (м):</span>
                <input type="number" id="input-look-ahead" class="calib-input" value="0.15" step="0.01" min="0.01" max="1.50" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Макс. линейная (м/с):</span>
                <input type="number" id="input-max-lin" class="calib-input" value="0.14" step="0.01" min="0.05" max="0.50" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Макс. угловая (рад/с):</span>
                <input type="number" id="input-max-ang" class="calib-input" value="0.70" step="0.01" min="0.10" max="3.00" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Kp Линейный:</span>
                <input type="number" id="input-kp-lin" class="calib-input" value="0.80" step="0.01" min="0.10" max="5.00" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Kp Угловой:</span>
                <input type="number" id="input-kp-ang" class="calib-input" value="1.50" step="0.01" min="0.10" max="5.00" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Точность финиша (м):</span>
                <input type="number" id="input-goal-tol" class="calib-input" value="0.04" step="0.01" min="0.01" max="0.50" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Торможение (м):</span>
                <input type="number" id="input-decel-dist" class="calib-input" value="0.30" step="0.01" min="0.05" max="1.50" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Мин. линейная (м/с):</span>
                <input type="number" id="input-min-lin" class="calib-input" value="0.03" step="0.01" min="0.01" max="0.20" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Мертвая зона Yaw (м):</span>
                <input type="number" id="input-yaw-deadzone" class="calib-input" value="0.05" step="0.01" min="0.01" max="0.30" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Точность точек (м):</span>
                <input type="number" id="input-wp-tol" class="calib-input" value="0.08" step="0.01" min="0.01" max="0.50" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <div class="calib-row">
                <span class="stat-label" style="font-size: 12px;">Торможение в повороте:</span>
                <input type="number" id="input-turn-decel" class="calib-input" value="0.20" step="0.01" min="0.00" max="1.50" style="font-size: 12px; padding: 2px 6px;">
            </div>
            <button class="btn btn-secondary" id="btn-set-follower-params" style="margin-top: 8px; font-size: 12px; padding: 8px 4px;">Применить параметры</button>
        </div>

        <div class="section-title" style="margin-top: 10px;">Ручное управление</div>
        <div class="control-box">
            <div style="display: flex; gap: 8px; margin-bottom: 12px; width: 100%;">
                <button class="btn btn-secondary" id="btn-rot-ccw" style="flex: 1; font-size: 13px; padding: 10px 4px; margin-bottom: 0;">↺ Влево</button>
                <button class="btn btn-secondary" id="btn-rot-cw" style="flex: 1; font-size: 13px; padding: 10px 4px; margin-bottom: 0;">Вправо ↻</button>
            </div>
            
            <div class="joystick-zone">
                <div id="joystick-base">
                    <div id="joystick-handle"></div>
                </div>
            </div>
        </div>

        <div id="terminal-log">
            <div class="log-entry"><span class="log-time">Система</span>Веб-интерфейс готов к получению данных.</div>
        </div>
    </div>

    <div id="map-container">
        <div class="video-panel">
            <div class="video-box" id="video-box">
                <div class="video-box-header">
                    <div class="video-title">
                        <span class="video-dot"></span>
                        Камера (ArUco)
                    </div>
                    <button class="video-toggle-btn" id="btn-toggle-cam" title="Свернуть / Развернуть">▼</button>
                </div>
                <div class="video-content" id="video-content">
                    <img id="camera-stream" src="/video_feed" alt="Загрузка видео..." />
                </div>
            </div>
            <div class="control-panel">
                <button class="icon-btn active" id="toggle-raw">Показать сырой путь</button>
                <button class="icon-btn" id="toggle-grid">Сетка</button>
            </div>
        </div>
        <canvas id="map-canvas"></canvas>
    </div>

    <script>
        const canvas = document.getElementById('map-canvas');
        const ctx = canvas.getContext('2d');

        let tags = {};
        let rawHistory = [];
        let filteredHistory = [];
        let robotPos = { x: 0, y: 0, z: 0, yaw: 0 };
        let activeTags = [];
        let isConnected = false;
        let showRaw = true;
        let showGrid = true;

        // Масштаб и сдвиг
        let zoom = 120; // Пикселей на метр
        let panX = 0;   // Сдвиг по X (в пикселях)
        let panY = 0;   // Сдвиг по Y (в пикселях)
        let isDragging = false;
        let startX, startY;
        let autoCenter = true;

        function resizeCanvas() {
            canvas.width = canvas.parentElement.clientWidth;
            canvas.height = canvas.parentElement.clientHeight;
            if (autoCenter) centerMap();
            draw();
        }
        window.addEventListener('resize', resizeCanvas);

        async function fetchConfig() {
            try {
                addLog("Запрос конфигурации меток...");
                const res = await fetch('/config');
                tags = await res.json();
                addLog("Загружено меток с сервера: " + Object.keys(tags).length);
                centerMap();
                draw();
            } catch (e) {
                console.error("Не удалось загрузить конфиг:", e);
                addLog("Ошибка загрузки конфигурации меток.");
            }
        }

        function centerMap() {
            panX = canvas.width / 2 - robotPos.x * zoom;
            panY = canvas.height / 2 + robotPos.y * zoom;
        }

        function addLog(text) {
            const logDiv = document.getElementById('terminal-log');
            const entry = document.createElement('div');
            entry.className = 'log-entry';
            const timeStr = new Date().toLocaleTimeString();
            entry.innerHTML = `<span class="log-time">${timeStr}</span>${text}`;
            logDiv.appendChild(entry);
            logDiv.scrollTop = logDiv.scrollHeight;
        }

        function draw() {
            ctx.fillStyle = '#0f1015';
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            // Отрисовка сетки
            if (showGrid) {
                let step = 1.0;
                let labelInterval = 1;
                
                if (zoom < 35) {
                    step = 2.0;
                    labelInterval = 1;
                } else if (zoom < 85) {
                    step = 1.0;
                    labelInterval = 1;
                } else if (zoom < 185) {
                    step = 0.5;
                    labelInterval = 2; // каждые 1.0м
                } else if (zoom < 450) {
                    step = 0.2;
                    labelInterval = 5; // каждые 1.0м
                } else if (zoom < 1000) {
                    step = 0.1;
                    labelInterval = 5; // каждые 0.5м
                } else if (zoom < 2000) {
                    step = 0.05;
                    labelInterval = 4; // каждые 0.2м
                } else {
                    step = 0.02;
                    labelInterval = 5; // каждые 0.1м
                }

                ctx.lineWidth = 1;

                const minX = (0 - panX) / zoom;
                const maxX = (canvas.width - panX) / zoom;
                const minY = (panY - canvas.height) / zoom;
                const maxY = (panY) / zoom;

                const startValX = Math.floor(minX / step);
                const endValX = Math.ceil(maxX / step);
                const startValY = Math.floor(minY / step);
                const endValY = Math.ceil(maxY / step);

                // Вертикальные линии
                for (let i = startValX; i <= endValX; i++) {
                    const x = i * step;
                    ctx.strokeStyle = Math.abs(x) < 0.001 ? 'rgba(102, 252, 241, 0.25)' : 'rgba(255, 255, 255, 0.03)';
                    ctx.beginPath();
                    const px = panX + x * zoom;
                    ctx.moveTo(px, 0);
                    ctx.lineTo(px, canvas.height);
                    ctx.stroke();

                    if (i % labelInterval === 0) {
                        ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
                        ctx.font = '10px monospace';
                        ctx.textAlign = 'center';
                        
                        let labelText = x.toFixed(2);
                        if (step >= 0.1) {
                            labelText = x.toFixed(1);
                        }
                        if (Math.abs(x) % 1 === 0) {
                            labelText = Math.round(x).toString();
                        }
                        ctx.fillText(labelText + 'm', px, canvas.height - 10);
                    }
                }

                // Горизонтальные линии
                for (let i = startValY; i <= endValY; i++) {
                    const y = i * step;
                    ctx.strokeStyle = Math.abs(y) < 0.001 ? 'rgba(255, 77, 77, 0.25)' : 'rgba(255, 255, 255, 0.03)';
                    ctx.beginPath();
                    const py = panY - y * zoom;
                    ctx.moveTo(0, py);
                    ctx.lineTo(canvas.width, py);
                    ctx.stroke();

                    if (i % labelInterval === 0) {
                        ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
                        ctx.font = '10px monospace';
                        ctx.textAlign = 'left';
                        
                        let labelText = y.toFixed(2);
                        if (step >= 0.1) {
                            labelText = y.toFixed(1);
                        }
                        if (Math.abs(y) % 1 === 0) {
                            labelText = Math.round(y).toString();
                        }
                        ctx.fillText(labelText + 'm', 10, py - 4);
                    }
                }
            }

            // Отрисовка потолочных меток
            for (const [tagId, info] of Object.entries(tags)) {
                const px = panX + info.x * zoom;
                const py = panY - info.y * zoom;
                const size = 18;

                // Подсветка активной метки
                const isActive = activeTags.includes(parseInt(tagId));
                
                ctx.fillStyle = isActive ? 'rgba(46, 204, 113, 0.15)' : 'rgba(255, 165, 0, 0.08)';
                ctx.strokeStyle = isActive ? '#2ecc71' : 'rgba(255, 165, 0, 0.6)';
                ctx.lineWidth = isActive ? 2.5 : 1.5;
                
                ctx.fillRect(px - size/2, py - size/2, size, size);
                ctx.strokeRect(px - size/2, py - size/2, size, size);

                // Номер метки
                ctx.fillStyle = isActive ? '#2ecc71' : '#ffa500';
                ctx.font = 'bold 11px Outfit, Arial';
                ctx.textAlign = 'center';
                ctx.fillText('Tag ' + tagId, px, py - size/2 - 4);
            }

            // Отрисовка сырой (зашумленной) траектории
            if (showRaw && rawHistory.length > 1) {
                ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
                ctx.lineWidth = 1.2;
                ctx.setLineDash([2, 3]);
                ctx.beginPath();
                ctx.moveTo(panX + rawHistory[0].x * zoom, panY - rawHistory[0].y * zoom);
                for (let i = 1; i < rawHistory.length; i++) {
                    ctx.lineTo(panX + rawHistory[i].x * zoom, panY - rawHistory[i].y * zoom);
                }
                ctx.stroke();
                ctx.setLineDash([]);
            }

            // Отрисовка отфильтрованной траектории
            if (filteredHistory.length > 1) {
                ctx.strokeStyle = '#66fcf1';
                ctx.lineWidth = 3;
                ctx.shadowColor = '#66fcf1';
                ctx.shadowBlur = 6;
                ctx.beginPath();
                ctx.moveTo(panX + filteredHistory[0].x * zoom, panY - filteredHistory[0].y * zoom);
                for (let i = 1; i < filteredHistory.length; i++) {
                    ctx.lineTo(panX + filteredHistory[i].x * zoom, panY - filteredHistory[i].y * zoom);
                }
                ctx.stroke();
                ctx.shadowBlur = 0;
            }

            // Отрисовка нарисованного маршрута автопилота
            if (plannedPath.length > 0) {
                ctx.strokeStyle = 'rgba(168, 85, 247, 0.8)';
                ctx.lineWidth = 3;
                ctx.beginPath();
                ctx.moveTo(panX + plannedPath[0].x * zoom, panY - plannedPath[0].y * zoom);
                for (let i = 1; i < plannedPath.length; i++) {
                    ctx.lineTo(panX + plannedPath[i].x * zoom, panY - plannedPath[i].y * zoom);
                }
                ctx.stroke();
                
                for (let i = 0; i < plannedPath.length; i++) {
                    ctx.fillStyle = i === 0 ? '#2ecc71' : (i === plannedPath.length - 1 ? '#ff4d4d' : '#a855f7');
                    ctx.beginPath();
                    ctx.arc(panX + plannedPath[i].x * zoom, panY - plannedPath[i].y * zoom, 5, 0, 2 * Math.PI);
                    ctx.fill();
                    ctx.strokeStyle = '#fff';
                    ctx.lineWidth = 1;
                    ctx.stroke();
                    
                    ctx.fillStyle = '#fff';
                    ctx.font = '10px monospace';
                    ctx.fillText(i + 1, panX + plannedPath[i].x * zoom + 8, panY - plannedPath[i].y * zoom - 4);
                }
            }

            // Отрисовка текущего положения робота
            const rpx = panX + robotPos.x * zoom;
            const rpy = panY - robotPos.y * zoom;

            ctx.fillStyle = '#2ecc71';
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 2;
            ctx.shadowColor = '#2ecc71';
            ctx.shadowBlur = 10;
            ctx.beginPath();
            ctx.arc(rpx, rpy, 9, 0, 2 * Math.PI);
            ctx.fill();
            ctx.stroke();
            ctx.shadowBlur = 0;

            // Стрелка направления (Yaw)
            const yawRad = robotPos.yaw * Math.PI / 180;
            const arrowLen = 16;
            const ax = rpx + Math.cos(yawRad) * arrowLen;
            const ay = rpy - Math.sin(yawRad) * arrowLen;

            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2.5;
            ctx.beginPath();
            ctx.moveTo(rpx, rpy);
            ctx.lineTo(ax, ay);
            ctx.stroke();
            
            // Наконечник стрелки
            const headlen = 5;
            ctx.fillStyle = '#ffffff';
            ctx.beginPath();
            ctx.moveTo(ax, ay);
            ctx.lineTo(ax - headlen * Math.cos(yawRad - Math.PI/6), ay + headlen * Math.sin(yawRad - Math.PI/6));
            ctx.lineTo(ax - headlen * Math.cos(yawRad + Math.PI/6), ay + headlen * Math.sin(yawRad + Math.PI/6));
            ctx.fill();
        }

        // Panning (Перетаскивание)
        canvas.addEventListener('mousedown', (e) => {
            if (drawMode) {
                dragStartX = e.clientX;
                dragStartY = e.clientY;
            }
            isDragging = true;
            startX = e.clientX - panX;
            startY = e.clientY - panY;
            autoCenter = false;
            document.getElementById('btn-autocenter').className = 'btn btn-secondary';
            document.getElementById('btn-autocenter').textContent = 'Автоцентрирование: ВЫКЛ';
        });

        canvas.addEventListener('mousemove', (e) => {
            if (isDragging) {
                panX = e.clientX - startX;
                panY = e.clientY - startY;
                draw();
            }
        });

        canvas.addEventListener('mouseup', () => isDragging = false);
        canvas.addEventListener('mouseleave', () => isDragging = false);

        // Поддержка Touch-событий для мобильных устройств (перетаскивание и pinch-to-zoom)
        let isTouching = false;
        let startTouchX = 0;
        let startTouchY = 0;
        let initialTouchDist = 0;
        let initialZoom = 0;
        
        canvas.addEventListener('touchstart', (e) => {
            if (e.touches.length === 1) {
                isTouching = true;
                startTouchX = e.touches[0].clientX - panX;
                startTouchY = e.touches[0].clientY - panY;
                autoCenter = false;
                document.getElementById('btn-autocenter').className = 'btn btn-secondary';
                document.getElementById('btn-autocenter').textContent = 'Автоцентрирование: ВЫКЛ';
                
                if (drawMode) {
                    dragStartX = e.touches[0].clientX;
                    dragStartY = e.touches[0].clientY;
                }
            } else if (e.touches.length === 2) {
                isTouching = false;
                const dx = e.touches[0].clientX - e.touches[1].clientX;
                const dy = e.touches[0].clientY - e.touches[1].clientY;
                initialTouchDist = Math.sqrt(dx * dx + dy * dy);
                initialZoom = zoom;
            }
        });
        
        canvas.addEventListener('touchmove', (e) => {
            if (isTouching && e.touches.length === 1) {
                panX = e.touches[0].clientX - startTouchX;
                panY = e.touches[0].clientY - startTouchY;
                draw();
            } else if (e.touches.length === 2 && initialTouchDist > 0) {
                const dx = e.touches[0].clientX - e.touches[1].clientX;
                const dy = e.touches[0].clientY - e.touches[1].clientY;
                const dist = Math.sqrt(dx * dx + dy * dy);
                
                const zoomFactor = dist / initialTouchDist;
                zoom = Math.min(Math.max(initialZoom * zoomFactor, 15), 3000);
                draw();
            }
        });
        
        canvas.addEventListener('touchend', (e) => {
            if (isTouching) {
                isTouching = false;
                
                if (drawMode && e.changedTouches.length > 0) {
                    const endX = e.changedTouches[0].clientX;
                    const endY = e.changedTouches[0].clientY;
                    const dist = Math.sqrt((endX - dragStartX)**2 + (endY - dragStartY)**2);
                    if (dist < 5) {
                        const rect = canvas.getBoundingClientRect();
                        const mouseX = endX - rect.left;
                        const mouseY = endY - rect.top;
                        
                        const x = (mouseX - panX) / zoom;
                        const y = (panY - mouseY) / zoom;
                        
                        plannedPath.push({ x, y });
                        addLog(`Точка маршрута: X=${x.toFixed(2)}, Y=${y.toFixed(2)}`);
                        draw();
                    }
                }
            }
            if (e.touches.length < 2) {
                initialTouchDist = 0;
            }
        });

        // Zoom (Масштабирование)
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
            const mouseX = e.clientX - canvas.getBoundingClientRect().left;
            const mouseY = e.clientY - canvas.getBoundingClientRect().top;

            const xMeters = (mouseX - panX) / zoom;
            const yMeters = (panY - mouseY) / zoom;

            zoom = Math.min(Math.max(zoom * zoomFactor, 15), 3000);

            panX = mouseX - xMeters * zoom;
            panY = mouseY + yMeters * zoom;

            draw();
        });

        // Кнопки управления
        document.getElementById('btn-autocenter').addEventListener('click', () => {
            autoCenter = !autoCenter;
            const btn = document.getElementById('btn-autocenter');
            if (autoCenter) {
                btn.className = 'btn';
                btn.textContent = 'Автоцентрирование: ВКЛ';
                centerMap();
                draw();
            } else {
                btn.className = 'btn btn-secondary';
                btn.textContent = 'Автоцентрирование: ВЫКЛ';
            }
        });

        document.getElementById('btn-reset').addEventListener('click', () => {
            rawHistory = [];
            filteredHistory = [];
            autoCenter = true;
            zoom = 120;
            const btn = document.getElementById('btn-autocenter');
            btn.className = 'btn';
            btn.textContent = 'Автоцентрирование: ВКЛ';
            centerMap();
            draw();
            addLog("Траектория сброшена.");
        });

        document.getElementById('toggle-raw').addEventListener('click', () => {
            showRaw = !showRaw;
            const btn = document.getElementById('toggle-raw');
            btn.className = showRaw ? 'icon-btn active' : 'icon-btn';
            draw();
        });

        document.getElementById('toggle-grid').addEventListener('click', () => {
            showGrid = !showGrid;
            const btn = document.getElementById('toggle-grid');
            btn.className = showGrid ? 'icon-btn active' : 'icon-btn';
            draw();
        });

        const btnToggleCam = document.getElementById('btn-toggle-cam');
        const videoBox = document.getElementById('video-box');
        if (btnToggleCam && videoBox) {
            btnToggleCam.addEventListener('click', () => {
                videoBox.classList.toggle('minimized');
                btnToggleCam.textContent = videoBox.classList.contains('minimized') ? '▲' : '▼';
            });
        }

        function updateActiveTagsUI() {
            const container = document.getElementById('active-tags');
            if (activeTags.length === 0) {
                container.innerHTML = '<span style="color: #666; font-style: italic; font-size: 11px;">Нет видимых меток</span>';
                return;
            }
            container.innerHTML = '';
            activeTags.forEach(tagId => {
                const badge = document.createElement('span');
                badge.className = 'tag-badge';
                badge.textContent = '#' + tagId;
                container.appendChild(badge);
            });
        }

        // Подключение к потоку данных SSE
        function connectSSE() {
            const sse = new EventSource('/events');
            const statusDot = document.getElementById('status-dot');
            const statusText = document.getElementById('status-text');

            sse.onopen = () => {
                isConnected = true;
                statusDot.className = 'status-dot connected';
                statusText.textContent = 'CONNECTED';
                addLog("Успешное соединение с ROS 2.");
            };

            sse.onerror = () => {
                if (isConnected) {
                    isConnected = false;
                    statusDot.className = 'status-dot';
                    statusText.textContent = 'DISCONNECTED';
                    addLog("Соединение с сервером потеряно.");
                }
            };

            sse.onmessage = (e) => {
                const data = JSON.parse(e.data);
                if (data.type === 'pose') {
                    robotPos.x = data.x;
                    robotPos.y = data.y;
                    robotPos.z = data.z;
                    robotPos.yaw = data.yaw;

                    filteredHistory.push({ x: data.x, y: data.y });
                    rawHistory.push({ x: data.raw_x, y: data.raw_y });

                    if (filteredHistory.length > 2000) filteredHistory.shift();
                    if (rawHistory.length > 2000) rawHistory.shift();

                    document.getElementById('val-x').innerHTML = data.x.toFixed(3) + '<span>m</span>';
                    document.getElementById('val-y').innerHTML = data.y.toFixed(3) + '<span>m</span>';
                    document.getElementById('val-z').innerHTML = data.z.toFixed(3) + '<span>m</span>';
                    document.getElementById('val-yaw').innerHTML = data.yaw.toFixed(1) + '<span>°</span>';

                    document.getElementById('stat-dist-raw').textContent = data.distance_raw.toFixed(2) + ' m';
                    document.getElementById('stat-dist-filt').textContent = data.distance_filtered.toFixed(2) + ' m';

                    activeTags = data.detected_tags || [];
                    updateActiveTagsUI();

                    // Обновляем состояние автопилота и статуса
                    updateRouteStatusUI(data);

                    if (autoCenter) {
                        centerMap();
                    }
                    draw();
                } else if (data.type === 'route_status') {
                    updateRouteStatusUI(data);
                }
            };
        }

        function updateRouteStatusUI(data) {
            const badge = document.getElementById('route-status-badge');
            const progress = document.getElementById('route-progress-text');
            const modeBadge = document.getElementById('tracking-mode-badge');
            const btnPause = document.getElementById('btn-route-pause');
            
            const rState = data.route_state || data.follower_status;
            if (rState) {
                if (badge) {
                    if (rState === 'running' || rState === 'tracking' || rState === 'pre_positioning') {
                        badge.textContent = "В ДВИЖЕНИИ";
                        badge.style.color = "#2ecc71";
                    } else if (rState === 'paused') {
                        badge.textContent = "НА ПАУЗЕ";
                        badge.style.color = "#f39c12";
                    } else if (rState === 'finished') {
                        badge.textContent = "ФИНИШ";
                        badge.style.color = "#3498db";
                    } else {
                        badge.textContent = "ОЖИДАНИЕ";
                        badge.style.color = "#8b9bb4";
                    }
                }
                if (btnPause) {
                    btnPause.textContent = (rState === 'paused') ? "▶ Продолжить" : "⏸ Пауза";
                    btnPause.style.backgroundColor = (rState === 'paused') ? "#2ecc71" : "#f39c12";
                    btnPause.style.borderColor = (rState === 'paused') ? "#2ecc71" : "#f39c12";
                }
            }
            if (progress && data.total_wps !== undefined) {
                const cur = (data.total_wps > 0 && data.current_wp !== undefined) ? (data.current_wp + 1) : 0;
                progress.textContent = `${cur} / ${data.total_wps}`;
            }
            if (modeBadge && data.tracking_mode) {
                if (data.tracking_mode === 'aruco_fused') {
                    modeBadge.textContent = "ArUco Fusion";
                    modeBadge.style.color = "#2ecc71";
                } else {
                    modeBadge.textContent = "Dead Reckoning";
                    modeBadge.style.color = "#f39c12";
                }
            }
            if (data.motor_power) {
                const pBadge = document.getElementById('motor-power-badge');
                if (pBadge) {
                    if (data.motor_power === 'enabled') {
                        pBadge.textContent = "⚡ ПОД ТОКОМ (УДЕРЖАНИЕ)";
                        pBadge.style.color = "#2ecc71";
                    } else {
                        pBadge.textContent = "💤 ТОК СНЯТ (СВОБОДНЫЙ ВАЛ)";
                        pBadge.style.color = "#8b9bb4";
                    }
                }
            }
        }

        // Обработчики калибровки и тестов
        const inputWheelMult = document.getElementById('input-wheel-mult');
        const inputBaseMult = document.getElementById('input-base-mult');
        const btnSetCalib = document.getElementById('btn-set-calib');
        const btnTestForward = document.getElementById('btn-test-forward');
        const btnTestRotate = document.getElementById('btn-test-rotate');
        const btnTestStop = document.getElementById('btn-test-stop');

        btnSetCalib.addEventListener('click', () => {
            const wMult = parseFloat(inputWheelMult.value);
            const bMult = parseFloat(inputBaseMult.value);
            
            fetch(`/set_calib?wheel_mult=${wMult}&base_mult=${bMult}`)
                .then(res => res.json())
                .then(data => {
                    if (data.status === 'success') {
                        addLog(`Успех: Коэффициенты отправлены (Wheel=${wMult.toFixed(3)}, Base=${bMult.toFixed(3)})`);
                    } else {
                        addLog("Ошибка применения коэффициентов.");
                    }
                })
                .catch(err => {
                    addLog("Сеть: Ошибка калибровки.");
                });
        });

        const triggerTestDrive = (type, label) => {
            fetch(`/test_drive?type=${type}`)
                .then(res => res.json())
                .then(data => {
                    if (data.status === 'success') {
                        addLog(`Робот: ${label}`);
                    }
                })
                .catch(err => {
                    addLog(`Сеть: Ошибка запуска ${label}`);
                });
        };

        btnTestForward.addEventListener('click', () => triggerTestDrive('forward', 'Тест движения: 1 метр вперед'));
        btnTestRotate.addEventListener('click', () => triggerTestDrive('rotate', 'Тест движения: поворот на 360°'));
        btnTestStop.addEventListener('click', () => triggerTestDrive('stop', 'ЭКСТРЕННАЯ ОСТАНОВКА'));

        // Управление питанием моторов (снятие и подача тока на обмотки)
        const btnPowerOn = document.getElementById('btn-power-on');
        const btnPowerOff = document.getElementById('btn-power-off');
        if (btnPowerOn) {
            btnPowerOn.addEventListener('click', () => {
                fetch('/set_motor_power?state=enable')
                    .then(res => res.json())
                    .then(data => {
                        const pBadge = document.getElementById('motor-power-badge');
                        if (pBadge) {
                            pBadge.textContent = "⚡ ПОД ТОКОМ (УДЕРЖАНИЕ)";
                            pBadge.style.color = "#2ecc71";
                        }
                        addLog("⚡ Ток подан на обмотки моторов (удержание активно).");
                    })
                    .catch(err => addLog("Сеть: Ошибка подачи тока на моторы."));
            });
        }
        if (btnPowerOff) {
            btnPowerOff.addEventListener('click', () => {
                fetch('/set_motor_power?state=disable')
                    .then(res => res.json())
                    .then(data => {
                        const pBadge = document.getElementById('motor-power-badge');
                        if (pBadge) {
                            pBadge.textContent = "💤 ТОК СНЯТ (СВОБОДНЫЙ ВАЛ)";
                            pBadge.style.color = "#8b9bb4";
                        }
                        addLog("💤 Ток с обмоток снят (валы свободны, охлаждение).");
                    })
                    .catch(err => addLog("Сеть: Ошибка снятия тока с моторов."));
            });
        }

        // Переменные автопилота
        let drawMode = false;
        let plannedPath = [];
        let dragStartX = 0;
        let dragStartY = 0;
        
        const btnDrawMode = document.getElementById('btn-draw-mode');
        const btnRouteStart = document.getElementById('btn-route-start');
        const btnRoutePause = document.getElementById('btn-route-pause');
        const btnRouteStop = document.getElementById('btn-route-stop');
        const btnClearPlan = document.getElementById('btn-clear-plan');
        const btnReturnHome = document.getElementById('btn-return-home');
        
        btnDrawMode.addEventListener('click', () => {
            drawMode = !drawMode;
            btnDrawMode.textContent = drawMode ? "✏️ Режим рисования: ВКЛ" : "✏️ Режим рисования: ВЫКЛ";
            btnDrawMode.style.borderColor = drawMode ? "#66fcf1" : "rgba(255, 255, 255, 0.1)";
            btnDrawMode.style.color = drawMode ? "#66fcf1" : "#c5c6c7";
            if (drawMode) {
                canvas.style.cursor = 'crosshair';
                addLog("Режим рисования включен. Кликайте на карту для расстановки точек.");
            } else {
                canvas.style.cursor = 'grab';
            }
        });

        canvas.addEventListener('click', (e) => {
            if (!drawMode) return;
            const dist = Math.sqrt((e.clientX - dragStartX)**2 + (e.clientY - dragStartY)**2);
            if (dist > 5) return; // это было перетаскивание карты
            
            const rect = canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            
            const x = (mouseX - panX) / zoom;
            const y = (panY - mouseY) / zoom;
            
            plannedPath.push({ x, y });
            addLog(`Точка маршрута: X=${x.toFixed(2)}, Y=${y.toFixed(2)}`);
            draw();
        });

        btnRouteStart.addEventListener('click', () => {
            if (plannedPath.length === 0) {
                addLog("⚠️ Сначала нарисуйте маршрут!");
                return;
            }
            
            if (drawMode) {
                drawMode = false;
                btnDrawMode.textContent = "✏️ Режим рисования: ВЫКЛ";
                btnDrawMode.style.borderColor = "rgba(255, 255, 255, 0.1)";
                btnDrawMode.style.color = "#c5c6c7";
                canvas.style.cursor = 'grab';
            }
            
            // Автоматически отправляем текущие параметры из полей ввода перед стартом
            const lookAhead = parseFloat(inputLookAhead.value);
            const maxLin = parseFloat(inputMaxLin.value);
            const maxAng = parseFloat(inputMaxAng.value);
            const kpLin = parseFloat(inputKpLin.value);
            const kpAng = parseFloat(inputKpAng.value);
            const goalTol = parseFloat(inputGoalTol.value);
            const decelDist = parseFloat(inputDecelDist.value);
            const minLin = parseFloat(inputMinLin.value);
            const yawDeadzone = parseFloat(inputYawDeadzone.value);
            const wpTol = parseFloat(inputWpTol.value);
            const turnDecel = parseFloat(inputTurnDecel.value);
            
            fetch(`/set_follower_params?look_ahead=${lookAhead}&max_lin=${maxLin}&max_ang=${maxAng}&kp_lin=${kpLin}&kp_ang=${kpAng}&goal_tol=${goalTol}&decel_dist=${decelDist}&min_lin=${minLin}&yaw_deadzone=${yawDeadzone}&wp_tol=${wpTol}&turn_decel=${turnDecel}`)
                .catch(err => console.error("Error setting follower params:", err));
            
            const ptsStr = plannedPath.map(pt => `${pt.x.toFixed(3)},${pt.y.toFixed(3)}`).join(';');
            fetch(`/set_path?points=${ptsStr}`)
                .then(res => res.json())
                .then(data => {
                    fetch(`/start_route`)
                        .then(r => r.json())
                        .then(resData => {
                            addLog("▶ Автопилот запущен. Робот начинает движение по маршруту!");
                        });
                })
                .catch(err => {
                    addLog("Сеть: Ошибка запуска маршрута.");
                });
        });

        btnRoutePause.addEventListener('click', () => {
            fetch('/pause_route')
                .then(res => res.json())
                .then(data => {
                    if (data.route_state === 'paused') {
                        addLog("⏸ Автопилот на паузе.");
                    } else if (data.route_state === 'running') {
                        addLog("▶ Движение возобновлено.");
                    }
                })
                .catch(err => addLog("Сеть: Ошибка переключения паузы."));
        });

        btnRouteStop.addEventListener('click', () => {
            fetch('/stop_route')
                .then(res => res.json())
                .then(data => {
                    addLog("⏹ Автопилот остановлен, маршрут сброшен.");
                })
                .catch(err => addLog("Сеть: Ошибка остановки маршрута."));
        });
        
        btnClearPlan.addEventListener('click', () => {
            plannedPath = [];
            fetch(`/clear_waypoints`)
                .then(res => res.json())
                .then(data => {
                    addLog("🗑 Маршрут и путевые точки очищены.");
                    draw();
                })
                .catch(err => {
                    addLog("Сеть: Ошибка очистки маршрута.");
                    draw();
                });
        });

        if (btnReturnHome) {
            btnReturnHome.addEventListener('click', () => {
                if (drawMode) {
                    drawMode = false;
                    btnDrawMode.textContent = "✏️ Режим рисования: ВЫКЛ";
                    btnDrawMode.style.borderColor = "rgba(255, 255, 255, 0.1)";
                    btnDrawMode.style.color = "#c5c6c7";
                    canvas.style.cursor = 'grab';
                }
                
                // Автоматически отправляем текущие параметры из полей ввода
                const lookAhead = parseFloat(inputLookAhead.value);
                const maxLin = parseFloat(inputMaxLin.value);
                const maxAng = parseFloat(inputMaxAng.value);
                const kpLin = parseFloat(inputKpLin.value);
                const kpAng = parseFloat(inputKpAng.value);
                const goalTol = parseFloat(inputGoalTol.value);
                const decelDist = parseFloat(inputDecelDist.value);
                const minLin = parseFloat(inputMinLin.value);
                const yawDeadzone = parseFloat(inputYawDeadzone.value);
                const wpTol = parseFloat(inputWpTol.value);
                const turnDecel = parseFloat(inputTurnDecel.value);
                
                fetch(`/set_follower_params?look_ahead=${lookAhead}&max_lin=${maxLin}&max_ang=${maxAng}&kp_lin=${kpLin}&kp_ang=${kpAng}&goal_tol=${goalTol}&decel_dist=${decelDist}&min_lin=${minLin}&yaw_deadzone=${yawDeadzone}&wp_tol=${wpTol}&turn_decel=${turnDecel}`)
                    .catch(err => console.error("Error setting follower params:", err));

                const rx = robotPos.x;
                const ry = robotPos.y;
                const dist = Math.sqrt(rx * rx + ry * ry);
                const numPts = Math.max(3, Math.ceil(dist / 0.05));
                plannedPath = [];
                for (let i = 0; i <= numPts; i++) {
                    const t = i / numPts;
                    plannedPath.push({ x: rx * (1.0 - t), y: ry * (1.0 - t) });
                }
                draw();
                addLog(`🎯 Возврат в (0,0): дистанция ${dist.toFixed(2)}м (${plannedPath.length} точек).`);
                
                const ptsStr = plannedPath.map(pt => `${pt.x.toFixed(3)},${pt.y.toFixed(3)}`).join(';');
                fetch(`/set_path?points=${ptsStr}`)
                    .then(res => res.json())
                    .then(() => fetch('/start_route'))
                    .then(r => r.json())
                    .then(() => addLog("▶ Автопилот запущен для возврата в ноль!"))
                    .catch(err => addLog("Сеть: Ошибка отправки маршрута возврата в ноль."));
            });
        }

        // Генерация тестовых контуров
        const btnGenSquare = document.getElementById('btn-gen-square');
        const btnGenCircle = document.getElementById('btn-gen-circle');
        const inputGenSpacing = document.getElementById('input-gen-spacing');
        
        btnGenSquare.addEventListener('click', () => {
            const spacing = parseFloat(inputGenSpacing.value) || 0.05;
            const cx = robotPos.x;
            const cy = robotPos.y;
            const side = 1.0;
            const half = side / 2;
            
            plannedPath = [];
            const corners = [
                {x: cx - half, y: cy - half},
                {x: cx + half, y: cy - half},
                {x: cx + half, y: cy + half},
                {x: cx - half, y: cy + half},
                {x: cx - half, y: cy - half}
            ];
            for (let i = 0; i < 4; i++) {
                const p1 = corners[i];
                const p2 = corners[i+1];
                const dx = p2.x - p1.x;
                const dy = p2.y - p1.y;
                const len = Math.sqrt(dx * dx + dy * dy);
                const numSteps = Math.ceil(len / spacing);
                for (let j = 0; j < numSteps; j++) {
                    const t = j / numSteps;
                    plannedPath.push({
                        x: p1.x + dx * t,
                        y: p1.y + dy * t
                    });
                }
            }
            plannedPath.push(corners[4]); // Замыкающая точка
            
            addLog(`Квадрат (1м) сгенерирован (шаг ${spacing}м, ${plannedPath.length} точек). Нажмите "Запустить" для старта.`);
            draw();
        });
        
        btnGenCircle.addEventListener('click', () => {
            const spacing = parseFloat(inputGenSpacing.value) || 0.05;
            const cx = robotPos.x;
            const cy = robotPos.y;
            const radius = 1.0;
            
            plannedPath = [];
            const circumference = 2 * Math.PI * radius;
            const numPoints = Math.ceil(circumference / spacing);
            for (let i = 0; i <= numPoints; i++) {
                const theta = (i / numPoints) * 2 * Math.PI;
                plannedPath.push({
                    x: cx + radius * Math.cos(theta),
                    y: cy + radius * Math.sin(theta)
                });
            }
            
            addLog(`Круг (R=1м) сгенерирован (шаг ${spacing}м, ${plannedPath.length} точек). Нажмите "Запустить" для старта.`);
            draw();
        });

        // Настройка параметров автопилота
        const inputLookAhead = document.getElementById('input-look-ahead');
        const inputMaxLin = document.getElementById('input-max-lin');
        const inputMaxAng = document.getElementById('input-max-ang');
        const inputKpLin = document.getElementById('input-kp-lin');
        const inputKpAng = document.getElementById('input-kp-ang');
        const inputGoalTol = document.getElementById('input-goal-tol');
        const inputDecelDist = document.getElementById('input-decel-dist');
        const inputMinLin = document.getElementById('input-min-lin');
        const inputYawDeadzone = document.getElementById('input-yaw-deadzone');
        const inputWpTol = document.getElementById('input-wp-tol');
        const inputTurnDecel = document.getElementById('input-turn-decel');
        const btnSetFollowerParams = document.getElementById('btn-set-follower-params');
        
        btnSetFollowerParams.addEventListener('click', () => {
            const lookAhead = parseFloat(inputLookAhead.value);
            const maxLin = parseFloat(inputMaxLin.value);
            const maxAng = parseFloat(inputMaxAng.value);
            const kpLin = parseFloat(inputKpLin.value);
            const kpAng = parseFloat(inputKpAng.value);
            const goalTol = parseFloat(inputGoalTol.value);
            const decelDist = parseFloat(inputDecelDist.value);
            const minLin = parseFloat(inputMinLin.value);
            const yawDeadzone = parseFloat(inputYawDeadzone.value);
            const wpTol = parseFloat(inputWpTol.value);
            const turnDecel = parseFloat(inputTurnDecel.value);
            
            fetch(`/set_follower_params?look_ahead=${lookAhead}&max_lin=${maxLin}&max_ang=${maxAng}&kp_lin=${kpLin}&kp_ang=${kpAng}&goal_tol=${goalTol}&decel_dist=${decelDist}&min_lin=${minLin}&yaw_deadzone=${yawDeadzone}&wp_tol=${wpTol}&turn_decel=${turnDecel}`)
                .then(res => res.json())
                .then(data => {
                    if (data.status === 'success') {
                        addLog(`Успех: Параметры автопилота обновлены.`);
                    } else {
                        addLog("Ошибка применения параметров автопилота.");
                    }
                })
                .catch(err => {
                    addLog("Сеть: Ошибка настройки автопилота.");
                });
        });

        // Логика джойстика и ручного вращения
        const jBase = document.getElementById('joystick-base');
        const jHandle = document.getElementById('joystick-handle');
        const btnRotCCW = document.getElementById('btn-rot-ccw');
        const btnRotCW = document.getElementById('btn-rot-cw');
        
        let jActive = false;
        const jMaxDist = 38; // px
        
        let targetVx = 0;
        let targetVy = 0;
        let targetW = 0;
        
        const updateJoystick = (clientX, clientY) => {
            const rect = jBase.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;
            
            let dx = clientX - centerX;
            let dy = clientY - centerY;
            
            const dist = Math.sqrt(dx * dx + dy * dy);
            
            if (dist > jMaxDist) {
                dx = (dx / dist) * jMaxDist;
                dy = (dy / dist) * jMaxDist;
            }
            
            jHandle.style.transform = `translate(${dx}px, ${dy}px)`;
            
            // Y-axis drag corresponds to forward/backward (dy < 0 is UP / Forward)
            // X-axis drag corresponds to strafe (dx > 0 is RIGHT / Strafe Right)
            targetVx = -(dy / jMaxDist) * 0.15; // max safe speed 0.15 m/s
            targetVy = -(dx / jMaxDist) * 0.15; // left is positive Vy (REP-103)
        };
        
        const resetJoystick = () => {
            jActive = false;
            jHandle.style.transform = 'translate(0px, 0px)';
            targetVx = 0;
            targetVy = 0;
            targetW = 0;
            sendDriveCommand();
        };
        
        jBase.addEventListener('mousedown', (e) => {
            jActive = true;
            updateJoystick(e.clientX, e.clientY);
            sendDriveCommand();
        });
        
        window.addEventListener('mousemove', (e) => {
            if (jActive) {
                updateJoystick(e.clientX, e.clientY);
            }
        });
        
        window.addEventListener('mouseup', () => {
            if (jActive) {
                resetJoystick();
            }
        });
        
        jBase.addEventListener('touchstart', (e) => {
            e.preventDefault();
            jActive = true;
            updateJoystick(e.touches[0].clientX, e.touches[0].clientY);
            sendDriveCommand();
        });
        
        window.addEventListener('touchmove', (e) => {
            if (jActive) {
                updateJoystick(e.touches[0].clientX, e.touches[0].clientY);
            }
        });
        
        window.addEventListener('touchend', () => {
            if (jActive) {
                resetJoystick();
            }
        });

        const startRotate = (dir) => {
            targetW = dir * 0.6; // rad/s
            sendDriveCommand();
        };
        
        const stopRotate = () => {
            targetW = 0.0;
            sendDriveCommand();
        };
        
        btnRotCCW.addEventListener('mousedown', () => startRotate(1.0));
        btnRotCCW.addEventListener('mouseup', stopRotate);
        btnRotCCW.addEventListener('mouseleave', stopRotate);
        
        btnRotCW.addEventListener('mousedown', () => startRotate(-1.0));
        btnRotCW.addEventListener('mouseup', stopRotate);
        btnRotCW.addEventListener('mouseleave', stopRotate);
        
        btnRotCCW.addEventListener('touchstart', (e) => { e.preventDefault(); startRotate(1.0); });
        btnRotCCW.addEventListener('touchend', stopRotate);
        
        btnRotCW.addEventListener('touchstart', (e) => { e.preventDefault(); startRotate(-1.0); });
        btnRotCW.addEventListener('touchend', stopRotate);

        // Управление с клавиатуры (WASD / QE / Пробел)
        let keyActive = false;
        window.addEventListener('keydown', (e) => {
            if (['input', 'textarea'].includes(document.activeElement.tagName.toLowerCase())) return;
            let changed = false;
            if (e.key === 'w' || e.key === 'W' || e.key === 'ArrowUp') { targetVx = 0.15; changed = true; keyActive = true; }
            else if (e.key === 's' || e.key === 'S' || e.key === 'ArrowDown') { targetVx = -0.15; changed = true; keyActive = true; }
            else if (e.key === 'd' || e.key === 'D' || e.key === 'ArrowRight') { targetVy = -0.15; changed = true; keyActive = true; }
            else if (e.key === 'a' || e.key === 'A' || e.key === 'ArrowLeft') { targetVy = 0.15; changed = true; keyActive = true; }
            else if (e.key === 'q' || e.key === 'Q') { targetW = 0.6; changed = true; keyActive = true; }
            else if (e.key === 'e' || e.key === 'E') { targetW = -0.6; changed = true; keyActive = true; }
            else if (e.key === ' ' || e.key === 'Escape') {
                targetVx = 0; targetVy = 0; targetW = 0; keyActive = false; resetJoystick(); changed = true;
            }
            if (changed) sendDriveCommand();
        });

        window.addEventListener('keyup', (e) => {
            if (['input', 'textarea'].includes(document.activeElement.tagName.toLowerCase())) return;
            let stopped = false;
            if (['w', 'W', 's', 'S', 'ArrowUp', 'ArrowDown'].includes(e.key)) { targetVx = 0; stopped = true; }
            if (['a', 'A', 'd', 'D', 'ArrowLeft', 'ArrowRight'].includes(e.key)) { targetVy = 0; stopped = true; }
            if (['q', 'Q', 'e', 'E'].includes(e.key)) { targetW = 0; stopped = true; }
            if (targetVx === 0 && targetVy === 0 && targetW === 0) keyActive = false;
            if (stopped) sendDriveCommand();
        });

        const sendDriveCommand = () => {
            fetch(`/drive?vx=${targetVx.toFixed(3)}&vy=${targetVy.toFixed(3)}&w=${targetW.toFixed(3)}`)
                .catch(err => console.error("Error sending drive command", err));
        };
        
        // Цикл отправки команд ручного управления на частоте 10 Гц
        setInterval(() => {
            if (jActive || keyActive || targetW !== 0) {
                sendDriveCommand();
            }
        }, 100);

        // Beforeunload abort safety handler
        window.addEventListener('beforeunload', () => {
            navigator.sendBeacon('/api/calibration/abort', JSON.stringify({reason: 'window_unload'}));
        });

        // Wizard confirm review handler
        const btnWizConfirm = document.getElementById('btn-wizard-confirm');
        if (btnWizConfirm) {
            btnWizConfirm.addEventListener('click', () => {
                fetch('/api/calibration/confirm', {method: 'POST'})
                    .then(res => res.json())
                    .then(data => {
                        addLog('Калибровка: ' + (data.message || 'Подтверждено'));
                        btnWizConfirm.style.display = 'none';
                        if (typeof loadTagRegistry === 'function') loadTagRegistry();
                    })
                    .catch(err => addLog('Ошибка подтверждения: ' + err));
            });
        }

        // --- Tag Registry & Calibration Wizard JS Logic ---
        let currentTagMapRevision = 0;
        let wizardActive = false;
        let physicalSettings = {wheel_diameter_mm: 70, default_marker_size_mm: 100, ceiling_height_m: 2.5};

        async function fetchPhysicalSettings() {
            try {
                const [settingsRes, extRes] = await Promise.all([fetch('/api/settings'), fetch('/api/extrinsics')]);
                if (settingsRes.ok) {
                    const data = await settingsRes.json();
                    physicalSettings = Object.assign(physicalSettings, data.settings || {});
                    document.getElementById('input-wheel-diameter').value = physicalSettings.wheel_diameter_mm;
                    document.getElementById('input-default-tag-size').value = physicalSettings.default_marker_size_mm;
                    document.getElementById('input-ceiling-height').value = physicalSettings.ceiling_height_m;
                    if (!document.getElementById('input-new-tag-size').value) document.getElementById('input-new-tag-size').value = physicalSettings.default_marker_size_mm;
                }
                if (extRes.ok) {
                    const ext = await extRes.json();
                    const badge = document.getElementById('sync-extrinsics-badge');
                    badge.textContent = String(ext.status || 'unverified').toUpperCase();
                    badge.style.color = ext.status === 'verified' ? '#2ecc71' : '#e74c3c';
                }
            } catch (e) { console.error('settings fetch failed', e); }
        }

        async function savePhysicalSettings() {
            const settings = {
                wheel_diameter_mm: parseFloat(document.getElementById('input-wheel-diameter').value),
                default_marker_size_mm: parseFloat(document.getElementById('input-default-tag-size').value),
                ceiling_height_m: parseFloat(document.getElementById('input-ceiling-height').value)
            };
            try {
                const res = await fetch('/api/settings', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({settings})});
                const data = await res.json();
                if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
                physicalSettings = data.settings;
                document.getElementById('input-new-tag-size').value = physicalSettings.default_marker_size_mm;
                addLog('Физические параметры сохранены');
            } catch (e) { addLog(`Ошибка параметров: ${e.message}`); }
        }

        async function fetchTagRegistry() {
            try {
                const res = await fetch('/api/tags');
                if (!res.ok) return;
                const data = await res.json();
                currentTagMapRevision = data.tag_map_revision || 0;

                const revEpochEl = document.getElementById('sync-rev-epoch');
                if (revEpochEl) {
                    revEpochEl.textContent = `rev ${data.tag_map_revision} | epoch ${data.config_epoch}`;
                }

                const anchorEl = document.getElementById('sync-anchor-badge');
                if (anchorEl) {
                    if (data.anchor_tag_id !== null && data.anchor_tag_id !== undefined) {
                        const statusStr = data.anchor_confirmed ? 'CONFIRMED' : 'UNCONFIRMED';
                        anchorEl.textContent = `⚓ Tag ${data.anchor_tag_id} [${statusStr}]`;
                        anchorEl.style.color = data.anchor_confirmed ? '#2ecc71' : '#ffa500';
                    } else {
                        anchorEl.textContent = '⚠️ Не назначена';
                        anchorEl.style.color = '#e74c3c';
                    }
                }

                const container = document.getElementById('tag-registry-table-container');
                if (container && data.tags) {
                    let html = '<table style="width:100%; border-collapse:collapse; text-align:left;">';
                    html += '<tr style="border-bottom:1px solid rgba(255,255,255,0.1); color:#66fcf1;"><th>ID</th><th>X</th><th>Y</th><th>Статус</th><th>Действия</th></tr>';
                    for (const [tid, info] of Object.entries(data.tags)) {
                        const p = info.pose || {x: 0, y: 0};
                        const stateColor = info.state === 'confirmed' ? '#2ecc71' : (info.state === 'provisional' ? '#f39c12' : '#888');
                        const isAnchor = (parseInt(tid) === parseInt(data.anchor_tag_id));
                        html += `<tr style="border-bottom:1px solid rgba(255,255,255,0.04);">
                            <td><b>${tid}</b> ${isAnchor ? '⚓' : ''}</td>
                            <td>${p.x.toFixed(2)}</td>
                            <td>${p.y.toFixed(2)}</td>
                            <td style="color:${stateColor}">${info.state || 'unconfirmed'}</td>
                            <td>
                                <button onclick="setAnchorTag(${tid})" style="background:none; border:none; color:#66fcf1; cursor:pointer; font-size:10px;">⚓</button>
                                <button onclick="deleteTag(${tid}, ${isAnchor})" style="background:none; border:none; color:#e74c3c; cursor:pointer; font-size:10px;">✕</button>
                            </td>
                        </tr>`;
                    }
                    html += '</table>';
                    container.innerHTML = html;
                }
            } catch (e) {
                console.error("fetchTagRegistry error:", e);
            }
        }

        async function setAnchorTag(tagId) {
            try {
                const res = await fetch('/api/anchor/confirm', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({anchor_tag_id: tagId, size_mm: physicalSettings.default_marker_size_mm, ceiling_z_m: physicalSettings.ceiling_height_m, expected_revision: currentTagMapRevision})
                });
                const data = await res.json();
                if (res.ok) {
                    addLog(`Anchor tag set to ${tagId}`);
                    fetchTagRegistry();
                } else {
                    addLog(`Error setting anchor: ${data.error}`);
                }
            } catch (e) {
                addLog(`Network error setting anchor: ${e}`);
            }
        }

        async function deleteTag(tagId, isAnchor = false) {
            const warning = isAnchor
                ? `Метка ${tagId} — текущий ноль координат. Удалить её и заблокировать навигацию до назначения нового якоря?`
                : `Полностью удалить метку ${tagId}?`;
            if (!confirm(warning)) return;
            try {
                const res = await fetch('/api/tags/delete', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({
                        tag_id: tagId,
                        expected_revision: currentTagMapRevision,
                        allow_anchor_delete: isAnchor
                    })
                });
                const data = await res.json();
                if (res.ok) {
                    addLog(`Tag ${tagId} deleted`);
                    fetchTagRegistry();
                } else {
                    addLog(`Delete error: ${data.error}`);
                }
            } catch (e) {
                addLog(`Network error deleting tag: ${e}`);
            }
        }

        async function saveNewTag() {
            const tidInput = document.getElementById('input-new-tag-id');
            const xInput = document.getElementById('input-new-tag-x');
            const yInput = document.getElementById('input-new-tag-y');
            const sizeInput = document.getElementById('input-new-tag-size');
            const tid = parseInt(tidInput.value);
            const x = parseFloat(xInput.value);
            const y = parseFloat(yInput.value);
            const sizeMm = parseFloat(sizeInput.value);
            if (isNaN(tid) || isNaN(x) || isNaN(y) || isNaN(sizeMm)) {
                alert("Укажите корректные ID, X, Y и размер метки");
                return;
            }

            const tagData = {
                enabled: true,
                state: "confirmed",
                marker_size_m: sizeMm / 1000.0,
                source: "manual",
                pose: { x: x, y: y, z: physicalSettings.ceiling_height_m, roll: 3.141592653589793, pitch: 0.0, yaw: 0.0 }
            };

            try {
                const res = await fetch('/api/tags/save', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({tag_id: tid, tag_data: tagData, expected_revision: currentTagMapRevision})
                });
                const data = await res.json();
                if (res.ok) {
                    addLog(`Tag ${tid} saved successfully (rev ${data.revision})`);
                    fetchTagRegistry();
                } else {
                    addLog(`Save tag error (${res.status}): ${data.error}`);
                }
            } catch (e) {
                addLog(`Network error saving tag: ${e}`);
            }
        }

        async function startWizard() {
            const tid = parseInt(document.getElementById('wizard-target-tag').value);
            const markerSizeM = parseFloat(document.getElementById('input-new-tag-size').value || physicalSettings.default_marker_size_mm) / 1000.0;
            if (isNaN(tid)) return;
            try {
                const res = await fetch('/api/calibration/start', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({tag_id: tid, marker_size_m: markerSizeM})
                });
                const data = await res.json();
                if (res.ok) {
                    wizardActive = true;
                    addLog(`Wizard started for tag ${tid}`);
                } else {
                    addLog(`Failed to start wizard: ${data.error}`);
                }
            } catch (e) {
                addLog(`Network error starting wizard: ${e}`);
            }
        }

        async function abortWizard() {
            try {
                await fetch('/api/calibration/abort', {method: 'POST'});
                wizardActive = false;
                addLog("Wizard aborted");
            } catch (e) {}
        }

        async function pollWizardStatus() {
            try {
                const res = await fetch('/api/calibration/status');
                if (!res.ok) return;
                const data = await res.json();
                const badge = document.getElementById('wizard-status-badge');
                const diag = document.getElementById('wizard-diagnostics');
                const stoppedByUser = data.state === 'ABORTED' &&
                    (data.abort_reason || '').toLowerCase().includes('user requested abort');
                if (badge) {
                    badge.textContent = stoppedByUser ? 'ОСТАНОВЛЕНО' : (data.state || 'IDLE');
                    badge.style.color = (data.state === 'COMPLETED') ? '#2ecc71' :
                        ((data.state === 'ABORTED' && !stoppedByUser) ? '#e74c3c' : '#66fcf1');
                }
                if (btnWizConfirm) btnWizConfirm.style.display = data.state === 'REVIEW' ? 'block' : 'none';
                if (diag) {
                    if (data.state === 'FINE_CENTERING') {
                        const axis = data.centering_axis === 'horizontal' ? 'по горизонтали' : 'по вертикали';
                        const err = Number.isFinite(data.centering_axis_error_px) ? `, ошибка ${data.centering_axis_error_px.toFixed(1)} px` : '';
                        diag.textContent = `Центрирование ${axis}${err} (${data.elapsed_s}s)`;
                    } else if (data.state === 'STATIONARY_SOLVE') {
                        diag.textContent = `Сбор статических кадров: ${data.samples_count}/30`;
                    } else if (data.state === 'COMPLETED') {
                        diag.textContent = `Успешно обучена метка ${data.target_tag_id}!`;
                        fetchTagRegistry();
                    } else if (data.state === 'ABORTED') {
                        diag.textContent = stoppedByUser
                            ? 'Остановлено пользователем'
                            : `Ошибка: ${data.abort_reason || 'Отмена'}`;
                    } else {
                        diag.textContent = `Режим: ${data.state}`;
                    }
                }

                // Send heartbeat while actively running
                if (data.state && !['IDLE', 'COMPLETED', 'ABORTED'].includes(data.state)) {
                    wizardActive = true;
                    fetch('/api/calibration/heartbeat', {method: 'POST'}).catch(() => {});
                } else {
                    wizardActive = false;
                }
            } catch (e) {}
        }

        // Attach listeners
        const btnRefresh = document.getElementById('btn-refresh-tags');
        if (btnRefresh) btnRefresh.addEventListener('click', fetchTagRegistry);

        const btnSave = document.getElementById('btn-add-tag-save');
        if (btnSave) btnSave.addEventListener('click', saveNewTag);

        const btnWizStart = document.getElementById('btn-wizard-start');
        if (btnWizStart) btnWizStart.addEventListener('click', startWizard);

        const btnWizAbort = document.getElementById('btn-wizard-abort');
        if (btnWizAbort) btnWizAbort.addEventListener('click', abortWizard);
        const btnSavePhysical = document.getElementById('btn-save-physical');
        if (btnSavePhysical) btnSavePhysical.addEventListener('click', savePhysicalSettings);

        fetchTagRegistry();
        fetchPhysicalSettings();
        setInterval(fetchTagRegistry, 2000);
        setInterval(pollWizardStatus, 400);

        // Старт
        resizeCanvas();
        fetchConfig();
        connectSSE();
    </script>
</body>
</html>
"""

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("KeyboardInterrupt received. Saving trajectory plot...")
        node.save_trajectory_and_shutdown()
    finally:
        try:
            node.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
