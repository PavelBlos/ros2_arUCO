import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from fake_tag_interfaces.msg import TagDetectionArray
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String, Empty
import tf2_ros
from ament_index_python.packages import get_package_share_directory
import os
import time
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
        self.load_tags_config()
        
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

        # Подписка на колесную одометрию
        self.odom_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.odom_callback, 10)

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

        # Параметры автопилота Pure Pursuit (плавный ход без зависаний)
        self.ap_look_ahead = 0.15
        self.ap_max_lin = 0.14
        self.ap_max_ang = 0.70
        self.ap_kp_lin = 0.80
        self.ap_kp_ang = 1.50
        self.ap_goal_tol = 0.04
        self.ap_decel_dist = 0.30
        self.ap_min_lin = 0.03
        self.ap_yaw_deadzone = 0.05
        self.ap_wp_tol = 0.08
        self.ap_turn_decel = 0.20

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
        self.git_commit = "5729a2e"
        
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

            with open(config_path, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)
                all_tags = config_data.get('tags', {})
                self.tags_db = {k: v for k, v in all_tags.items() if v.get('enabled', True)}
                self.get_logger().info(f"Loaded {len(self.tags_db)} enabled tags from {config_path} (out of {len(all_tags)} total).")
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {str(e)}")

    def image_callback(self, msg):
        with self.latest_frame_lock:
            self.latest_jpeg_frame = bytes(msg.data)

    def tag_callback(self, msg):
        try:
            # Проверяем сигнал окончания
            if msg.header.frame_id == "finished":
                self.get_logger().info("Received finished signal. Saving trajectory plot and shutting down...")
                self.save_trajectory_and_shutdown()
                return

            detections = msg.detections
            if not detections:
                # Метки не обнаружены в текущем кадре
                return

            translations = []
            rotations = []

            # 1. Получаем смещение камеры относительно базы робота base_link -> camera_link из TF
            try:
                t = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
                T_base_camera = np.eye(4)
                T_base_camera[:3, :3] = R.from_quat([
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                ]).as_matrix()
                T_base_camera[:3, 3] = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z
                ]
            except Exception as tf_err:
                # Если TF еще не опубликован, считаем, что они совпадают
                self.get_logger().debug(f"TF lookup base_link->camera_link failed, using identity/hardcoded pitch: {str(tf_err)}")
                # По умолчанию: камера на роботе смотрит вверх (pitch = -90 градусов, yaw = +90 градусов)
                camera_rot = R.from_euler('xyz', [0.0, -np.pi / 2.0, np.pi / 2.0])
                T_base_camera = np.eye(4)
                T_base_camera[:3, :3] = camera_rot.as_matrix()
                T_base_camera[:3, 3] = [0.0, 0.0, 0.0]

            # 2. Обрабатываем каждую метку из массива обнаруженных
            for detection in detections:
                tag_id = detection.tag_id
                tag_key = f"tag_{tag_id}"
                
                if tag_key not in self.tags_db:
                    self.get_logger().warn(f"Detected unknown tag with ID: {tag_id}")
                    continue
                
                tag_info = self.tags_db[tag_key]

                # Поза метки на потолке T_map_tag (высота 2.5м, разворот вниз)
                T_map_tag = np.eye(4)
                T_map_tag[:3, :3] = R.from_euler('xyz', [tag_info['roll'], tag_info['pitch'], tag_info['yaw']]).as_matrix()
                T_map_tag[:3, 3] = [tag_info['x'], tag_info['y'], tag_info['z']]

                # Поза метки относительно камеры T_camera_tag
                T_camera_tag = np.eye(4)
                rel_rot = R.from_quat([
                    detection.pose.orientation.x,
                    detection.pose.orientation.y,
                    detection.pose.orientation.z,
                    detection.pose.orientation.w
                ])
                T_camera_tag[:3, :3] = rel_rot.as_matrix()
                T_camera_tag[:3, 3] = [detection.pose.position.x, detection.pose.position.y, detection.pose.position.z]

                # Поза камеры на карте: T_map_camera = T_map_tag * (T_camera_tag)^-1
                T_map_camera = T_map_tag @ np.linalg.inv(T_camera_tag)

                # Поза робота на карте T_map_base = T_map_camera * (T_base_camera)^-1
                T_map_base = T_map_camera @ np.linalg.inv(T_base_camera)

                # Извлекаем смещение и кватернион
                robot_pos = T_map_base[:3, 3]
                robot_rot = R.from_matrix(T_map_base[:3, :3]).as_quat()

                translations.append(robot_pos)
                rotations.append(robot_rot)

            if not translations:
                return

            # 3. Усреднение (слияние) данных локализации от нескольких меток
            if len(translations) == 1:
                # Если обнаружена только одна метка, берем ее позу напрямую
                avg_pos = translations[0]
                avg_rot = rotations[0]
            else:
                # Если несколько меток:
                # 3.1. Усредняем линейные координаты (X, Y, Z) - среднее арифметическое
                avg_pos = np.mean(translations, axis=0)
                # 3.2. Усредняем вращения с помощью Rotation.mean()
                try:
                    avg_rot = R.from_quat(rotations).mean().as_quat()
                except Exception as rot_mean_err:
                    self.get_logger().error(f"Rotation averaging failed: {str(rot_mean_err)}")
                    avg_rot = rotations[0]

            # 4. Применяем слияние датчиков (Комплементарный фильтр + Outlier Rejection)
            avg_yaw = self.quaternion_to_yaw_from_quat(avg_rot)
            
            if not self.fused_initialized:
                self.fused_x = avg_pos[0]
                self.fused_y = avg_pos[1]
                self.fused_z = avg_pos[2]
                self.fused_yaw = avg_yaw
                self.fused_initialized = True
                self.outlier_count = 0
            else:
                # Проверка на пространственный выброс (Outlier Gating)
                spatial_jump = np.sqrt((avg_pos[0] - self.fused_x)**2 + (avg_pos[1] - self.fused_y)**2)
                # Если скачок больше 0.40 м за один кадр:
                if spatial_jump > 0.40:
                    self.outlier_count += 1
                    if self.outlier_count < 4:
                        self.get_logger().warn(
                            f"⚠️ Игнорируем выброс ArUco: скачок {spatial_jump:.2f}м (порог 0.40м). Выбросов подряд: {self.outlier_count}"
                        )
                        return
                    else:
                        # Если 4 кадра подряд фиксируют новую позицию (робота переставили руками)
                        self.get_logger().info(f"🔄 Смена позиции робота подтверждена (4 кадра): X={avg_pos[0]:.2f}, Y={avg_pos[1]:.2f}")
                        self.fused_x = avg_pos[0]
                        self.fused_y = avg_pos[1]
                        self.fused_yaw = avg_yaw
                        self.outlier_count = 0
                else:
                    self.outlier_count = 0

                # Коэффициент доверия к визуальной метке (filter_alpha)
                K = self.filter_alpha
                self.fused_x += K * (avg_pos[0] - self.fused_x)
                self.fused_y += K * (avg_pos[1] - self.fused_y)
                self.fused_z += K * (avg_pos[2] - self.fused_z)
                
                yaw_diff = avg_yaw - self.fused_yaw
                yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
                self.fused_yaw += K * yaw_diff
                self.fused_yaw = np.arctan2(np.sin(self.fused_yaw), np.cos(self.fused_yaw))

            self.tracking_mode = "aruco_fused"
            self.last_valid_tag_time = time.time()

            # Обновляем длину сырого пути
            if len(self.raw_trajectory_x) > 0:
                dx = avg_pos[0] - self.raw_trajectory_x[-1]
                dy = avg_pos[1] - self.raw_trajectory_y[-1]
                dz = avg_pos[2] - self.raw_trajectory_z[-1]
                self.raw_path_length += np.sqrt(dx*dx + dy*dy + dz*dz)

            self.raw_trajectory_x.append(avg_pos[0])
            self.raw_trajectory_y.append(avg_pos[1])
            self.raw_trajectory_z.append(avg_pos[2])

            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.trajectory_timestamps.append(stamp_sec)

            # Вызываем публикацию отфильтрованной позы и TF
            self.last_detected_tags = [d.tag_id for d in detections]
            self.publish_fused_pose(msg.header.stamp)

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
                wheel_radius=0.030,
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
        """Коллбэк прямой одометрии шаговых двигателей от TermitRobotAPI (20-50 Гц)"""
        self.last_esp32_odom_time = time.time()
        now = self.get_clock().now()
        dt = 0.04
        
        if not self.fused_initialized:
            self.fused_x = odom.x
            self.fused_y = odom.y
            self.fused_yaw = odom.theta
            self.fused_initialized = True
            return

        # В termit_api:
        # odom.vy = продольная скорость тела робота (вперед > 0, назад < 0)
        # odom.vx = боковая скорость тела робота (вправо > 0, влево < 0)
        # В СК робота base_link (REP 103):
        # X_base = вперед = odom.vy
        # Y_base = влево = -odom.vx
        v_forward = odom.vy
        v_left = -odom.vx
        w = odom.omega

        self.fused_yaw += w * dt
        self.fused_yaw = np.arctan2(np.sin(self.fused_yaw), np.cos(self.fused_yaw))

        # Точное преобразование движения из ПСК робота в СК карты
        dx_global = (v_forward * np.cos(self.fused_yaw) - v_left * np.sin(self.fused_yaw)) * dt
        dy_global = (v_forward * np.sin(self.fused_yaw) + v_left * np.cos(self.fused_yaw)) * dt

        self.fused_x += dx_global
        self.fused_y += dy_global

        if time.time() - self.last_valid_tag_time > 0.6:
            self.tracking_mode = "dead_reckoning"

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

    def drive_robot(self, forward, strafe, w):
        """Прямое аппаратное управление моторами через ESP32 API + публикация Twist в /cmd_vel"""
        is_motion = (abs(forward) > 0.001 or abs(strafe) > 0.001 or abs(w) > 0.001)

        # Автоматическое включение питания обмоток и обновление таймера простоя при команде движения
        if is_motion:
            self.last_motion_cmd_time = time.time()
            if self.motor_power_state == "disabled":
                self.set_motor_power("enable")

        # 1. Прямая отправка в ESP32
        if self.robot and self.robot.is_connected:
            try:
                if not is_motion:
                    self.robot.stop()
                else:
                    # В termit_api: vx = боковой стрейф (вправо > 0), vy = продольный ход (вперед > 0), omega = разворот
                    self.robot.drive(vx=float(strafe), vy=float(forward), omega=float(w))
            except Exception as e:
                self.get_logger().error(f"ESP32 motor drive error: {str(e)}")

        # 2. Публикация в ROS 2 топик /cmd_vel для совместимости
        msg = Twist()
        msg.linear.x = float(forward)
        msg.linear.y = float(strafe)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def set_path_plan(self, waypoints):
        """Сохранение путевых точек маршрута и публикация Path в ROS"""
        self.route_waypoints = list(waypoints)
        self.current_wp_idx = 0
        self.route_state = "idle"
        self.publish_plan(waypoints)
        self.notify_ui_event()
        self.get_logger().info(f"Загружен новый маршрут из {len(waypoints)} точек")

    def start_route(self):
        """Запуск автономного движения по маршруту (Pure Pursuit)"""
        if not self.route_waypoints:
            self.get_logger().warn("Невозможно запустить маршрут: список точек пуст!")
            return False
            
        self.set_motor_power("enable")
        self.route_state = "running"
        self.autopilot_active = True
        
        if self.autopilot_thread is None or not self.autopilot_thread.is_alive():
            self.autopilot_thread = threading.Thread(target=self.autopilot_loop, daemon=True, name="Autopilot")
            self.autopilot_thread.start()
            
        self.notify_ui_event()
        self.get_logger().info(f"▶ Старт автопилота с точки {self.current_wp_idx + 1}/{len(self.route_waypoints)}")
        return True

    def pause_route(self):
        """Пауза / Снятие с паузы автопилота"""
        if self.route_state == "running":
            self.route_state = "paused"
            self.autopilot_active = False
            self.drive_robot(0.0, 0.0, 0.0)
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
        self.drive_robot(0.0, 0.0, 0.0)
        # Allow the ESP32 braking ramp to finish before the 2 s power timer.
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
        Высокоточный цикл автономного движения Lookahead Pure Pursuit (20 Гц).
        Прямое управление через ESP32 API с плавным круиз-контролем.
        Исключает рывки, зависания на промежуточных точках и разворот векторов назад.
        """
        import time as pytime
        smooth_forward = 0.0
        smooth_strafe = 0.0
        smooth_w = 0.0

        while self.autopilot_active and self.route_state == "running":
            t_loop_start = pytime.time()
            self.last_motion_cmd_time = t_loop_start
            
            if not self.route_waypoints or self.current_wp_idx >= len(self.route_waypoints):
                self.route_state = "finished"
                self.autopilot_active = False
                self.drive_robot(0.0, 0.0, 0.0)
                self.notify_ui_event()
                self.get_logger().info("🎉 Маршрут полностью выполнен! Робот на финише.")
                # Power watchdog releases coils after braking, not mid-ramp.
                self.last_motion_cmd_time = pytime.time()
                break

            rx = float(self.fused_x)
            ry = float(self.fused_y)
            ryaw = float(self.fused_yaw)

            fx, fy = self.route_waypoints[-1]
            dist_to_finish = float(np.hypot(fx - rx, fy - ry))

            # Проверка достижения финальной цели
            if dist_to_finish < self.ap_goal_tol and self.current_wp_idx >= len(self.route_waypoints) - 2:
                self.route_state = "finished"
                self.autopilot_active = False
                self.drive_robot(0.0, 0.0, 0.0)
                self.notify_ui_event()
                self.get_logger().info(f"🎉 Финиш достигнут (дистанция {dist_to_finish*100:.1f} см)!")
                # Power watchdog releases coils after braking, not mid-ramp.
                self.last_motion_cmd_time = pytime.time()
                break

            # 1. Продвижение по точкам вперед (Dynamic Waypoint Advancement)
            # Робот никогда не зависает на пройденной точке: переключается, если ближе к следующей
            # или если проекция позиции находится впереди текущего сегмента
            while self.current_wp_idx < len(self.route_waypoints) - 1:
                c_wp = self.route_waypoints[self.current_wp_idx]
                n_wp = self.route_waypoints[self.current_wp_idx + 1]
                d_c = np.hypot(c_wp[0] - rx, c_wp[1] - ry)
                d_n = np.hypot(n_wp[0] - rx, n_wp[1] - ry)
                s_dx = n_wp[0] - c_wp[0]
                s_dy = n_wp[1] - c_wp[1]
                s_len_sq = s_dx**2 + s_dy**2

                if d_c < max(0.10, self.ap_wp_tol) or d_n < d_c:
                    self.current_wp_idx += 1
                    self.notify_ui_event()
                    self.get_logger().info(f"📍 Пройдена точка! Следующая: {self.current_wp_idx + 1}/{len(self.route_waypoints)}")
                elif s_len_sq > 1e-6:
                    proj = ((rx - c_wp[0]) * s_dx + (ry - c_wp[1]) * s_dy) / s_len_sq
                    if proj > 0.75:
                        self.current_wp_idx += 1
                        self.notify_ui_event()
                        self.get_logger().info(f"📍 Пройдена точка! Следующая: {self.current_wp_idx + 1}/{len(self.route_waypoints)}")
                    else:
                        break
                else:
                    break

            # 2. Расчет точки упреждения (Lookahead Carrot Point) вдоль маршрута
            # Carrot point всегда находится на расстоянии lookahead вперед по пути
            lookahead_dist = max(0.14, self.ap_look_ahead)
            accum_dist = 0.0
            carrot_pt = self.route_waypoints[self.current_wp_idx]
            
            for idx in range(self.current_wp_idx, len(self.route_waypoints) - 1):
                p_a = self.route_waypoints[idx]
                p_b = self.route_waypoints[idx + 1]
                seg_len = np.hypot(p_b[0] - p_a[0], p_b[1] - p_a[1])
                if accum_dist + seg_len >= lookahead_dist:
                    rem = lookahead_dist - accum_dist
                    t_seg = rem / max(0.001, seg_len)
                    carrot_pt = [p_a[0] + t_seg * (p_b[0] - p_a[0]), p_a[1] + t_seg * (p_b[1] - p_a[1])]
                    break
                accum_dist += seg_len
                carrot_pt = p_b

            tx, ty = carrot_pt
            dx = tx - rx
            dy = ty - ry
            dist_to_target = np.hypot(dx, dy)
            if dist_to_target < 0.001:
                dist_to_target = 0.001

            # 3. Скорость: постоянная крейсерская по трассе, плавное торможение только перед финишем
            if dist_to_finish > self.ap_decel_dist:
                linear_speed = self.ap_max_lin
            else:
                ratio = (dist_to_finish - self.ap_goal_tol) / max(0.01, self.ap_decel_dist - self.ap_goal_tol)
                ratio = np.clip(ratio, 0.0, 1.0)
                linear_speed = self.ap_min_lin + ratio * (self.ap_max_lin - self.ap_min_lin)

            # 4. Кинематика Omni: проекция вектора скорости на систему координат робота
            # v_forward: проекция на продольную ось робота (вперед)
            # v_left:    проекция на поперечную ось робота (влево)
            v_forward = linear_speed * (dx * np.cos(ryaw) + dy * np.sin(ryaw)) / dist_to_target
            v_left    = linear_speed * (-dx * np.sin(ryaw) + dy * np.cos(ryaw)) / dist_to_target

            # 5. Мягкая угловая ориентация (ограничена 0.35 рад/с для стабильности ArUco)
            target_yaw = np.arctan2(dy, dx)
            yaw_err = target_yaw - ryaw
            yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))
            w = np.clip(0.8 * yaw_err, -0.35, 0.35)

            strafe_right = -v_left

            # EMA-фильтр векторов скорости (исключает ступенчатые рывки между путевыми точками)
            alpha = 0.35
            smooth_forward = smooth_forward * (1.0 - alpha) + v_forward * alpha
            smooth_strafe  = smooth_strafe  * (1.0 - alpha) + strafe_right * alpha
            smooth_w       = smooth_w       * (1.0 - alpha) + w * alpha

            self.drive_robot(smooth_forward, smooth_strafe, smooth_w)

            rec = {
                "t": t_loop_start,
                "run_id": self.active_run_id,
                "wp_idx": int(self.current_wp_idx),
                "rx": round(rx, 4),
                "ry": round(ry, 4),
                "ryaw": round(ryaw, 4),
                "tx": round(tx, 4),
                "ty": round(ty, 4),
                "dist_finish": round(dist_to_finish, 4),
                "cmd_fwd": round(smooth_forward, 4),
                "cmd_strafe": round(smooth_strafe, 4),
                "cmd_w": round(smooth_w, 4),
                "mode": self.tracking_mode
            }
            self.log_record(rec)

            elapsed = pytime.time() - t_loop_start
            pytime.sleep(max(0.01, 0.05 - elapsed))
            
        self.drive_robot(0.0, 0.0, 0.0)

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
            self.drive_robot(0.0, 0.0, 0.0)
            self.set_motor_power("disable")
            return True
            
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
            self.drive_robot(forward, strafe, w)
            pytime.sleep(0.05)
            
        self.drive_robot(0.0, 0.0, 0.0)
        self.test_drive_active = False
        self.get_logger().info("Test motion finished, robot stopped.")
        pytime.sleep(0.3)
        self.set_motor_power("disable")

    def odom_callback(self, msg):
        """Интеграция одометрии шаговых двигателей для экстраполяции позы"""
        now = self.get_clock().now()
        if self.last_odom_msg_time is None:
            self.last_odom_msg_time = now
            return
            
        dt = (now - self.last_odom_msg_time).nanoseconds / 1e9
        self.last_odom_msg_time = now
        
        if dt <= 0 or dt > 0.5:
            dt = 0.04
            
        if not self.fused_initialized:
            self.fused_x = msg.pose.pose.position.x
            self.fused_y = msg.pose.pose.position.y
            self.fused_z = msg.pose.pose.position.z
            
            q = msg.pose.pose.orientation
            q_arr = [q.x, q.y, q.z, q.w]
            self.fused_yaw = self.quaternion_to_yaw_from_quat(q_arr)
            self.fused_initialized = True
            return

        vx_local = msg.twist.twist.linear.x
        vy_local = msg.twist.twist.linear.y
        w = msg.twist.twist.angular.z

        self.fused_yaw += w * dt
        self.fused_yaw = np.arctan2(np.sin(self.fused_yaw), np.cos(self.fused_yaw))

        dx_local = vx_local * dt
        dy_local = vy_local * dt

        dx_global = dx_local * np.cos(self.fused_yaw) - dy_local * np.sin(self.fused_yaw)
        dy_global = dx_local * np.sin(self.fused_yaw) + dy_local * np.cos(self.fused_yaw)

        self.fused_x += dx_global
        self.fused_y += dy_global

        if time.time() - self.last_valid_tag_time > 0.6:
            self.tracking_mode = "dead_reckoning"

        self.publish_fused_pose(now.to_msg())

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
            
            self.server.node.drive_robot(vx, vy, w)
            
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

        elif self.path.startswith('/api/health'):
            now_t = time.time()
            connected = bool(self.server.node.robot and self.server.node.robot.is_connected)
            esp_odom_fresh = (now_t - self.server.node.last_esp32_odom_time < 0.5) if self.server.node.last_esp32_odom_time > 0 else False
            pose_fresh = (now_t - self.server.node.last_pose_publish_time < 0.5) if self.server.node.last_pose_publish_time > 0 else False
            tag_fresh = (now_t - self.server.node.last_valid_tag_time < 1.0) if self.server.node.last_valid_tag_time > 0 else False
            
            health_data = {
                "status": "ok" if connected else "degraded",
                "git_commit": getattr(self.server.node, "git_commit", "5729a2e"),
                "esp32_connected": connected,
                "esp32_port": self.server.node.robot._port_name if self.server.node.robot else None,
                "esp32_odom_fresh": esp_odom_fresh,
                "esp32_odom_age_s": round(now_t - self.server.node.last_esp32_odom_time, 3) if self.server.node.last_esp32_odom_time > 0 else None,
                "pose_fresh": pose_fresh,
                "pose_age_s": round(now_t - self.server.node.last_pose_publish_time, 3) if self.server.node.last_pose_publish_time > 0 else None,
                "tag_fresh": tag_fresh,
                "tag_age_s": round(now_t - self.server.node.last_valid_tag_time, 3) if self.server.node.last_valid_tag_time > 0 else None,
                "active_run_id": self.server.node.active_run_id,
                "motor_power": self.server.node.motor_power_state,
                "tracking_mode": self.server.node.tracking_mode,
                "active_tags_count": len(self.server.node.tags_db)
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(health_data).encode('utf-8'))

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
        <button class="btn" id="btn-test-stop" style="background-color: #ff4d4d; color: white; box-shadow: 0 0 10px rgba(255, 77, 77, 0.3); border-color: #ff4d4d; margin-bottom: 12px; font-size: 13px;">ЭКСТРЕННЫЙ СТОП</button>

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
            targetVy = (dx / jMaxDist) * 0.15;  // right is positive Vy
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
            else if (e.key === 'd' || e.key === 'D' || e.key === 'ArrowRight') { targetVy = 0.15; changed = true; keyActive = true; }
            else if (e.key === 'a' || e.key === 'A' || e.key === 'ArrowLeft') { targetVy = -0.15; changed = true; keyActive = true; }
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
