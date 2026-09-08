import rclpy
from rclpy.node import Node
from fake_tag_interfaces.msg import TagDetection, TagDetectionArray
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import os
import yaml
import time
import threading
import subprocess
from scipy.spatial.transform import Rotation as R

class VideoTagDetector(Node):
    def __init__(self):
        super().__init__('video_tag_detector')
        
        # Декларируем ROS2 параметры с поддержкой динамического типа
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('video_path', 'config/robot_drive.mp4', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('calibration_path', 'config/camera_info.yaml', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('marker_length', 0.15, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('detection_rate', 30.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_100', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('loop_video', True, ParameterDescriptor(dynamic_typing=True))

        video_param = self.get_parameter('video_path')
        self.video_path = str(video_param.value) if video_param.value is not None else 'config/robot_drive.mp4'

        calib_param = self.get_parameter('calibration_path')
        self.calibration_path = str(calib_param.value) if calib_param.value is not None else 'config/camera_info.yaml'

        marker_param = self.get_parameter('marker_length')
        try:
            self.marker_length = float(marker_param.value)
        except (TypeError, ValueError):
            self.marker_length = 0.15

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

        # Разрешаем относительный путь к видеофайлу через share-директорию пакета
        if not os.path.isabs(self.video_path):
            try:
                share_dir = get_package_share_directory('fake_tag_publisher')
                full_video_path = os.path.join(share_dir, self.video_path)
                if os.path.exists(full_video_path):
                    self.video_path = full_video_path
            except Exception as e:
                self.get_logger().warn(f"Could not resolve video path in share directory: {str(e)}")

        self.get_logger().info('Video Tag Detector node starting...')
        self.get_logger().info(f"Params: video={self.video_path}, calib={self.calibration_path}, marker_size={self.marker_length}m, loop={self.loop_video}")

        # 1. Загрузка параметров калибровки камеры
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration()

        # 2. Инициализация OpenCV детектора ArUco (словарь DICT_4X4_100)
        self.init_aruco_detector()

        # 3. Открываем видеофайл или камеру с автоматическим fallback
        self.cap = None
        self.open_source()

        # 4. Создаем публикататор и таймер обработки кадров
        self.publisher_ = self.create_publisher(TagDetectionArray, '/fake_tag', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/camera/annotated_image/compressed', 10)
        
        timer_period = 1.0 / self.detection_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def open_source(self):
        import re
        match_dev = re.match(r'^/dev/video(\d+)$', str(self.video_path))
        is_cam = str(self.video_path).isdigit() or match_dev is not None or self.video_path in ['/dev/video0', '0']
        
        opened = False
        
        if is_cam:
            cam_idx = int(self.video_path) if str(self.video_path).isdigit() else (int(match_dev.group(1)) if match_dev else 0)
            self.get_logger().info(f"🎥 Открытие физической камеры (индекс {cam_idx})...")
            
            # Стандартный захват через OpenCV V4L2 backend (работает совместно с libcamerify)
            self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(cam_idx)
                
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # Прогрев камеры для сходимости аппаратной автоэкспозиции (AEC / AGC)
                self.get_logger().info("⏳ Прогрев камеры и автоэкспозиции...")
                test_f = None
                for _ in range(12):
                    ret, f = self.cap.read()
                    if ret and f is not None and f.size > 0:
                        test_f = f
                        opened = True
                    time.sleep(0.04)
                    
                if opened and test_f is not None:
                    self.is_live = True
                    self.get_logger().info(f"✅ Физическая камера успешно открыта! Размер кадра: {test_f.shape}")
                else:
                    self.cap.release()
                    self.cap = None
                    opened = False
        else:
            self.cap = cv2.VideoCapture(self.video_path)
            if self.cap.isOpened():
                ret, test_f = self.cap.read()
                if ret and test_f is not None:
                    opened = True
                    self.is_live = False
                    self.get_logger().info(f"Successfully opened video file: {self.video_path}")

        if not opened:
            self.get_logger().warn(f"⚠️ Камера '{self.video_path}' недоступна. Переключаемся на симуляцию (config/robot_drive.mp4)...")
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
            if self.cap.isOpened():
                self.get_logger().info(f"✅ Запущен видеопоток из файла: {fb_path}")
            else:
                self.get_logger().error(f"❌ Не удалось открыть видеофайл: {fb_path}")

    def load_calibration(self):
        try:
            # Если путь не абсолютный, ищем в share пакета
            if not os.path.isabs(self.calibration_path):
                share_dir = get_package_share_directory('fake_tag_publisher')
                full_path = os.path.join(share_dir, self.calibration_path)
                if not os.path.exists(full_path):
                    # Пробуем как относительный путь от запуска
                    full_path = self.calibration_path
            else:
                full_path = self.calibration_path

            self.get_logger().info(f"Loading camera calibration from: {full_path}")
            with open(full_path, 'r') as f:
                calib_data = yaml.safe_load(f)
                self.camera_matrix = np.array(calib_data['camera_matrix']).reshape(3, 3)
                self.dist_coeffs = np.array(calib_data['distortion_coefficients'])
                
                self.get_logger().info("Successfully loaded camera calibration matrix and distortion coefficients.")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration file: {str(e)}")
            # Фолбэк на дефолтные идеальные значения
            self.camera_matrix = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
            self.dist_coeffs = np.zeros(5)

    def init_aruco_detector(self):
        # Преобразуем имя словаря из строки в OpenCV константу
        dict_id = getattr(cv2.aruco, self.aruco_dict_name, cv2.aruco.DICT_4X4_50)
        
        # Для OpenCV < 4.7 (например, Ubuntu 24.04 OpenCV 4.6.0) используем *_create
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.dictionary = cv2.aruco.Dictionary_get(dict_id)
            self.parameters = cv2.aruco.DetectorParameters_create()
            if hasattr(self.parameters, 'minMarkerPerimeterRate'):
                self.parameters.minMarkerPerimeterRate = 0.05
            if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detect_func = lambda img: cv2.aruco.detectMarkers(img, self.dictionary, parameters=self.parameters)
            self.get_logger().info(f"Initialized OpenCV 4.6 Legacy ArUco detector ({self.aruco_dict_name})")
        else:
            self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            self.parameters = cv2.aruco.DetectorParameters()
            if hasattr(self.parameters, 'minMarkerPerimeterRate'):
                self.parameters.minMarkerPerimeterRate = 0.05
            if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            self.detect_func = lambda img: self.detector.detectMarkers(img)
            self.get_logger().info(f"Initialized OpenCV 4.7+ ArUco detector ({self.aruco_dict_name})")

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        # Считываем следующий кадр
        ret, frame = self.cap.read()
        
        # Если кадр не считался (видео закончилось или сбой камеры)
        if not ret or frame is None:
            if not getattr(self, 'is_live', False):
                if self.loop_video:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                
                if not ret or frame is None:
                    if not self.loop_video:
                        self.get_logger().info("Video finished. Sending finished signal and stopping timer.")
                        array_msg = TagDetectionArray()
                        array_msg.header.stamp = self.get_clock().now().to_msg()
                        array_msg.header.frame_id = "finished"
                        self.publisher_.publish(array_msg)
                        self.timer.cancel()
                    return
            else:
                return

        # 1. Перевод в оттенки серого напрямую (без ресурсоемкого cv2.undistort для всего кадра)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. Детектируем маркеры ArUco
        corners, ids, rejected = self.detect_func(gray)

        # Фильтруем ложные мелкие шумы и дедуплицируем по tag_id
        valid_corners = []
        valid_ids = []
        if ids is not None:
            best_by_id = {}
            for i in range(len(ids)):
                tag_id = int(ids[i][0])
                pts = corners[i][0]
                perim = cv2.arcLength(pts, True)
                if perim >= 45.0:
                    if tag_id not in best_by_id or perim > best_by_id[tag_id][1]:
                        best_by_id[tag_id] = (corners[i], perim)
            for tid, (c, _) in best_by_id.items():
                valid_corners.append(c)
                valid_ids.append([tid])

        if valid_ids:
            corners = tuple(valid_corners)
            ids = np.array(valid_ids)
        else:
            corners = ()
            ids = None

        # Создаем ROS2 сообщение массива детекций
        array_msg = TagDetectionArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "camera_link"

        detected_ids = []

        if ids is not None:
            # Координаты вершин идеального маркера в его собственной 3D-системе координат
            # Центр маркера в (0, 0, 0), размер маркера - marker_length
            half_len = self.marker_length / 2.0
            obj_pts = np.array([
                [-half_len,  half_len, 0.0],
                [ half_len,  half_len, 0.0],
                [ half_len, -half_len, 0.0],
                [-half_len, -half_len, 0.0]
            ], dtype=np.float32)

            for i in range(len(ids)):
                tag_id = int(ids[i][0])
                detected_ids.append(tag_id)

                # 3. Вычисляем 3D-позу маркера в OpenCV-камере с помощью SolvePnP
                # corners[i][0] имеет размер (4, 2) и хранит 2D-координаты вершин
                ret_pnp, rvec, tvec = cv2.solvePnP(obj_pts, corners[i][0], self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                
                if not ret_pnp:
                    continue

                # Преобразуем вектор вращения в матрицу поворота
                R_mat, _ = cv2.Rodrigues(rvec)

                # Составляем матрицу T_cameraOpt_tag (OpenCV камера -> Маркер)
                T_cameraOpt_tag = np.eye(4)
                T_cameraOpt_tag[:3, :3] = R_mat
                T_cameraOpt_tag[:3, 3] = tvec.ravel()

                # 4. Преобразование систем координат (OpenCV Optical -> ROS REP 103 standard)
                # В OpenCV Optical: Z вперед, X вправо, Y вниз.
                # В ROS camera_link: X вперед, Y влево, Z вверх.
                # Матрица перехода T_ros_opt
                R_ros_opt = np.array([
                    [0.0, 0.0, 1.0],  # X_ros = Z_opt
                    [-1.0, 0.0, 0.0], # Y_ros = -X_opt
                    [0.0, -1.0, 0.0]  # Z_ros = -Y_opt
                ])
                T_ros_opt = np.eye(4)
                T_ros_opt[:3, :3] = R_ros_opt

                # Поза тега в ROS camera_link: T_cameraRos_tag = T_ros_opt @ T_cameraOpt_tag
                T_cameraRos_tag = T_ros_opt @ T_cameraOpt_tag

                pos_rel = T_cameraRos_tag[:3, 3]
                rot_rel = R.from_matrix(T_cameraRos_tag[:3, :3]).as_quat()

                # Заполняем сообщение детекции
                detection = TagDetection()
                detection.tag_id = tag_id
                
                detection.pose.position.x = pos_rel[0]
                detection.pose.position.y = pos_rel[1]
                detection.pose.position.z = pos_rel[2]

                detection.pose.orientation.x = rot_rel[0]
                detection.pose.orientation.y = rot_rel[1]
                detection.pose.orientation.z = rot_rel[2]
                detection.pose.orientation.w = rot_rel[3]

                array_msg.detections.append(detection)

        # Публикуем детекции
        try:
            self.publisher_.publish(array_msg)
        except Exception:
            return
        
        # Рисуем найденные маркеры и ID прямо на кадре
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Сжимаем кадр с разметкой в JPEG и публикуем
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
                return

        if detected_ids:
            self.get_logger().info(f"Video frame processed. Detected tags: {detected_ids}")

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
        except:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass

if __name__ == '__main__':
    main()
