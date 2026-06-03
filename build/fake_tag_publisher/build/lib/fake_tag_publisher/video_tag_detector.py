import rclpy
from rclpy.node import Node
from fake_tag_interfaces.msg import TagDetection, TagDetectionArray
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import os
import yaml
from scipy.spatial.transform import Rotation as R

class VideoTagDetector(Node):
    def __init__(self):
        super().__init__('video_tag_detector')
        
        # Декларируем ROS2 параметры
        self.declare_parameter('video_path', 'config/robot_drive.mp4')
        self.declare_parameter('calibration_path', 'config/camera_info.yaml')
        self.declare_parameter('marker_length', 0.15) # Реальный размер маркера в метрах (например, 15см)
        self.declare_parameter('detection_rate', 30.0) # Частота обработки кадров (30 Гц)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_100') # Семейство ArUco маркеров
        self.declare_parameter('loop_video', True) # Циклический повтор видео

        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.calibration_path = self.get_parameter('calibration_path').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.detection_rate = self.get_parameter('detection_rate').get_parameter_value().double_value
        self.aruco_dict_name = self.get_parameter('aruco_dictionary').get_parameter_value().string_value
        loop_param = self.get_parameter('loop_video')
        if loop_param.type_ == rclpy.Parameter.Type.STRING:
            self.loop_video = loop_param.value.lower() in ['true', '1', 'yes']
        else:
            self.loop_video = bool(loop_param.value)

        self.get_logger().info('Video Tag Detector node starting...')
        self.get_logger().info(f"Params: video={self.video_path}, calib={self.calibration_path}, marker_size={self.marker_length}m, loop={self.loop_video}")

        # 1. Загрузка параметров калибровки камеры
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration()

        # 2. Инициализация OpenCV детектора ArUco (словарь DICT_4X4_50)
        self.init_aruco_detector()

        # 3. Открываем видеофайл
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")
            return
        
        self.get_logger().info(f"Successfully opened video file: {self.video_path}")

        # 4. Создаем публикататор и таймер обработки кадров
        self.publisher_ = self.create_publisher(TagDetectionArray, '/fake_tag', 10)
        
        timer_period = 1.0 / self.detection_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

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
        
        try:
            # OpenCV 4.7+
            self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            self.parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            self.detect_func = lambda img: self.detector.detectMarkers(img)
            self.get_logger().info(f"Initialized OpenCV 4.7+ ArUco detector ({self.aruco_dict_name})")
        except AttributeError:
            # Старые версии OpenCV
            self.dictionary = cv2.aruco.Dictionary_get(dict_id)
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.detect_func = lambda img: cv2.aruco.detectMarkers(img, self.dictionary, parameters=self.parameters)
            self.get_logger().info(f"Initialized Legacy OpenCV ArUco detector ({self.aruco_dict_name})")

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        # Считываем следующий кадр
        ret, frame = self.cap.read()
        
        # Если видео закончилось
        if not ret:
            if self.loop_video:
                self.get_logger().info("Video finished. Looping back to the beginning.")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            else:
                self.get_logger().info("Video finished. Sending finished signal and stopping timer.")
                # Отправляем сообщение окончания
                array_msg = TagDetectionArray()
                array_msg.header.stamp = self.get_clock().now().to_msg()
                array_msg.header.frame_id = "finished"
                self.publisher_.publish(array_msg)
                
                # Останавливаем таймер
                self.timer.cancel()
                return

        # 1. Перевод в оттенки серого напрямую (без ресурсоемкого cv2.undistort для всего кадра)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. Детектируем маркеры ArUco
        corners, ids, rejected = self.detect_func(gray)

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
        self.publisher_.publish(array_msg)
        
        if detected_ids:
            self.get_logger().info(f"Video frame processed. Detected tags: {detected_ids}")

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = VideoTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
