import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        
        # Декларируем ROS2 параметры
        self.declare_parameter('video_path', 'config/calibration.mp4')
        self.declare_parameter('square_size', 0.025) # Размер квадрата в метрах (по умолчанию 25мм)
        self.declare_parameter('board_width', 8)     # Количество внутренних углов по ширине (9 квадратов -> 8 углов)
        self.declare_parameter('board_height', 6)    # Количество внутренних углов по высоте (7 квадратов -> 6 углов)
        
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self.board_width = self.get_parameter('board_width').get_parameter_value().integer_value
        self.board_height = self.get_parameter('board_height').get_parameter_value().integer_value

        self.get_logger().info(f"Calibration node started.")
        self.get_logger().info(f"Parameters: video={self.video_path}, square={self.square_size}m, grid={self.board_width}x{self.board_height}")

        # Запуск калибровки
        self.run_calibration()

    def run_calibration(self):
        # 1. Проверяем наличие видеофайла
        if not os.path.exists(self.video_path):
            # Пробуем найти относительный путь воркспейса
            self.get_logger().error(f"Video file not found at {self.video_path}. Please check the path.")
            return

        # 2. Подготовка 3D-координат идеальных углов шахматной доски (object points)
        # Они имеют вид (0,0,0), (1,0,0), (2,0,0) ... (W-1, H-1, 0) умноженные на размер квадрата
        objp = np.zeros((self.board_width * self.board_height, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_width, 0:self.board_height].T.reshape(-1, 2)
        objp *= self.square_size

        objpoints = [] # 3d точки в физическом мире
        imgpoints = [] # 2d точки на кадрах изображения

        # 3. Открываем видеофайл
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")
            return

        frame_count = 0
        success_count = 0
        processed_frames = 0
        
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.get_logger().info(f"Video opened. Size: {width}x{height}, Total frames: {total_frames}")

        # Критерии для субпиксельного уточнения координат углов
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 4. Считываем видео кадр за кадром
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            frame_count += 1
            
            # Обрабатываем каждый 15-й кадр, чтобы собрать разнообразные ракурсы без перегрузки памяти
            if frame_count % 15 != 0:
                continue

            processed_frames += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Ищем углы шахматной доски
            ret_corners, corners = cv2.findChessboardCorners(gray, (self.board_width, self.board_height), None)

            if ret_corners:
                success_count += 1
                objpoints.append(objp)
                
                # Уточняем координаты углов до субпиксельной точности
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                
                self.get_logger().info(f"Frame {frame_count}: Chessboard detected! Total successes: {success_count}")
            else:
                self.get_logger().debug(f"Frame {frame_count}: Chessboard NOT detected.")

        cap.release()

        self.get_logger().info(f"Finished parsing video. Processed {processed_frames} frames, detected board on {success_count} frames.")

        # 5. Проверяем, набралось ли достаточное количество успешных кадров
        if success_count < 10:
            self.get_logger().error(f"Not enough successful detections (only {success_count}/10 required). Calibration aborted.")
            return

        # 6. Запускаем математический алгоритм калибровки OpenCV
        self.get_logger().info("Running camera calibration algorithm...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        if ret:
            self.get_logger().info("Camera successfully calibrated!")
            self.get_logger().info(f"RMS error: {ret:.4f}")
            self.get_logger().info(f"Camera Matrix:\n{mtx}")
            self.get_logger().info(f"Distortion Coefficients:\n{dist.ravel()}")

            # 7. Записываем результаты калибровки в файл конфигурации camera_info.yaml
            calib_data = {
                'image_width': width,
                'image_height': height,
                'camera_matrix': mtx.flatten().tolist(),
                'distortion_coefficients': dist.flatten().tolist()
            }

            # Путь для сохранения в source-директорию (чтобы файл остался в проекте для будущих сборок)
            try:
                # Пытаемся сохранить напрямую в src/fake_tag_publisher/config/
                pkg_src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                src_config_path = os.path.join(pkg_src_dir, 'config', 'camera_info.yaml')
                
                with open(src_config_path, 'w') as f:
                    yaml.dump(calib_data, f, default_flow_style=False)
                self.get_logger().info(f"Successfully saved calibration to SOURCE config: {src_config_path}")
            except Exception as src_err:
                self.get_logger().warn(f"Could not save to source directory: {str(src_err)}")

            # Путь для сохранения в share-директорию (для текущей работающей сборки)
            try:
                share_dir = get_package_share_directory('fake_tag_publisher')
                share_config_path = os.path.join(share_dir, 'config', 'camera_info.yaml')
                with open(share_config_path, 'w') as f:
                    yaml.dump(calib_data, f, default_flow_style=False)
                self.get_logger().info(f"Successfully saved calibration to INSTALLED share: {share_config_path}")
            except Exception as share_err:
                self.get_logger().error(f"Failed to save to share directory: {str(share_err)}")

        else:
            self.get_logger().error("OpenCV Calibration failed.")

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    # Так как калибровка одноразовая, нам не нужно крутить spin в цикле. Нода выполнит расчет и завершится.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
