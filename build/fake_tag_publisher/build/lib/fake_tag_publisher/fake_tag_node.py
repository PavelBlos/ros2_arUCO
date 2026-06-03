import rclpy
from rclpy.node import Node
from fake_tag_interfaces.msg import TagDetection, TagDetectionArray
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math

class FakeTagPublisher(Node):

    def __init__(self):
        super().__init__('fake_tag_publisher')

        # Публикуем в топик /fake_tag массив обнаруженных меток
        self.publisher_ = self.create_publisher(TagDetectionArray, '/fake_tag', 10)

        # Загрузка базы данных меток из конфигурационного файла tags_config.yaml
        self.tags_db = {}
        self.load_tags_config()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0

        # Статическое смещение камеры base_link -> camera_link (камера смотрит вверх: pitch = -90 градусов)
        # T_robot_camera
        camera_rot = R.from_euler('xyz', [0.0, -math.pi / 2.0, 0.0])
        self.T_robot_camera = np.eye(4)
        self.T_robot_camera[:3, :3] = camera_rot.as_matrix()
        self.T_robot_camera[:3, 3] = [0.0, 0.0, 0.0] # Камера в центре робота

        # Критерии видимости камеры (Field of View)
        self.max_detection_dist = 3.5  # Максимальная дальность детекции (3.5 метра)
        self.fov_angle_rad = math.radians(55.0)  # Половина угла обзора камеры (55 градусов)

        self.get_logger().info('Fake tag simulation node started (Upward camera & Ceiling tags)')

    def load_tags_config(self):
        try:
            share_dir = get_package_share_directory('fake_tag_publisher')
            config_path = os.path.join(share_dir, 'config', 'tags_config.yaml')
            
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                self.tags_db = config_data.get('tags', {})
                self.get_logger().info(f"Loaded {len(self.tags_db)} ceiling tags from config.")
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {str(e)}")

    def timer_callback(self):
        # 1. Симулируем движение робота по эллиптической траектории в плоскости пола (Z=0)
        # Эллипс: x = 2.5 * cos(t), y = 1.8 * sin(t)
        x_r = 2.5 * math.cos(self.t)
        y_r = 1.8 * math.sin(self.t)
        z_r = 0.0
        # Робот развернут по касательной к эллипсу
        dx = -2.5 * math.sin(self.t)
        dy = 1.8 * math.cos(self.t)
        yaw_r = math.atan2(dy, dx)

        # Матрица T_map_robot
        robot_rot = R.from_euler('xyz', [0.0, 0.0, yaw_r])
        T_map_robot = np.eye(4)
        T_map_robot[:3, :3] = robot_rot.as_matrix()
        T_map_robot[:3, 3] = [x_r, y_r, z_r]

        # 2. Вычисляем положение камеры на карте: T_map_camera = T_map_robot @ T_robot_camera
        T_map_camera = T_map_robot @ self.T_robot_camera

        # Создаем сообщение-контейнер для обнаруженных меток
        array_msg = TagDetectionArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "camera_link"

        visible_tags_ids = []

        # 3. Перебираем все метки из конфигурации и проверяем их видимость
        for tag_name, tag_info in self.tags_db.items():
            tag_id = int(tag_name.split('_')[1])
            
            # Матрица T_map_tag
            tag_rot = R.from_euler('xyz', [tag_info['roll'], tag_info['pitch'], tag_info['yaw']])
            T_map_tag = np.eye(4)
            T_map_tag[:3, :3] = tag_rot.as_matrix()
            T_map_tag[:3, 3] = [tag_info['x'], tag_info['y'], tag_info['z']]

            # Относительное положение метки в камере: T_camera_tag = (T_map_camera)^-1 * T_map_tag
            T_camera_tag = np.linalg.inv(T_map_camera) @ T_map_tag
            
            pos_rel = T_camera_tag[:3, 3]
            dist = np.linalg.norm(pos_rel)

            # Проверяем критерий FOV (Field of View)
            # В нашей системе камера смотрит вдоль своей оси X (так как приложен pitch -90 градусов).
            # Угол отклонения метки от оптической оси камеры (оси X):
            if dist > 0:
                angle_to_tag = math.acos(pos_rel[0] / dist) # угол между вектором на тег и осью X камеры
            else:
                angle_to_tag = math.pi

            # Если метка близко и находится в конусе обзора камеры
            if dist <= self.max_detection_dist and angle_to_tag <= self.fov_angle_rad:
                rot_rel = R.from_matrix(T_camera_tag[:3, :3]).as_quat()

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
                visible_tags_ids.append(tag_id)

        # Публикуем массив обнаружений
        self.publisher_.publish(array_msg)
        
        self.get_logger().info(
            f"Robot at ({x_r:.2f}, {y_r:.2f}). "
            f"Visible ceiling tags: {visible_tags_ids}"
        )

        self.t += 0.035 # Шаг по времени

def main(args=None):
    rclpy.init(args=args)
    node = FakeTagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
