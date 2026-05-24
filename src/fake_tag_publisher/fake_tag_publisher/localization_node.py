import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from fake_tag_interfaces.msg import TagDetectionArray
import tf2_ros
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R

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
        
        # Публикатор оцененного положения робота в топик /estimated_pose
        self.pose_pub = self.create_publisher(
            PoseStamped, '/estimated_pose', 10)
        
        self.get_logger().info('Localization node started successfully (Multi-tag Data Fusion)')

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

    def tag_callback(self, msg):
        try:
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
                # По умолчанию: камера на роботе смотрит вверх (pitch = -90 градусов)
                camera_rot = R.from_euler('xyz', [0.0, -np.pi / 2.0, 0.0])
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

            # 4. Публикуем финальную усредненную позу робота в PoseStamped
            robot_pose = PoseStamped()
            robot_pose.header.stamp = msg.header.stamp
            robot_pose.header.frame_id = 'map'
            
            robot_pose.pose.position.x = avg_pos[0]
            robot_pose.pose.position.y = avg_pos[1]
            robot_pose.pose.position.z = avg_pos[2]
            
            robot_pose.pose.orientation.x = avg_rot[0]
            robot_pose.pose.orientation.y = avg_rot[1]
            robot_pose.pose.orientation.z = avg_rot[2]
            robot_pose.pose.orientation.w = avg_rot[3]
            
            self.pose_pub.publish(robot_pose)

            # 5. Транслируем динамическое TF-преобразование map -> base_link
            t_msg = TransformStamped()
            t_msg.header.stamp = msg.header.stamp
            t_msg.header.frame_id = 'map'
            t_msg.child_frame_id = 'base_link'
            
            t_msg.transform.translation.x = avg_pos[0]
            t_msg.transform.translation.y = avg_pos[1]
            t_msg.transform.translation.z = avg_pos[2]
            
            t_msg.transform.rotation.x = avg_rot[0]
            t_msg.transform.rotation.y = avg_rot[1]
            t_msg.transform.rotation.z = avg_rot[2]
            t_msg.transform.rotation.w = avg_rot[3]
            
            self.tf_broadcaster.sendTransform(t_msg)

            tag_ids = [d.tag_id for d in detections]
            self.get_logger().info(
                f"Localized by tags {tag_ids}: X={avg_pos[0]:.2f}, Y={avg_pos[1]:.2f}, Z={avg_pos[2]:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error in tag_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
