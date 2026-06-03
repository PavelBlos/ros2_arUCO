import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from fake_tag_interfaces.msg import TagDetectionArray
import tf2_ros
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

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
        self.declare_parameter('filter_alpha', 0.15)
        self.filter_alpha = self.get_parameter('filter_alpha').get_parameter_value().double_value

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

            # 4. Применяем фильтрацию Exponential Moving Average (EMA) и Slerp
            if self.last_pos is None:
                self.last_pos = avg_pos
                self.last_rot = avg_rot
                filtered_pos = avg_pos
                filtered_rot = avg_rot
            else:
                alpha = self.filter_alpha
                filtered_pos = alpha * avg_pos + (1.0 - alpha) * self.last_pos
                
                # Slerp интерполяция для сглаживания кватернионов поворота
                try:
                    q1 = self.last_rot / np.linalg.norm(self.last_rot)
                    q2 = avg_rot / np.linalg.norm(avg_rot)
                    key_rots = R.from_quat([q1, q2])
                    slerp = Slerp([0.0, 1.0], key_rots)
                    filtered_rot = slerp(alpha).as_quat()
                except Exception as slerp_err:
                    self.get_logger().debug(f"Slerp failed, using raw rotation: {str(slerp_err)}")
                    filtered_rot = avg_rot
                
                self.last_pos = filtered_pos
                self.last_rot = filtered_rot

            # 5. Публикуем финальную ОТФИЛЬТРОВАННУЮ позу робота в PoseStamped
            robot_pose = PoseStamped()
            robot_pose.header.stamp = msg.header.stamp
            robot_pose.header.frame_id = 'map'
            
            robot_pose.pose.position.x = filtered_pos[0]
            robot_pose.pose.position.y = filtered_pos[1]
            robot_pose.pose.position.z = filtered_pos[2]
            
            robot_pose.pose.orientation.x = filtered_rot[0]
            robot_pose.pose.orientation.y = filtered_rot[1]
            robot_pose.pose.orientation.z = filtered_rot[2]
            robot_pose.pose.orientation.w = filtered_rot[3]
            
            self.pose_pub.publish(robot_pose)

            # 6. Транслируем динамическое TF-преобразование map -> base_link (отфильтрованное)
            t_msg = TransformStamped()
            t_msg.header.stamp = msg.header.stamp
            t_msg.header.frame_id = 'map'
            t_msg.child_frame_id = 'base_link'
            
            t_msg.transform.translation.x = filtered_pos[0]
            t_msg.transform.translation.y = filtered_pos[1]
            t_msg.transform.translation.z = filtered_pos[2]
            
            t_msg.transform.rotation.x = filtered_rot[0]
            t_msg.transform.rotation.y = filtered_rot[1]
            t_msg.transform.rotation.z = filtered_rot[2]
            t_msg.transform.rotation.w = filtered_rot[3]
            
            self.tf_broadcaster.sendTransform(t_msg)

            tag_ids = [d.tag_id for d in detections]
            self.get_logger().info(
                f"Localized by tags {tag_ids}: X={filtered_pos[0]:.2f}, Y={filtered_pos[1]:.2f}, Z={filtered_pos[2]:.2f}"
            )

            # Сохраняем точки траектории (сырую и отфильтрованную)
            self.raw_trajectory_x.append(avg_pos[0])
            self.raw_trajectory_y.append(avg_pos[1])
            self.raw_trajectory_z.append(avg_pos[2])

            self.filtered_trajectory_x.append(filtered_pos[0])
            self.filtered_trajectory_y.append(filtered_pos[1])
            self.filtered_trajectory_z.append(filtered_pos[2])

            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.trajectory_timestamps.append(stamp_sec)

        except Exception as e:
            self.get_logger().error(f"Error in tag_callback: {str(e)}")

    def save_trajectory_and_shutdown(self):
        if not self.raw_trajectory_x:
            self.get_logger().warn("Trajectory is empty. Cannot generate plot or statistics.")
            rclpy.shutdown()
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

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
