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

        # Запуск веб-сервера для real-time визуализации траектории
        self.web_port = 8080
        self.start_web_server()

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

            # Обновляем длину пути в реальном времени
            if len(self.raw_trajectory_x) > 0:
                dx = avg_pos[0] - self.raw_trajectory_x[-1]
                dy = avg_pos[1] - self.raw_trajectory_y[-1]
                dz = avg_pos[2] - self.raw_trajectory_z[-1]
                self.raw_path_length += np.sqrt(dx*dx + dy*dy + dz*dz)

            if len(self.filtered_trajectory_x) > 0:
                dx = filtered_pos[0] - self.filtered_trajectory_x[-1]
                dy = filtered_pos[1] - self.filtered_trajectory_y[-1]
                dz = filtered_pos[2] - self.filtered_trajectory_z[-1]
                self.filtered_path_length += np.sqrt(dx*dx + dy*dy + dz*dz)

            # Сохраняем точки траектории (сырую и отфильтрованную)
            self.raw_trajectory_x.append(avg_pos[0])
            self.raw_trajectory_y.append(avg_pos[1])
            self.raw_trajectory_z.append(avg_pos[2])

            self.filtered_trajectory_x.append(filtered_pos[0])
            self.filtered_trajectory_y.append(filtered_pos[1])
            self.filtered_trajectory_z.append(filtered_pos[2])

            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.trajectory_timestamps.append(stamp_sec)

            # Извлекаем Yaw (угол поворота вокруг оси Z)
            try:
                yaw_deg = R.from_quat(filtered_rot).as_euler('zyx', degrees=True)[0]
            except Exception:
                yaw_deg = 0.0

            # Отправляем обновление на веб-клиенты
            web_data = {
                "type": "pose",
                "x": float(filtered_pos[0]),
                "y": float(filtered_pos[1]),
                "z": float(filtered_pos[2]),
                "yaw": float(yaw_deg),
                "raw_x": float(avg_pos[0]),
                "raw_y": float(avg_pos[1]),
                "raw_z": float(avg_pos[2]),
                "distance_raw": float(self.raw_path_length),
                "distance_filtered": float(self.filtered_path_length),
                "timestamp": float(stamp_sec),
                "detected_tags": tag_ids
            }
            with sse_clients_lock:
                for q in sse_clients:
                    q.put(web_data)

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

    def destroy_node(self):
        if hasattr(self, 'server'):
            self.get_logger().info("Stopping web server...")
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception as e:
                self.get_logger().warn(f"Error closing web server: {str(e)}")
        super().destroy_node()

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

# --- ВЕБ-СЕРВЕР ДЛЯ ОТОБРАЖЕНИЯ В РЕАЛЬНОМ ВРЕМЕНИ ---

sse_clients = []
sse_clients_lock = threading.Lock()

class WebServerHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        # Отключаем логирование запросов, чтобы не мусорить в консоли
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_TEMPLATE.encode('utf-8'))
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
        .control-panel {
            position: absolute;
            top: 20px;
            right: 20px;
            display: flex;
            gap: 8px;
            z-index: 10;
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

        <div id="terminal-log">
            <div class="log-entry"><span class="log-time">Система</span>Веб-интерфейс готов к получению данных.</div>
        </div>
    </div>

    <div id="map-container">
        <div class="control-panel">
            <button class="icon-btn active" id="toggle-raw">Показать сырой путь</button>
            <button class="icon-btn" id="toggle-grid">Сетка</button>
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
                const step = 0.5; // Шаг в метрах
                const gridLimit = 15;
                ctx.lineWidth = 1;

                // Вертикальные линии
                for (let x = -gridLimit; x <= gridLimit; x += step) {
                    ctx.strokeStyle = x === 0 ? 'rgba(102, 252, 241, 0.25)' : 'rgba(255, 255, 255, 0.03)';
                    ctx.beginPath();
                    const px = panX + x * zoom;
                    ctx.moveTo(px, 0);
                    ctx.lineTo(px, canvas.height);
                    ctx.stroke();

                    if (Math.abs(x) % 1 === 0) {
                        ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
                        ctx.font = '10px monospace';
                        ctx.textAlign = 'center';
                        ctx.fillText(x + 'm', px, canvas.height - 10);
                    }
                }

                // Горизонтальные линии
                for (let y = -gridLimit; y <= gridLimit; y += step) {
                    ctx.strokeStyle = y === 0 ? 'rgba(255, 77, 77, 0.25)' : 'rgba(255, 255, 255, 0.03)';
                    ctx.beginPath();
                    const py = panY - y * zoom;
                    ctx.moveTo(0, py);
                    ctx.lineTo(canvas.width, py);
                    ctx.stroke();

                    if (Math.abs(y) % 1 === 0) {
                        ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
                        ctx.font = '10px monospace';
                        ctx.textAlign = 'left';
                        ctx.fillText(y + 'm', 10, py - 4);
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

        // Zoom (Масштабирование)
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
            const mouseX = e.clientX - canvas.getBoundingClientRect().left;
            const mouseY = e.clientY - canvas.getBoundingClientRect().top;

            const xMeters = (mouseX - panX) / zoom;
            const yMeters = (panY - mouseY) / zoom;

            zoom = Math.min(Math.max(zoom * zoomFactor, 15), 1000);

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

                    if (autoCenter) {
                        centerMap();
                    }
                    draw();
                }
            };
        }

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
