import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Empty
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Декларируем параметры с поддержкой динамического изменения
        self.declare_parameter('look_ahead_distance', 0.001, ParameterDescriptor(dynamic_typing=True)) # м
        self.declare_parameter('max_linear_velocity', 0.18, ParameterDescriptor(dynamic_typing=True)) # м/с
        self.declare_parameter('max_angular_velocity', 0.8, ParameterDescriptor(dynamic_typing=True)) # рад/с
        self.declare_parameter('kp_linear', 0.8, ParameterDescriptor(dynamic_typing=True))            # П-коэффициент для скорости
        self.declare_parameter('kp_angular', 1.5, ParameterDescriptor(dynamic_typing=True))           # П-коэффициент для вращения
        self.declare_parameter('goal_tolerance', 0.01, ParameterDescriptor(dynamic_typing=True))      # м (точность доезда до финиша)
        self.declare_parameter('decel_dist', 0.30, ParameterDescriptor(dynamic_typing=True))          # м (расстояние торможения)
        self.declare_parameter('min_linear_velocity', 0.04, ParameterDescriptor(dynamic_typing=True))  # м/с (минимальная скорость)
        self.declare_parameter('yaw_deadzone_dist', 0.02, ParameterDescriptor(dynamic_typing=True))    # м (зона отключения вращения)
        self.declare_parameter('waypoint_tolerance', 0.01, ParameterDescriptor(dynamic_typing=True))    # м (прохождение точек, 0.0 = PP)
        self.declare_parameter('kp_turn_decel', 0.2, ParameterDescriptor(dynamic_typing=True))         # замедление на поворотах

        # Текущие значения параметров
        self.get_params()

        # Подписчики и публикатор
        self.pose_sub = self.create_subscription(PoseStamped, '/estimated_pose', self.pose_callback, 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/follower_status', 10)
        self.start_work_sub = self.create_subscription(Empty, '/start_work', self.start_work_callback, 10)

        # Состояние ноды
        self.current_pose = None
        self.path_waypoints = []
        self.active_path = False
        self.current_waypoint_idx = 0
        self.is_pre_positioning = False
        self.status = "idle"

        # Таймер для периодического чтения параметров (динамическая калибровка)
        self.create_timer(1.0, self.get_params)
        self.get_logger().info("Path Follower (Pure Pursuit Omni) node initialized.")

    def publish_status(self, status_str):
        self.status = status_str
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)

    def start_work_callback(self, msg):
        if self.active_path and self.is_pre_positioning:
            self.is_pre_positioning = False
            self.get_logger().info("🚀 Start work signal received! Commencing path tracking.")
            self.publish_status("tracking")

    def get_params(self):
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.decel_dist = self.get_parameter('decel_dist').value
        self.min_linear_velocity = self.get_parameter('min_linear_velocity').value
        self.yaw_deadzone_dist = self.get_parameter('yaw_deadzone_dist').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.kp_turn_decel = self.get_parameter('kp_turn_decel').value

    def path_callback(self, msg):
        """Получение нового маршрута"""
        self.path_waypoints = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.path_waypoints.append((x, y))

        if len(self.path_waypoints) > 0:
            self.active_path = True
            self.current_waypoint_idx = 0
            self.is_pre_positioning = True # Активируем точный выход на старт
            self.publish_status("pre_positioning")
            self.get_logger().info(f"Received new path with {len(self.path_waypoints)} points. Pre-positioning to start...")
        else:
            self.active_path = False
            self.is_pre_positioning = False
            self.stop_robot()
            self.publish_status("idle")
            self.get_logger().info("Path cleared. Robot stopped.")

    def pose_callback(self, msg):
        """Получение текущей отфильтрованной позы робота и расчет управления"""
        self.current_pose = msg.pose
        if not self.active_path or len(self.path_waypoints) == 0:
            return

        # Извлекаем координаты робота
        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        ryaw = self.quaternion_to_yaw(self.current_pose.orientation)

        # Режим 1: Точный выход на стартовую точку (первый вейпоинт) и угловое выравнивание
        if self.is_pre_positioning:
            sx, sy = self.path_waypoints[0]
            dx = sx - rx
            dy = sy - ry
            dist_to_start = np.sqrt(dx**2 + dy**2)
            
            # Угол направления первого сегмента (курс старта)
            if len(self.path_waypoints) > 1:
                x0, y0 = self.path_waypoints[0]
                x1, y1 = self.path_waypoints[1]
                start_yaw = np.arctan2(y1 - y0, x1 - x0)
            else:
                start_yaw = 0.0
                
            yaw_err = start_yaw - ryaw
            yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))
            
            if dist_to_start < self.goal_tolerance and abs(yaw_err) < 0.05:
                # Встали на старт и довернулись по курсу! Стоим и ждем нажатия кнопки в UI
                self.stop_robot()
                self.publish_status("ready_to_work")
                return
            else:
                # Едем точно в стартовую точку с пропорциональным замедлением
                linear_speed = min(self.max_linear_velocity, self.kp_linear * dist_to_start)
                linear_speed = max(self.min_linear_velocity, linear_speed)
                
                vx_local = linear_speed * (dx * np.cos(ryaw) + dy * np.sin(ryaw)) / dist_to_start
                vy_local = linear_speed * (-dx * np.sin(ryaw) + dy * np.cos(ryaw)) / dist_to_start
                
                # Доворачиваем по курсу первого сегмента
                w = self.kp_angular * yaw_err
                w = np.clip(w, -self.max_angular_velocity, self.max_angular_velocity)
                
                self.publish_status("pre_positioning")
                cmd = Twist()
                cmd.linear.x = float(vx_local)
                cmd.linear.y = float(vy_local)
                cmd.angular.z = float(w)
                self.cmd_vel_pub.publish(cmd)
                return

        # Режим 2: Динамическое следование по траектории (Pure Pursuit)
        # 1. Проверяем расстояние до финальной точки маршрута
        # Предотвращаем преждевременный финиш: разрешаем останавливаться только если робот
        # продвинулся по маршруту хотя бы во вторую его половину (прошел 70% точек)
        min_finish_idx = max(1, int(len(self.path_waypoints) * 0.7))
        
        fx, fy = self.path_waypoints[-1]
        dist_to_finish = np.sqrt((fx - rx)**2 + (fy - ry)**2)
        if dist_to_finish < self.goal_tolerance and self.current_waypoint_idx >= min_finish_idx:
            self.get_logger().info("🎉 Goal reached! Stopping robot.")
            self.active_path = False
            self.stop_robot()
            self.publish_status("finished")
            return

        self.publish_status("tracking")

        # 2. Поиск целевой точки на маршруте
        if self.waypoint_tolerance > 0.0:
            # Режим Strict Waypoint: едем строго к текущей точке, пока не попадем в ее радиус
            tx, ty = self.path_waypoints[self.current_waypoint_idx]
            dist_to_target = np.sqrt((tx - rx)**2 + (ty - ry)**2)
            
            # Если мы попали в waypoint_tolerance, переключаемся на следующую точку
            if dist_to_target < self.waypoint_tolerance:
                self.current_waypoint_idx = min(self.current_waypoint_idx + 1, len(self.path_waypoints) - 1)
                tx, ty = self.path_waypoints[self.current_waypoint_idx]
                
            target_point = (tx, ty)
        else:
            # Режим Pure Pursuit: стандартное следование по оглядыванию
            target_point = self.get_lookahead_point(rx, ry)

        # 3. Расчет вектора направления
        tx, ty = target_point
        dx = tx - rx
        dy = ty - ry
        dist = np.sqrt(dx**2 + dy**2)
        if dist == 0:
            dist = 0.001

        # 4. Расчет линейной скорости (круиз-контроль с замедлением перед финишем и поворотами)
        if dist_to_finish > self.decel_dist:
            linear_speed = self.max_linear_velocity
        else:
            # Линейное торможение перед финишем до min_linear_velocity
            ratio = (dist_to_finish - self.goal_tolerance) / (self.decel_dist - self.goal_tolerance)
            ratio = np.clip(ratio, 0.0, 1.0)
            linear_speed = self.min_linear_velocity + ratio * (self.max_linear_velocity - self.min_linear_velocity)

        # Применяем замедление перед поворотами (если настроено)
        if self.kp_turn_decel > 0.0:
            turn_slowdown = self.get_upcoming_turn_slowdown(ryaw)
            linear_speed *= turn_slowdown

        # Переводим вектор скорости в локальную СК робота
        vx_local = linear_speed * (dx * np.cos(ryaw) + dy * np.sin(ryaw)) / dist
        vy_local = linear_speed * (-dx * np.sin(ryaw) + dy * np.cos(ryaw)) / dist

        # 5. Расчет угловой скорости (отключается в yaw_deadzone_dist)
        if dist_to_finish < self.yaw_deadzone_dist or dist < self.yaw_deadzone_dist:
            w = 0.0
        else:
            target_yaw = np.arctan2(dy, dx)
            yaw_err = target_yaw - ryaw
            yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))
            w = self.kp_angular * yaw_err
            w = np.clip(w, -self.max_angular_velocity, self.max_angular_velocity)

        # Публикуем cmd_vel
        cmd = Twist()
        cmd.linear.x = float(vx_local)
        cmd.linear.y = float(vy_local)
        cmd.angular.z = float(w)
        self.cmd_vel_pub.publish(cmd)

    def get_lookahead_point(self, rx, ry):
        """Поиск целевой точки путем интегрирования расстояния вдоль пути"""
        if not self.path_waypoints:
            return (rx, ry)
            
        # 1. Находим индекс ближайшей точки на пути к роботу (в локальном окне по расстоянию)
        closest_idx = self.current_waypoint_idx
        min_dist = 999.0
        accum_dist = 0.0
        search_window = max(0.40, 2.0 * self.look_ahead_distance)
        
        for i in range(self.current_waypoint_idx, len(self.path_waypoints)):
            if i > self.current_waypoint_idx:
                x1, y1 = self.path_waypoints[i-1]
                x2, y2 = self.path_waypoints[i]
                accum_dist += np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                
            if accum_dist > search_window:
                break
                
            wx, wy = self.path_waypoints[i]
            d = np.sqrt((wx - rx)**2 + (wy - ry)**2)
            if d < min_dist:
                min_dist = d
                closest_idx = i
                
        # Запоминаем текущий прогресс (чтобы не ехать назад)
        self.current_waypoint_idx = max(self.current_waypoint_idx, closest_idx)
        
        # 2. Двигаемся вперед по пути и суммируем расстояние вдоль траектории
        accum_dist = 0.0
        target_idx = self.current_waypoint_idx
        
        for i in range(self.current_waypoint_idx, len(self.path_waypoints) - 1):
            x1, y1 = self.path_waypoints[i]
            x2, y2 = self.path_waypoints[i+1]
            segment_len = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            accum_dist += segment_len
            
            if accum_dist >= self.look_ahead_distance:
                target_idx = i + 1
                break
        else:
            # Если до конца пути расстояние меньше look_ahead, то целевой точкой становится конец пути
            target_idx = len(self.path_waypoints) - 1
            
        return self.path_waypoints[target_idx]

    def get_upcoming_turn_slowdown(self, ryaw):
        """Расчет коэффициента замедления на основе разницы между курсом робота и направлением пути впереди"""
        if len(self.path_waypoints) < 3 or self.current_waypoint_idx >= len(self.path_waypoints) - 2:
            return 1.0
            
        curr = self.current_waypoint_idx
        
        # Находим предел заглядывания (например, 0.25м)
        lookahead_dist = 0.25
        accum_dist = 0.0
        
        max_angle_diff = 0.0
        
        for i in range(curr, len(self.path_waypoints) - 1):
            xa, ya = self.path_waypoints[i]
            xb, yb = self.path_waypoints[i+1]
            segment_len = np.sqrt((xb - xa)**2 + (yb - ya)**2)
            accum_dist += segment_len
            
            # Угол этого сегмента
            yaw_segment = np.arctan2(yb - ya, xb - xa)
            
            # Разница между физическим курсом робота ryaw и направлением сегмента пути
            diff = yaw_segment - ryaw
            diff = abs(np.arctan2(np.sin(diff), np.cos(diff)))
            if diff > max_angle_diff:
                max_angle_diff = diff
                
            if accum_dist >= lookahead_dist:
                break
                
        # Замедляемся на основе максимальной разницы углов впереди
        decel = 1.0 - np.clip(self.kp_turn_decel * max_angle_diff, 0.0, 0.7)
        return decel

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def quaternion_to_yaw(self, q):
        """Конвертер кватерниона в угол Yaw (в радианах)"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
