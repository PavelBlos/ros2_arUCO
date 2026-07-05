import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import serial
import threading
import time
import numpy as np

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # Декларация ROS 2 параметров с динамической типизацией
        self.declare_parameter('port', '/dev/ttyUSB0', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('baudrate', 115200, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('wheel_radius', 0.030, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('base_radius', 0.122, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('steps_per_rev', 3200, ParameterDescriptor(dynamic_typing=True)) # 200 шагов * 16 микрошагов
        self.declare_parameter('sim', False, ParameterDescriptor(dynamic_typing=True)) # Режим симуляции без ESP32
        
        # Калибровочные множители (изменяются "на лету" из Web UI)
        self.declare_parameter('wheel_radius_multiplier', 1.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('base_radius_multiplier', 1.0, ParameterDescriptor(dynamic_typing=True))

        # Чтение параметров
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_radius = self.get_parameter('base_radius').value
        self.steps_per_rev = self.get_parameter('steps_per_rev').value
        self.sim = self.get_parameter('sim').value
        self.wheel_radius_multiplier = self.get_parameter('wheel_radius_multiplier').value
        self.base_radius_multiplier = self.get_parameter('base_radius_multiplier').value

        # Инициализация Serial или Симуляции
        self.ser = None
        if not self.sim:
            self.get_logger().info(f"Connecting to ESP32 on port {self.port} at {self.baudrate} baud...")
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                time.sleep(1.0) # даем ESP32 перезагрузиться после подключения
                self.get_logger().info("Successfully connected to ESP32!")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to ESP32: {str(e)}. Falling back to SIMULATION mode!")
                self.sim = True
        else:
            self.get_logger().info("Running in SIMULATION mode.")

        # Переменные для симуляции
        self.sim_vx = 0.0
        self.sim_vy = 0.0
        self.sim_w = 0.0

        # Переменные состояния одометрии
        self.last_pos_F = None
        self.last_pos_R = None
        self.last_pos_L = None
        
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        self.last_odom_time = self.get_clock().now()

        # Публикатор одометрии и подписчик на cmd_vel
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Поток для чтения данных из последовательного порта или таймер симуляции
        self.running = True
        if self.sim:
            self.sim_timer = self.create_timer(0.04, self.sim_odom_loop)
            self.last_sim_time = self.get_clock().now()
            self.read_thread = None
        else:
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()

        # Таймер для отслеживания изменений параметров
        self.create_timer(1.0, self.update_parameters)

    def update_parameters(self):
        """Обновление параметров "на лету" из ROS системы"""
        self.wheel_radius_multiplier = self.get_parameter('wheel_radius_multiplier').value
        self.base_radius_multiplier = self.get_parameter('base_radius_multiplier').value

    def cmd_vel_callback(self, msg):
        """Обработка входящих скоростей робота и отправка на ESP32"""
        if self.sim:
            self.sim_vx = msg.linear.x
            self.sim_vy = msg.linear.y
            self.sim_w = msg.angular.z
            return

        if self.ser is None or not self.ser.is_open:
            return

        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z

        # Вычисляем эффективные параметры с учетом калибровочных множителей
        r_eff = self.wheel_radius * self.wheel_radius_multiplier
        R_eff = self.base_radius * self.base_radius_multiplier
        
        # Коэффициент перевода скорости (м/с) в шаги в секунду
        steps_per_meter = self.steps_per_rev / (2.0 * np.pi * r_eff)

        # Обратная кинематика 3-колесного Omni робота:
        # F - переднее колесо (sideways), R - правое, L - левое
        v_F = -vy + R_eff * w
        v_R = 0.5 * vy + 0.866 * vx + R_eff * w
        v_L = 0.5 * vy - 0.866 * vx + R_eff * w

        # Переводим в целые шаги/сек
        speed_F = int(v_F * steps_per_meter)
        speed_R = int(v_R * steps_per_meter)
        speed_L = int(v_L * steps_per_meter)

        # Отправляем команду в ESP32
        cmd_str = f"s {speed_F} {speed_R} {speed_L}\n"
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Failed to send command to ESP32: {str(e)}")

    def serial_read_loop(self):
        """Фоновый цикл чтения одометрии от ESP32"""
        while self.running:
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.5)
                continue

            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('o '):
                    # Разбор строки: "o <pos_F> <pos_R> <pos_L>"
                    parts = line.split()
                    if len(parts) == 4:
                        pos_F = int(parts[1])
                        pos_R = int(parts[2])
                        pos_L = int(parts[3])
                        self.process_odometry(pos_F, pos_R, pos_L)
                elif line.startswith('w '):
                    self.get_logger().warn(f"ESP32 Warning: {line[2:]}")

            except Exception as e:
                self.get_logger().error(f"Error reading serial: {str(e)}")
                time.sleep(0.1)

    def sim_odom_loop(self):
        """Математическое интегрирование скоростей для режима симуляции"""
        now = self.get_clock().now()
        dt = (now - self.last_sim_time).nanoseconds / 1e9
        self.last_sim_time = now
        
        if dt <= 0 or dt > 0.5:
            dt = 0.04

        # Интегрируем угол
        self.odom_theta += self.sim_w * dt
        self.odom_theta = np.arctan2(np.sin(self.odom_theta), np.cos(self.odom_theta))

        # Локальное смещение
        dx_local = self.sim_vx * dt
        dy_local = self.sim_vy * dt

        # Глобальное смещение по текущему углу
        cos_th = np.cos(self.odom_theta)
        sin_th = np.sin(self.odom_theta)
        dx_global = dx_local * cos_th - dy_local * sin_th
        dy_global = dx_local * sin_th + dy_local * cos_th

        self.odom_x += dx_global
        self.odom_y += dy_global

        # Публикуем одометрию
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0

        q = self.euler_to_quaternion(0.0, 0.0, self.odom_theta)
        odom_msg.pose.pose.orientation = q

        # Симулируем небольшой реалистичный шум одометрии
        noise_vx = np.random.normal(0, 0.001) if self.sim_vx != 0 else 0.0
        noise_vy = np.random.normal(0, 0.001) if self.sim_vy != 0 else 0.0
        noise_w = np.random.normal(0, 0.003) if self.sim_w != 0 else 0.0

        odom_msg.twist.twist.linear.x = self.sim_vx + noise_vx
        odom_msg.twist.twist.linear.y = self.sim_vy + noise_vy
        odom_msg.twist.twist.angular.z = self.sim_w + noise_w

        self.odom_pub.publish(odom_msg)

    def process_odometry(self, pos_F, pos_R, pos_L):
        """Прямая кинематика: пересчет шагов колес в перемещение робота"""
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        if dt <= 0:
            dt = 0.04

        if self.last_pos_F is None:
            # Инициализация начальных значений
            self.last_pos_F = pos_F
            self.last_pos_R = pos_R
            self.last_pos_L = pos_L
            return

        # Изменение шагов для каждого колеса
        ds_F = pos_F - self.last_pos_F
        ds_R = pos_R - self.last_pos_R
        ds_L = pos_L - self.last_pos_L

        self.last_pos_F = pos_F
        self.last_pos_R = pos_R
        self.last_pos_L = pos_L

        # Вычисляем эффективные параметры
        r_eff = self.wheel_radius * self.wheel_radius_multiplier
        R_eff = self.base_radius * self.base_radius_multiplier
        
        # Шагов на метр пути
        steps_per_meter = self.steps_per_rev / (2.0 * np.pi * r_eff)

        # Переводим приращение шагов в метры
        dq_F = ds_F / steps_per_meter
        dq_R = ds_R / steps_per_meter
        dq_L = ds_L / steps_per_meter

        # Прямая кинематика 3-колесного Omni робота (смещения в локальной системе координат)
        dx_local = (dq_R - dq_L) / np.sqrt(3.0)
        dy_local = (dq_R + dq_L - 2.0 * dq_F) / 3.0
        dtheta = (dq_F + dq_R + dq_L) / (3.0 * R_eff)

        # Интегрируем угловую координату
        self.odom_theta += dtheta
        
        # Переводим локальные перемещения в глобальную систему координат (odom)
        cos_th = np.cos(self.odom_theta)
        sin_th = np.sin(self.odom_theta)
        
        dx_global = dx_local * cos_th - dy_local * sin_th
        dy_global = dx_local * sin_th + dy_local * cos_th

        self.odom_x += dx_global
        self.odom_y += dy_global

        # Расчет мгновенных скоростей
        vx = dx_local / dt
        vy = dy_local / dt
        w = dtheta / dt

        # Публикуем сообщение Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Запись координат позы (pose)
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0

        # Кватернион поворота
        q = self.euler_to_quaternion(0.0, 0.0, self.odom_theta)
        odom_msg.pose.pose.orientation = q

        # Скорость (twist)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(odom_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Конвертер углов Эйлера в ROS кватернион"""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            # Перед закрытием останавливаем моторы
            try:
                self.ser.write(b"s 0 0 0\n")
                self.ser.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
