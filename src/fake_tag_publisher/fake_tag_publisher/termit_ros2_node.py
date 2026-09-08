#!/usr/bin/env python3
"""
Termit Omni Robot - ROS 2 Hardware Driver Node
==============================================
Готовая нода ROS 2 (Humble / Iron / Jazzy) для запуска на Raspberry Pi.

Интерфейсы ROS 2:
- Подписка:  /cmd_vel (geometry_msgs/msg/Twist) -> движение платформы
- Публикация: /odom (nav_msgs/msg/Odometry)     -> одометрия колес (20-50 Гц)
- Публикация: tf (odom -> base_link)             -> трансформация системы координат
- Параметры:  serial_port, baudrate, wheel_radius, base_radius, publish_tf
"""

import math
import sys

# Проверка наличия rclpy
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformBroadcaster
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from termit_api import TermitRobotAPI, RobotConfig, HoldMode, OdometryData


if HAS_ROS2:
    class TermitDriverNode(Node):
        def __init__(self):
            super().__init__('termit_driver_node')

            # Объявление параметров ноды
            self.declare_parameter('serial_port', '/dev/ttyUSB0')
            self.declare_parameter('baudrate', 115200)
            self.declare_parameter('wheel_radius', 0.030)
            self.declare_parameter('base_radius', 0.122)
            self.declare_parameter('publish_tf', True)
            self.declare_parameter('odom_frame_id', 'odom')
            self.declare_parameter('base_frame_id', 'base_link')

            port = self.get_parameter('serial_port').get_parameter_value().string_value
            baud = self.get_parameter('baudrate').get_parameter_value().integer_value
            w_r = self.get_parameter('wheel_radius').get_parameter_value().double_value
            b_r = self.get_parameter('base_radius').get_parameter_value().double_value
            self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
            self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
            self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value

            # Инициализация робота
            self.config = RobotConfig(
                wheel_radius=w_r,
                base_radius=b_r,
                watchdog_timeout_ms=500
            )
            self.robot = TermitRobotAPI(self.config)

            self.get_logger().info(f"Подключение к Termit ESP32 на порту: {port}...")
            try:
                self.robot.connect(port=port, baudrate=baud)
                self.get_logger().info("✅ Успешно подключено к платформе!")
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка подключения к контроллеру: {e}")
                sys.exit(1)

            # ROS 2 паблишеры и подписчики
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
            self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            
            if self.publish_tf:
                self.tf_broadcaster = TransformBroadcaster(self)

            # Подписка на внутренний поток одометрии (20 Гц)
            self.robot.add_odometry_callback(self.odometry_callback)

        def cmd_vel_callback(self, msg: Twist):
            """Обработка команд скорости от навигации Nav2 / телеоператора."""
            # msg.linear.x  - стрейф вправо/влево
            # msg.linear.y  - движение вперед/назад
            # msg.angular.z - вращение вокруг оси Z
            self.robot.drive(vx=msg.linear.x, vy=msg.linear.y, omega=msg.angular.z)

        def odometry_callback(self, odom: OdometryData):
            """Публикация одометрии и TF в стек ROS 2."""
            now_msg = self.get_clock().now().to_msg()

            # 1. Формирование nav_msgs/Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = now_msg
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame

            # Поза (Pose)
            odom_msg.pose.pose.position.x = odom.x
            odom_msg.pose.pose.position.y = odom.y
            odom_msg.pose.pose.position.z = 0.0

            # Кватернион из угла yaw (theta)
            qz = math.sin(odom.theta / 2.0)
            qw = math.cos(odom.theta / 2.0)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw

            # Скорость (Twist)
            odom_msg.twist.twist.linear.x = odom.vx
            odom_msg.twist.twist.linear.y = odom.vy
            odom_msg.twist.twist.angular.z = odom.omega

            self.odom_pub.publish(odom_msg)

            # 2. Публикация TF трансформации odom -> base_link
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = now_msg
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.base_frame

                t.transform.translation.x = odom.x
                t.transform.translation.y = odom.y
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw

                self.tf_broadcaster.sendTransform(t)

        def destroy_node(self):
            self.get_logger().info("Остановка робота и отключение...")
            self.robot.disconnect()
            super().destroy_node()

    def main(args=None):
        rclpy.init(args=args)
        node = TermitDriverNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()

else:
    # Запуск без ROS 2 (информационная заглушка)
    def main():
        print("Этот модуль предназначен для ROS 2 на Raspberry Pi.")
        print("Для обычного управления используйте termit_api.py или example_api_usage.py!")

    if __name__ == '__main__':
        main()
