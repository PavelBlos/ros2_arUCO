import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticTFPublisher(Node):

    def __init__(self):
        super().__init__('static_tf_publisher')

        self.br = tf2_ros.StaticTransformBroadcaster(self)

        transforms = []

        # --- map → tag_1 ---
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = "map"
        t1.child_frame_id = "tag_1"

        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 2.0

        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        transforms.append(t1)

        

        # --- camera_link → base_link ---
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = "camera_link"
        t3.child_frame_id = "base_link"

        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.0

        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0

        transforms.append(t3)

        # отправляем все сразу
        self.br.sendTransform(transforms)

        self.get_logger().info("Static TF published")

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
