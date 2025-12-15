#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class DirectionSubscriber(Node):
    def __init__(self):
        super().__init__('direction_subscriber')
        self.subscription = self.create_subscription(
            Bool,
            '/direction',
            self.listener_callback,
            10)
        self.publisher_clean = self.create_publisher(Bool, '/direction_clean', 10)

    def listener_callback(self, msg):
        clean_msg = Bool()
        clean_msg.data = msg.data
        self.publisher_clean.publish(clean_msg)
        self.get_logger().info(f"Direction clean: {clean_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = DirectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

