#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BrakeSubscriber(Node):
    def __init__(self):
        super().__init__('brake_subscriber')

        # Souscription à /brake
        self.subscription = self.create_subscription(
            Float32,
            '/brake',
            self.listener_callback,
            10)

        # Publication vers /brake_clean
        self.publisher_clean = self.create_publisher(Float32, '/brake_clean', 10)

        self.get_logger().info("brake_subscriber started")

    def listener_callback(self, msg):
        raw = msg.data

        # Threshold simple (0 ou 1)
        clean = 1.0 if raw > 0.5 else 0.0

        # Publication
        msg_out = Float32()
        msg_out.data = clean
        self.publisher_clean.publish(msg_out)

        self.get_logger().info(f"Brake raw={raw:.3f} → clean={clean}")

def main(args=None):
    rclpy.init(args=args)
    node = BrakeSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

