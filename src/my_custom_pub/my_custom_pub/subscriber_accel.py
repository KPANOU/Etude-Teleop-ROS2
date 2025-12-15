#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class AcceleratorSubscriber(Node):
    def __init__(self):
        super().__init__('accelerator_subscriber')
        
        # 🔸 Souscription au topic brut
        self.subscription = self.create_subscription(
            Float32,
            '/accelerator',
            self.listener_callback,
            10)
        
        # 🔹 Publication du signal "nettoyé" / mappé
        self.publisher_ = self.create_publisher(Float32, '/accelerator_clean', 10)
        self.get_logger().info("✅ Node 'accelerator_subscriber' started (listening to /accelerator, publishing /accelerator_clean)")

    def listener_callback(self, msg):
        raw_value = msg.data  # Valeur brute (0.0 → 0.9999)
        raw_value = max(0.0, min(1.0, raw_value))  # Sécurité bornes

        # 🔁 Mapping linéaire vers 0–255
        mapped_value = int(raw_value * 255)

        # Publication ROS
        clean_msg = Float32()
        clean_msg.data = float(mapped_value)
        self.publisher_.publish(clean_msg)

        # Log
        self.get_logger().info(f" Accelerator: raw={raw_value:.3f} → mapped={mapped_value}")

def main(args=None):
    rclpy.init(args=args)
    node = AcceleratorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

