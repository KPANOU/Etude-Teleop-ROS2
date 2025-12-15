import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SteeringCleaner(Node):
    def __init__(self):
        super().__init__('steering_cleaner')
        self.subscription = self.create_subscription(
            Float32,
            '/steering',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Float32, '/steering_clean', 10)

        self.get_logger().info("✅ Node 'steering_cleaner' up, listening to /steering, publishing to /steering_clean")

    def listener_callback(self, msg):
        raw_value = msg.data
        if not self.is_data_valid(raw_value):
            self.get_logger().warn('Invalid steering value received, skipping...')
            return

        # Mapping -3, 0, 3 -> -450, 0, 450
        mapped_value = self.map_value(raw_value, -3, 3, -450, 450)

        self.get_logger().info(f" Raw steering: {raw_value:.3f} -> Mapped to: {mapped_value:.1f}°")
        self.publisher.publish(Float32(data=mapped_value))

    def is_data_valid(self, data):
        return data is not None and data == data  # valid & not NaN

    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    node = SteeringCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

