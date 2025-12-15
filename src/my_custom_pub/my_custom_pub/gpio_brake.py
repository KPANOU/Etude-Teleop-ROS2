#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio   # bibliothèque GPIO officielle pour Linux

CHIP = 0        # GPIO chip (toujours 0 sur Pi)
PIN = 21        # GPIO BCM à utiliser (remplace par ton pin)

HANDLE = lgpio.gpiochip_open(CHIP)

class GPIOBrake(Node):
    def __init__(self):
        super().__init__('gpio_brake')

        lgpio.gpio_claim_output(HANDLE, PIN)   # configure en sortie
        lgpio.gpio_write(HANDLE, PIN, 0)

        self.subscription = self.create_subscription(
            Float32,
            '/brake_clean',
            self.listener_callback,
            10)

        self.get_logger().info(" gpio_brake started (lgpio)")

    def listener_callback(self, msg):
        value = int(msg.data)

        lgpio.gpio_write(HANDLE, PIN, value)

        if value == 1:
            self.get_logger().info("🟥 Brake ON")
        else:
            self.get_logger().info("🟩 Brake OFF")

def main(args=None):
    rclpy.init(args=args)
    node = GPIOBrake()
    try:
        rclpy.spin(node)
    finally:
        lgpio.gpio_write(HANDLE, PIN, 0)
        lgpio.gpiochip_close(HANDLE)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

