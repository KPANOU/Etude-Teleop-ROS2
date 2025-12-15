#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio   # Bibliothèque GPIO officielle Linux (déjà installée sur Raspberry Pi)

CHIP = 0              # gpiochip0 sur Raspberry Pi
PIN = 24              # GPIO BCM à utiliser pour ton relais

# Ouverture du chip GPIO
HANDLE = lgpio.gpiochip_open(CHIP)

class GPIODirection(Node):
    def __init__(self):
        super().__init__('gpio_direction')

        # Configurer le GPIO en sortie
        lgpio.gpio_claim_output(HANDLE, PIN)

        # Subscriber
        self.subscription = self.create_subscription(
            Bool,
            '/direction_clean',
            self.direction_callback,
            10)

        self.get_logger().info(f"GPIO Direction node started. Using GPIO {PIN}.")

    def direction_callback(self, msg):
        # HIGH si True, LOW si False
        value = 1 if msg.data else 0
        lgpio.gpio_write(HANDLE, PIN, value)
        self.get_logger().info(f"GPIO {PIN} set to: {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    node = GPIODirection()
    rclpy.spin(node)

    # Nettoyage GPIO
    lgpio.gpio_free(HANDLE, PIN)
    lgpio.gpiochip_close(HANDLE)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

