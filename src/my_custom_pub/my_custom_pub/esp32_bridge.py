#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

SERIAL_PORT = "/dev/ttyUSB5"   # Port USB
BAUDRATE = 115200              # même baudrate que notre ESP32

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__("esp32_bridge")

        # Ouverture du port série
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            self.get_logger().info(f" Serial link opened on {SERIAL_PORT} @ {BAUDRATE}")
        except Exception as e:
            self.get_logger().error(f" Cannot open serial port {SERIAL_PORT}: {e}")
            raise e

        # Souscription
        self.subscription = self.create_subscription(
            Float32,
            "/accelerator_clean",
            self.accel_callback,
            10
        )

        self.get_logger().info(" ESP32 Bridge started (listening to /accelerator_clean)")

    def accel_callback(self, msg):
        value = int(msg.data)  # 0–255
        value = max(0, min(255, value))

        # 🔹 Protocole simple : ACC:<value>\n
        tx_string = f"ACC:{value}\n"

        try:
            self.serial.write(tx_string.encode())
            self.get_logger().info(f" TX → ESP32 : {tx_string.strip()}")
        except Exception as e:
            self.get_logger().error(f" Serial write error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

