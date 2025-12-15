#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        # --- Souscription au topic /accelerator_clean ---
        self.subscription = self.create_subscription(
            Float32,
            '/accelerator_clean',
            self.listener_callback,
            10)
        self.subscription

        # --- Connexion série vers ESP32 ---
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            time.sleep(2)  # laisse le temps à l’ESP32 de redémarrer
            self.get_logger().info("✅ Serial connected to ESP32 on /dev/ttyUSB0 at 115200 baud")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to open serial port: {e}")
            self.ser = None

        self.last_value = 0.0

    def listener_callback(self, msg):
        accel = msg.data  # valeur entre 0 et 255 venant de /accelerator_clean
        accel = max(0.0, min(255.0, accel))  # borne sécurité
        self.last_value = accel

        self.get_logger().info(f" Accelerator command: {accel:.1f}")

        # --- Envoi vers l’ESP32 ---
        if self.ser:
            try:
                data = f"{accel:.1f}\n"
                self.ser.write(data.encode())
            except Exception as e:
                self.get_logger().error(f"Erreur d’envoi série : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)

    if node.ser:
        node.ser.close()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

