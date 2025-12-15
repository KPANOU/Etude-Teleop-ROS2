#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32

class SteeringToSerial(Node):
    def __init__(self):
        super().__init__('steering_to_serial')

        # 🔌 Abonnement au topic steering_clean
        self.subscription = self.create_subscription(
            Float32,
            '/steering_clean',
            self.listener_callback,
            10
        )

        # 🔌 Connexion série vers Arduino
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info('✅ Node steering_to_serial ready, serial opened on /dev/ttyACM0')
        except Exception as e:
            self.get_logger().error(f"❌ Impossible d’ouvrir le port série : {e}")
            self.ser = None

        # Timer pour envoi régulier (40 Hz = toutes les 25 ms)
        self.timer = self.create_timer(0.025, self.timer_callback)
        self.last_angle = 0.0

    def listener_callback(self, msg):
        """Callback ROS : met à jour la dernière valeur reçue"""
        self.last_angle = msg.data

    def timer_callback(self):
        """Envoi régulier sur le port série"""
        if self.ser:
            try:
                data = f"{self.last_angle:.1f}\n"
                self.ser.write(data.encode())
            except Exception as e:
                self.get_logger().error(f"Erreur Serial : {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SteeringToSerial()
    rclpy.spin(node)
    node.destroy_node()
    if node.ser:
        node.ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

