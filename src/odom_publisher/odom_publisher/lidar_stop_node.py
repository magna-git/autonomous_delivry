import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import serial

class LidarStopNode(Node):
    def __init__(self):
        super().__init__('lidar_stop_node')

        # Connexion série avec Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("✅ Port série ouvert avec succès")
        except:
            self.get_logger().error("❌ Impossible d’ouvrir le port série")
            exit(1)

        # Souscription au scan avec QoS sensor_data
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

    def scan_callback(self, msg):
        ranges = msg.ranges
        # Vérifie les 20 points devant le robot (au centre du scan)
        middle = len(ranges) // 2
        front_ranges = ranges[middle - 10 : middle + 10]

        # Filtre les valeurs inf (0.0)
        front_values = [r for r in front_ranges if r > 0.05]
        if not front_values:
            return

        front_min = min(front_values)

        if front_min < 0.3:
            self.get_logger().warn(f"🛑 Obstacle à {front_min:.2f} m → STOP")
            self.serial_port.write(b"STOP\n")
        else:
            self.get_logger().info("✅ Zone dégagée → GO")
            self.serial_port.write(b"GO\n")

def main(args=None):
    rclpy.init(args=args)
    node = LidarStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
