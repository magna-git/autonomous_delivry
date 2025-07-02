import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import math
import re

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Connexion série
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            print(f"🔁 Ligne reçue : {line}")  # DEBUG ajouté ici

            match = re.search(r'X:\s*(-?\d+\.?\d*)\s*\|\s*Y:\s*(-?\d+\.?\d*)\s*\|\s*θ:\s*(-?\d+\.?\d*)', line)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                angle_deg = float(match.group(3))
                theta = math.radians(angle_deg)

                msg = Odometry()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_link'

                msg.pose.pose.position.x = x / 1000.0
                msg.pose.pose.position.y = y / 1000.0
                msg.pose.pose.position.z = 0.0

                qz = math.sin(theta / 2.0)
                qw = math.cos(theta / 2.0)
                msg.pose.pose.orientation.z = qz
                msg.pose.pose.orientation.w = qw

                self.publisher_.publish(msg)

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = x / 1000.0
                t.transform.translation.y = y / 1000.0
                t.transform.translation.z = 0.0
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"Erreur lecture série: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
