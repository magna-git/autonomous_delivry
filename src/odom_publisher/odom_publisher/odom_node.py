import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
import serial
import math
import re

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialisation du port série (modifie si nécessaire)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        # Initialisation du chemin
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode(errors='ignore').strip()
            match = re.search(r'X:\s*(-?\d+\.?\d*)\s*\|\s*Y:\s*(-?\d+\.?\d*)\s*\|\s*θ:\s*(-?\d+\.?\d*)', line)
            if match:
                x = float(match.group(1)) / 1000.0
                y = float(match.group(2)) / 1000.0
                theta_deg = float(match.group(3))
                theta = math.radians(theta_deg)

                # Publier sur /odom
                msg = Odometry()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_link'
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y
                msg.pose.pose.orientation.z = math.sin(theta / 2.0)
                msg.pose.pose.orientation.w = math.cos(theta / 2.0)
                self.publisher_.publish(msg)

                # Publier la transformation TF
                t = TransformStamped()
                t.header = msg.header
                t.child_frame_id = msg.child_frame_id
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.rotation.z = msg.pose.pose.orientation.z
                t.transform.rotation.w = msg.pose.pose.orientation.w
                self.broadcaster.sendTransform(t)

                # Mettre à jour le chemin /path
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose = msg.pose.pose
                self.path.header.stamp = msg.header.stamp
                self.path.poses.append(pose)
                self.path_publisher.publish(self.path)

        except Exception as e:
            self.get_logger().warn(f"Erreur série : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
