from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws/src/my_robot_description/urdf/simple_robot.urdf'
    )

    rviz_config_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws/src/my_robot_description/rviz/robot_mapping.rviz'
    )

    return LaunchDescription([
        # 🟦 URDF du robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # 🟩 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        # 🟨 Odométrie réelle (Arduino → ROS2)
        Node(
            package='odom_publisher',
            executable='odom_pub',
            name='odom_pub',
            output='screen'
        ),

        # 🟥 SLAM Toolbox (carte)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # 🟪 LiDAR YDLIDAR
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',  # adapte si besoin
                'frame_id': 'laser',
                'baudrate': 128000
            }]
        ),
    ])
