from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=['--ros-args', '--qos-profile-overrides-path', '/home/khadija/ros2_ws/rviz_config/qos_override.yaml']
        )
    ])

