from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'scan_topic': '/scan',
                'slam_toolbox': {
                    'scan_qos': 'sensor_data'
                }
            }]
        )
    ])
