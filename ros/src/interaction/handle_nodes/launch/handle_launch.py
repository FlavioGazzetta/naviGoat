from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='handle_nodes',
            executable='serial_bridge'
        )
    ])
