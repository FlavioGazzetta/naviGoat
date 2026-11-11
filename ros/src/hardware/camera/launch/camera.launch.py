import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    camera_dir = os.path.dirname(os.path.dirname(__file__))
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', os.path.join(camera_dir, 'camera.py')], output='screen'),
    ])
