import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    perception_dir = os.path.dirname(os.path.dirname(__file__))

    description = ExecuteProcess(
        cmd=['python3', os.path.join(perception_dir, 'scene_description.py')],
        output='screen',
    )
    detection = ExecuteProcess(
        cmd=['python3', os.path.join(perception_dir, 'object_detection.py'), '--ros-args', '-p', ['search_timeout:=', LaunchConfiguration('search_timeout')]],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('search_timeout', default_value='10.0'),
        description,
        # Start detection once Qwen has loaded, or on the Jetson YOLO pushes Qwen onto the CPU
        RegisterEventHandler(OnProcessIO(target_action=description, on_stdout=lambda event: [detection] if b'scene_description ready' in event.text else None)),
    ])
