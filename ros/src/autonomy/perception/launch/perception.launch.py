import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    perception_dir = os.path.dirname(os.path.dirname(__file__))
    camera_launch = os.path.join(perception_dir, '..', '..', 'hardware', 'camera', 'launch', 'camera.launch.py')

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
        DeclareLaunchArgument('camera', default_value='false'),
        # The Femto Bolt node from hardware/camera, when nothing else runs the camera
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.normpath(camera_launch)), condition=IfCondition(LaunchConfiguration('camera'))),
        description,
        # Start detection once Qwen has loaded, or on the Jetson YOLO pushes Qwen onto the CPU
        RegisterEventHandler(OnProcessIO(target_action=description, on_stdout=lambda event: [detection] if b'scene_description ready' in event.text else None)),
    ])
