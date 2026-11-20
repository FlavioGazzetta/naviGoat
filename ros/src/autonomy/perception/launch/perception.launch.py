import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

LOCAL_YOLO = 'http://127.0.0.1:8767'


def generate_launch_description():
    perception_dir = os.path.dirname(os.path.dirname(__file__))
    camera_launch = os.path.join(perception_dir, '..', '..', 'hardware', 'camera', 'launch', 'camera.launch.py')

    description = ExecuteProcess(
        cmd=['python3', os.path.join(perception_dir, 'scene_description.py')],
        output='screen',
    )
    # YOLO, loaded once: object detection and the people tracker both ask this server. Not started when yolo_url names
    # a server that already runs elsewhere (the Jetson's, started by navigation with social:=true)
    yolo_url = LaunchConfiguration('yolo_url')
    yolo = ExecuteProcess(cmd=['python3', '-u', os.path.join(perception_dir, 'yolo_server.py')], output='screen',
                          condition=IfCondition(PythonExpression(["'", yolo_url, f"' == '{LOCAL_YOLO}'"])))
    detection = ExecuteProcess(
        cmd=['python3', os.path.join(perception_dir, 'object_detection.py'), '--ros-args', '-p', ['search_time:=', LaunchConfiguration('search_time')],
             '-p', ['yolo_url:=', yolo_url]],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('search_time', default_value='5.0'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('yolo_url', default_value=LOCAL_YOLO),  # another machine's server: http://192.168.55.1:8767
        # The Femto Bolt node from hardware/camera, when nothing else runs the camera
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.normpath(camera_launch)), condition=IfCondition(LaunchConfiguration('camera'))),
        description,
        # Start YOLO once the local language model has loaded, or on the Jetson YOLO pushes it onto the CPU
        RegisterEventHandler(OnProcessIO(target_action=description, on_stdout=lambda event: [yolo, detection] if b'scene_description ready' in event.text else None)),
    ])
