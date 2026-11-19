"""Starts the Femto Bolt the same way as social_perception/jetson_pipeline.sh, so Pete and the people tracker share one
camera setup: the Orbbec ROS 2 driver, then depth registration (depth aligned to colour) as a separate node.

    driver:=orbbec   (default) the Orbbec driver + depth_image_proc; needs the SOCIAL=true image
    driver:=sdk      the old camera.py, which reads the camera through the Orbbec SDK (PETE=true image)
    enable_point_cloud:=true publishes /camera/depth/points with the Orbbec driver

Either way the topics are /camera/color/image_raw, /camera/color/camera_info and /camera/depth_registered/image_rect.
Only one program can open the camera: do not run this together with jetson_pipeline.sh.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    camera_dir = os.path.dirname(os.path.dirname(__file__))
    is_driver = lambda name: IfCondition(PythonExpression(["'", LaunchConfiguration('driver'), "' == '", name, "'"]))

    orbbec = ExecuteProcess(
        cmd=['ros2', 'launch', 'orbbec_camera', 'femto_bolt.launch.py', 'depth_registration:=false',
             ['enable_point_cloud:=', LaunchConfiguration('enable_point_cloud')]],
        output='screen', condition=is_driver('orbbec'))
    # ROS 2 finds publishers and subscribers as they appear, so this can start before the first camera frames arrive
    registration = ExecuteProcess(
        cmd=['ros2', 'run', 'depth_image_proc', 'register_node', '--ros-args',
             '-r', 'depth/image_rect:=/camera/depth/image_raw',
             '-r', 'depth/camera_info:=/camera/depth/camera_info',
             '-r', 'rgb/camera_info:=/camera/color/camera_info',
             '-r', 'depth_registered/image_rect:=/camera/depth_registered/image_rect'],
        output='screen', condition=is_driver('orbbec'))
    sdk = ExecuteProcess(cmd=['python3', os.path.join(camera_dir, 'camera.py')], output='screen', condition=is_driver('sdk'))

    # As in the pipeline: if a camera component exits, stop everything, so nothing keeps running half broken
    stop_together = [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=[
        LogInfo(msg=f'[camera] {name} exited; stopping.'), EmitEvent(event=Shutdown(reason=f'{name} exited'))]))
        for process, name in ((orbbec, 'The Orbbec driver'), (registration, 'Depth registration'), (sdk, 'camera.py'))]

    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='orbbec'),
        DeclareLaunchArgument('enable_point_cloud', default_value='false', choices=['true', 'false']),
        LogInfo(msg='[camera] Starting Orbbec Femto Bolt driver...', condition=is_driver('orbbec')),
        orbbec,
        LogInfo(msg='[camera] Starting external depth registration...', condition=is_driver('orbbec')),
        registration,
        sdk,
        *stop_together,
    ])
