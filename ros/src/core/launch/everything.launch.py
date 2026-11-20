"""Everything on the real robot from one launch file: the base (motors on), the lidar, RTAB-Map, Nav2 and the
navigation nodes (navigation_robot.launch.py), Pete (pete.launch.py: camera, scene description, object detection, the
middleman, and with their flags the voice, the people tracker, person following and GPS), and the handle and wristband.

Run it in the container, with the workspace built and sourced (it needs p2os_bringup, the lidar driver and Nav2):

    source /workspace/install/setup.bash
    ros2 launch /workspace/ros/src/core/launch/everything.launch.py database_path:=/workspace/ros/src/bringup/maps/lab.db \
        mode:=localisation voice:=true

On the Jetson, start_pete.sh does the headset, Whisper, the container and Ollama first, then this file:

    PETE_LAUNCH=core/launch/everything.launch.py ros/src/core/start_pete.sh voice:=true mode:=localisation \
        database_path:=/workspace/ros/src/bringup/maps/lab.db

Every argument of the two launch files works here under its own name (ros2 launch <this file> --show-args lists them):
database_path, mode, teleop, filter, use_lidar, use_depth_camera ... from navigation; voice, people, follow,
follow_drive, gps, local_llm ... from Pete.

Three things are started once although both launch files could start them, and this file decides who does:
  - the camera: Pete's, unless use_depth_camera:=true, where navigation's is used because it also publishes the point cloud
  - the YOLO server and the people tracker: always Pete's. For social navigation use people:=true, not social:=true
  - the base: navigation's (Pete has none)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))  # ros/src
NAVIGATION = os.path.join(SRC_DIR, 'autonomy', 'navigation', 'navigation_nodes', 'launch', 'navigation_robot.launch.py')
PETE = os.path.join(SRC_DIR, 'core', 'launch', 'pete.launch.py')


def include(path, **arguments):
    """Include a launch file, setting these arguments for it. Not in a scope of its own: perception starts YOLO later,
    when scene description is ready, and by then a scope's arguments (search_time) are gone and the launch stops. The
    two launch files share no argument names, so they do not get in each other's way."""
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())


def generate_launch_description():
    use_depth_camera = LaunchConfiguration('use_depth_camera')
    devices = IfCondition(LaunchConfiguration('devices'))
    package_launch = lambda package, name: IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', name])), condition=devices)

    return LaunchDescription([
        DeclareLaunchArgument('database_path', description='RTAB-Map database: a new file to make a map, an existing one to continue or navigate on it'),
        DeclareLaunchArgument('use_depth_camera', default_value='false', choices=['true', 'false'],
                              description='navigation also uses the Femto (then navigation starts the camera, not Pete)'),
        DeclareLaunchArgument('camera', default_value=NotSubstitution(use_depth_camera), choices=['true', 'false'],
                              description="Pete starts the camera; false when navigation does (use_depth_camera:=true) or there is none"),
        DeclareLaunchArgument('devices', default_value='true', choices=['true', 'false'],
                              description='also start the handle and wristband bridges'),
        LogInfo(msg='[everything] social:=true is ignored here: use people:=true, Pete starts the YOLO server and the people tracker.',
                condition=IfCondition(LaunchConfiguration('social', default='false'))),

        # base, lidar, RTAB-Map, Nav2, navigation nodes. Its own people tracker stays off (Pete's people:=true is the
        # same tracker, on the YOLO server Pete already runs), and it starts the camera only when it needs the point cloud
        # (database_path has no default there, so an include must hand it over by name)
        include(NAVIGATION, database_path=LaunchConfiguration('database_path'), social='false', start_femto=use_depth_camera),
        # Pete, with the camera unless navigation started it
        include(PETE, camera=LaunchConfiguration('camera')),
        # vibrations: navigation's turn and proximity feedback go to these two
        package_launch('handle_nodes', 'handle_launch.py'),
        package_launch('wristband_nodes', 'wristband_launch.py'),
    ])
