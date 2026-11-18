"""The whole navigation pipeline on the real robot: the base (with the motors turned on), the lidar and RTAB-Map
(from slam/launch/mapping.launch.py), Nav2, and the navigation nodes.

Map first (mode:=mapping, the default), then navigate on the saved map:

    ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_robot.launch.py \
        database_path:=/workspace/ros/src/bringup/maps/lab.db
    ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_robot.launch.py \
        database_path:=/workspace/ros/src/bringup/maps/lab.db mode:=localisation

The places are kept next to the map: lab.db -> lab_places.yaml. Only save places in localisation mode, on a finished
map: in mapping mode the map can still shift.

If the base or the lidar don't connect (SYNC failed, Setup() failed), the container's /dev/ttyUSB numbers are probably
swapped: add pioneer_port:=/dev/unilidar lidar_port:=/dev/pioneer, or recreate the container with the
/dev/serial/by-id names (see slam/README.md).
"""

# everything for the real robot in one command: the base from bringup (so the motors get turned on), the team's
# slam/launch/mapping.launch.py for the lidar and RTAB-Map (its own base is switched off), Nav2 with the real-robot
# params, and our nodes from navigation.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))  # navigation_nodes/launch
SRC_DIR = os.path.abspath(os.path.join(LAUNCH_DIR, '..', '..', '..', '..'))  # ros/src
NAV2_PARAMS = os.path.join(SRC_DIR, 'autonomy', 'navigation', 'config', 'nav2_params.yaml')
PERCEPTION_DIR = os.path.join(SRC_DIR, 'autonomy', 'perception')
PEOPLE_DIR = os.path.join(PERCEPTION_DIR, 'social_perception', 'femto_id_velocity_tracker')


def social_nav():
    """Only what social nav needs, with social:=true: the camera, the YOLO server and Enid's people tracker, which
    publishes /social_perception/tracks for people_node. The same three pete.launch.py people:=true starts, without
    Qwen, scene description, object detection or the middleman. Needs the PETE=true image, the GPU and the camera."""
    social = IfCondition(LaunchConfiguration('social'))
    return [
        # its own `ros2 launch`: camera.launch.py shuts down everything around it if the camera stops, and that should
        # stop the camera side only, not mapping and Nav2
        ExecuteProcess(cmd=['ros2', 'launch', os.path.join(SRC_DIR, 'hardware', 'camera', 'launch', 'camera.launch.py')],
                       output='screen', condition=social),
        # YOLO, loaded once, asked over HTTP by the tracker (http://127.0.0.1:8767)
        ExecuteProcess(cmd=['python3', '-u', os.path.join(PERCEPTION_DIR, 'yolo_server.py')], output='screen', condition=social),
        # the tracker, with the same options as in pete.launch.py. Its own Python (the image's SOCIAL=true layer), run from
        # its folder so the YOLO weights saved there are reused
        ExecuteProcess(cmd=['/opt/people_env/bin/python3', '-u', 'femto_track_people.py', '--yolo-url', 'http://127.0.0.1:8767',
                            '--imgsz', '320', '--conf', '0.40', '--max-age', '1.0', '--no-plot'],
                       cwd=PEOPLE_DIR, output='screen', condition=social),
    ]


def launch_navigation_nodes(context):
    # the places file goes next to the map: /x/lab.db -> /x/lab_places.yaml. launch arguments only turn into text
    # inside an OpaqueFunction, that's why this is a separate function
    database = LaunchConfiguration('database_path').perform(context)
    places = LaunchConfiguration('places_file').perform(context) or os.path.splitext(database)[0] + '_places.yaml'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'navigation.launch.py')),
        launch_arguments={'places_file': places, 'use_sim_time': 'false'}.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('database_path', description='RTAB-Map database: a new file to make a map, an existing one to continue or navigate on it'),
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localisation']),
        DeclareLaunchArgument('places_file', default_value='', description='default: next to the map, <map>_places.yaml'),
        DeclareLaunchArgument('pioneer_port', default_value='/dev/pioneer'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/unilidar'),
        DeclareLaunchArgument('filter', default_value='object', choices=['object', 'deskew', 'none'],
                              description='object: moving people are left out of the map (the costmaps still see them)'),
        DeclareLaunchArgument('enable_motors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', description="RTAB-Map's RViz on the Jetson (heavy: better on the laptop)"),
        DeclareLaunchArgument('teleop', default_value='false', choices=['true', 'false'],
                              description='true: no Nav2, so you can map with teleop without Nav2 fighting it over /cmd_vel. '
                                          'Ctrl+C saves the map; launch again without teleop (mode:=localisation) to navigate'),
        DeclareLaunchArgument('social', default_value='false', choices=['true', 'false'],
                              description='true: social nav with real people (camera, YOLO server, people tracker). Not '
                                          'together with pete.launch.py people:=true, which starts the same ones'),

        # the base, with the motors turned on once it's connected
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SRC_DIR, 'bringup', 'launch', 'base.launch.py')),
            launch_arguments={'port': LaunchConfiguration('pioneer_port'),
                              'enable_motors': LaunchConfiguration('enable_motors')}.items(),
        ),
        # lidar, robot model and RTAB-Map. start_base:=false because the base is started above.
        # odom_frame_id:=odom: RTAB-Map reads the wheel odometry from TF instead of matching /odom messages to scans.
        # mode goes through to slam.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SRC_DIR, 'autonomy', 'slam', 'launch', 'mapping.launch.py')),
            launch_arguments={
                'start_base': 'false',
                'database_path': LaunchConfiguration('database_path'),
                'mode': LaunchConfiguration('mode'),
                'odom_frame_id': 'odom',
                'lidar_port': LaunchConfiguration('lidar_port'),
                'filter': LaunchConfiguration('filter'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
        # Nav2 with the real-robot params. Not with teleop:=true, so the keyboard has /cmd_vel to itself
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={'params_file': NAV2_PARAMS, 'use_sim_time': 'false', 'use_composition': 'False'}.items(),
            condition=UnlessCondition(LaunchConfiguration('teleop')),
        ),
        # and our nodes
        OpaqueFunction(function=launch_navigation_nodes),
        # with social:=true, the people for the social layer
        *social_nav(),
    ])
