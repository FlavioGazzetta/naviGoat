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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))  # navigation_nodes/launch
SRC_DIR = os.path.abspath(os.path.join(LAUNCH_DIR, '..', '..', '..', '..'))  # ros/src
NAV2_PARAMS = os.path.join(SRC_DIR, 'autonomy', 'navigation', 'config', 'nav2_params.yaml')


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
                              description='true: Nav2 starts switched off, so you can drive with teleop (mapping) without '
                                          'Nav2 fighting it. Switch Nav2 on later with the manage_nodes service (README)'),

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
        # Nav2 with the real-robot params. With teleop:=true it starts switched off (autostart false): its nodes exist but
        # don't drive or take goals, so teleop has /cmd_vel to itself. Switch it on with the lifecycle manager's
        # manage_nodes service (command 0 = start), and off again for teleop with command 1 (pause), 2 (resume)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={'params_file': NAV2_PARAMS, 'use_sim_time': 'false', 'use_composition': 'False',
                              'autostart': PythonExpression(["'false' if '", LaunchConfiguration('teleop'), "' == 'true' else 'true'"])}.items(),
        ),
        # and our nodes
        OpaqueFunction(function=launch_navigation_nodes),
    ])
