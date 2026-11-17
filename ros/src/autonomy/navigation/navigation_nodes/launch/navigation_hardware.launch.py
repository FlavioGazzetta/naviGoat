"""Starts Pioneer hardware, mapping or localisation, Nav2 and navigation feedback."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


RESOURCES_DIR = get_package_share_directory('navigation_resources')
NAVIGATION_DIR = os.path.join(RESOURCES_DIR, 'navigation')
LAUNCH_DIR = os.path.join(get_package_share_directory('navigation_nodes'), 'launch')
SLAM_DIR = os.path.join(RESOURCES_DIR, 'autonomy', 'slam')


def launch_hardware(context):
    mode = LaunchConfiguration('mode').perform(context)
    database_path = os.path.abspath(os.path.expanduser(LaunchConfiguration('database_path').perform(context)))
    if mode == 'localisation' and not os.path.isfile(database_path):
        raise RuntimeError(f'No saved map at {database_path}. Use mode:=mapping to create it.')

    places_file = LaunchConfiguration('places_file').perform(context)
    if not places_file:
        places_file = os.path.splitext(database_path)[0] + '_places.yaml'

    hardware_arguments = {name: LaunchConfiguration(name) for name in (
        'pioneer_port', 'lidar_port', 'lidar_x', 'lidar_y', 'lidar_z', 'lidar_roll', 'lidar_pitch', 'lidar_yaw',
        'start_base', 'enable_motors', 'start_lidar', 'start_camera', 'use_rgbd', 'filter_dynamic_objects', 'initial_pose',
    )}
    hardware_arguments.update({'mode': mode, 'database_path': database_path, 'odom_frame_id': 'odom', 'rviz': 'false'})

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SLAM_DIR, 'launch', 'mapping.launch.py')),
            launch_arguments=hardware_arguments.items(),
        ),
        Node(package='navigation_nodes', executable='cloud_filter', output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_composition': 'False',
                'params_file': LaunchConfiguration('params_file'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'navigation.launch.py')),
            launch_arguments={'places_file': places_file, 'use_sim_time': 'false'}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(NAVIGATION_DIR, 'config', 'navigation_hardware.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
        Node(
            package='handle_nodes',
            executable='serial_bridge',
            parameters=[{'port': ParameterValue(LaunchConfiguration('handle_port'), value_type=str)}],
            remappings=[('/handle/vibration', LaunchConfiguration('handle_feedback_topic'))],
            condition=IfCondition(LaunchConfiguration('start_handle')),
            output='screen',
        ),
        Node(
            package='wristband_nodes',
            executable='ble_bridge',
            remappings=[('/wristband/vibration', LaunchConfiguration('wristband_feedback_topic'))],
            condition=IfCondition(LaunchConfiguration('start_wristband')),
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='localisation', choices=['mapping', 'localisation']),
        DeclareLaunchArgument('database_path', description='Hardware RTAB-Map database to create or load'),
        DeclareLaunchArgument('places_file', default_value='', description='Defaults to <database stem>_places.yaml'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(NAVIGATION_DIR, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('initial_pose', default_value='0 0 0 0 0 0', description='Initial map pose: x y z roll pitch yaw'),
        DeclareLaunchArgument('pioneer_port', default_value='/dev/pioneer'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/unilidar'),
        DeclareLaunchArgument('lidar_x', default_value='0'),
        DeclareLaunchArgument('lidar_y', default_value='0'),
        DeclareLaunchArgument('lidar_z', default_value='0.225'),
        DeclareLaunchArgument('lidar_roll', default_value='0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('enable_motors', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('use_rgbd', default_value='false'),
        DeclareLaunchArgument('filter_dynamic_objects', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_handle', default_value='false'),
        DeclareLaunchArgument('start_wristband', default_value='false'),
        DeclareLaunchArgument('handle_port', default_value='/dev/handle'),
        DeclareLaunchArgument('handle_feedback_topic', default_value='/navigation/turn_feedback'),
        DeclareLaunchArgument('wristband_feedback_topic', default_value='/navigation/proximity_feedback'),
        OpaqueFunction(function=launch_hardware),
    ])
