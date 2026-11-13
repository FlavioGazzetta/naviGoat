import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_rtabmap(context):
    slam_dir = os.path.dirname(os.path.dirname(__file__))
    database_path = LaunchConfiguration('database_path').perform(context)
    mode = LaunchConfiguration('mode').perform(context)

    if mode == 'localisation' and not os.path.isfile(database_path):
        raise RuntimeError(f'No saved map at {database_path}')

    parameters = [
        os.path.join(slam_dir, 'config', 'rtabmap.yaml'),
        LaunchConfiguration('config_file'),
        {
            'database_path': database_path,
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'odom_frame_id': ParameterValue(LaunchConfiguration('odom_frame_id'), value_type=str),
        },
    ]

    if mode == 'localisation':
        parameters.append({
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'initial_pose': '0 0 0 0 0 0',
        })

    remappings = [
        ('scan_cloud', LaunchConfiguration('scan_cloud_topic')),
        ('odom', LaunchConfiguration('odom_topic')),
    ]
    imu_topic = LaunchConfiguration('imu_topic').perform(context)

    if imu_topic:
        remappings.append(('imu', imu_topic))

    return [Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=parameters,
        remappings=remappings,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localisation']),
        DeclareLaunchArgument('database_path'),
        DeclareLaunchArgument('config_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/unilidar/cloud'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value=''),
        DeclareLaunchArgument('odom_frame_id', default_value=''),
        OpaqueFunction(function=launch_rtabmap),
    ])
