import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_rtabmap(context):
    simulation_dir = os.path.dirname(os.path.dirname(__file__))
    database_path = os.path.join(simulation_dir, 'maps', 'rtabmap.db')
    mode = LaunchConfiguration('mode').perform(context)
    parameters = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'subscribe_scan_cloud': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_odom': True,
        'approx_sync': True,
        'qos': 1,
        'database_path': database_path,
        'Reg/Strategy': '1',
        'Grid/3D': 'true',
    }

    if mode == 'localisation':
        if not os.path.isfile(database_path):
            raise RuntimeError(f'No saved map at {database_path}')
        parameters.update({
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'initial_pose': '0 0 0 0 0 0',
        })

    return [
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[parameters],
            remappings=[('scan_cloud', '/lidar/points'), ('odom', '/odom')],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localisation']),
        OpaqueFunction(function=launch_rtabmap),
    ])
