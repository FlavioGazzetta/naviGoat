import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.dirname(__file__)
    simulation_dir = os.path.dirname(launch_dir)
    nav2_params_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            '..', '..',
            'autonomy', 'navigation', 'config',
            'nav2_sim_params.yaml'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localisation']),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapping.launch.py')),
            launch_arguments={
                'mode': LaunchConfiguration('mode'),
                'world': LaunchConfiguration('world')
            }.items(),
        ),

        # nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')), # I ll change later to custom
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params_path,
                'use_composition': 'False',
            }.items()
        ),
      
      
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(simulation_dir, 'config', 'mapping.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
