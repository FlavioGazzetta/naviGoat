import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.dirname(__file__)
    simulation_dir = os.path.dirname(launch_dir)

    

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localisation']),
        DeclareLaunchArgument("world", default_value="indoors.sdf", description="Lets us choose which world we wanna be in"),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapping.launch.py')),
            launch_arguments={
                'mode': LaunchConfiguration('mode'),
                'world': LaunchConfiguration('world')
            }.items(),
        ),
      
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(simulation_dir, 'config', 'mapping.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
