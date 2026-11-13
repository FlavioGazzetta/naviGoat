import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_slam(context):
    simulation_dir = os.path.dirname(os.path.dirname(__file__))
    world_file = LaunchConfiguration('world').perform(context)
    world_name = os.path.splitext(os.path.basename(world_file))[0]
    database_path = os.path.join(simulation_dir, 'maps', f'{world_name}.db')
    slam_dir = os.path.abspath(os.path.join(simulation_dir, '..', 'autonomy', 'slam'))

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_dir, 'launch', 'slam.launch.py')),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'database_path': database_path,
            'config_file': os.path.join(slam_dir, 'config', 'rtabmap_sim.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )]


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
        DeclareLaunchArgument("world", default_value="indoor.sdf", description="Lets us choose which world we wanna be in"),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
        ),
        OpaqueFunction(function=launch_slam),

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
