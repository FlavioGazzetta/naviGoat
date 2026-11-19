import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
            'use_rgbd': LaunchConfiguration('use_rgbd'),
            'scan_cloud_topic': PythonExpression(["'/unilidar/cloud_static' if '", LaunchConfiguration('filter'), "' == 'object' else '/unilidar/cloud/deskewed' if '", LaunchConfiguration('filter'), "' == 'deskew' else '/unilidar/cloud'"]),
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
        DeclareLaunchArgument('use_rgbd', default_value='true'),
        DeclareLaunchArgument('filter', default_value='none', choices=['object', 'deskew', 'none']),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(simulation_dir, 'config', 'mapping.rviz'),
                              description='RViz config, e.g. autonomy/navigation/config/robot.rviz (the real robot one)'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'use_rgbd': LaunchConfiguration('use_rgbd')}.items(),
        ),
        Node(
            package='rtabmap_util',
            executable='lidar_deskewing',
            name='lidar_deskewing',
            parameters=[{'fixed_frame_id': 'pioneer3at/odom', 'use_sim_time': True}],
            remappings=[('input_cloud', '/unilidar/cloud')],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('filter'), "' != 'none'"])),
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'dynamic-object-removal-realtime',
                '--pointcloud-topic', '/unilidar/cloud/deskewed',
                '--output-topic', '/unilidar/cloud_static',
                '--algorithm', 'range',
                '--fixed-frame', 'pioneer3at/odom',
                '--range-window', '3',
            ],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('filter'), "' == 'object'"])),
            output='screen',
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
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
