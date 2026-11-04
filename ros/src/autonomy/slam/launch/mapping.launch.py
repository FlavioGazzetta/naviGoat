import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_dir = os.path.dirname(os.path.dirname(__file__))

    return LaunchDescription([
        DeclareLaunchArgument('pioneer_port', default_value='/dev/pioneer'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/unilidar'),
        DeclareLaunchArgument('lidar_x', default_value='0'),
        DeclareLaunchArgument('lidar_y', default_value='0'),
        DeclareLaunchArgument('lidar_z', default_value='0.3'),
        DeclareLaunchArgument('lidar_roll', default_value='0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0'),
        DeclareLaunchArgument('database_path', default_value=os.path.expanduser('~/.ros/navigoat_hardware.db')),
        DeclareLaunchArgument('odom_frame_id', default_value=''),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('filter_dynamic_objects', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('p2os_bringup'), 'launch', 'p2os_driver_launch.py'])),
            launch_arguments={'port': LaunchConfiguration('pioneer_port')}.items(),
            condition=IfCondition(LaunchConfiguration('start_base')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('p2os_urdf'), 'launch', 'pioneer3at_urdf_launch.py'])),
        ),
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            parameters=[{'port': LaunchConfiguration('lidar_port'), 'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_mount',
            arguments=[
                '--x', LaunchConfiguration('lidar_x'), '--y', LaunchConfiguration('lidar_y'), '--z', LaunchConfiguration('lidar_z'),
                '--roll', LaunchConfiguration('lidar_roll'), '--pitch', LaunchConfiguration('lidar_pitch'), '--yaw', LaunchConfiguration('lidar_yaw'),
                '--frame-id', 'top_plate', '--child-frame-id', 'unilidar_lidar',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_imu_mount',
            arguments=[
                '--x', '-0.007698', '--y', '-0.014655', '--z', '0.00667',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'unilidar_lidar', '--child-frame-id', 'unilidar_imu',
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_util',
            executable='lidar_deskewing',
            name='lidar_deskewing',
            parameters=[{'fixed_frame_id': 'odom'}],
            remappings=[('input_cloud', '/unilidar/cloud')],
            condition=IfCondition(LaunchConfiguration('filter_dynamic_objects')),
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'dynamic-object-removal-realtime',
                '--pointcloud-topic', '/unilidar/cloud/deskewed',
                '--output-topic', '/unilidar/cloud_static',
                '--algorithm', 'range',
                '--fixed-frame', 'odom',
                '--range-window', '3',
            ],
            condition=IfCondition(LaunchConfiguration('filter_dynamic_objects')),
            output='screen',
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[os.path.join(slam_dir, 'config', 'rtabmap.yaml'), {'database_path': LaunchConfiguration('database_path'), 'odom_frame_id': ParameterValue(LaunchConfiguration('odom_frame_id'), value_type=str)}],
            remappings=[('scan_cloud', PythonExpression(["'/unilidar/cloud_static' if '", LaunchConfiguration('filter_dynamic_objects'), "'.lower() in ('true', '1') else '/unilidar/cloud'"])), ('imu', '/unilidar/imu'), ('odom', '/odom')],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(slam_dir, 'config', 'mapping.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
