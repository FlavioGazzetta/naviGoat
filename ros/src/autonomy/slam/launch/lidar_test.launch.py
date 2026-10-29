import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.dirname(__file__)

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/unilidar'),
        DeclareLaunchArgument('database_path', default_value=os.path.expanduser('~/.ros/navigoat_lidar_test.db')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('odometry', default_value='stationary', choices=['stationary', 'icp']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapping.launch.py')),
            launch_arguments={
                'start_base': 'false',
                'lidar_port': LaunchConfiguration('lidar_port'),
                'database_path': LaunchConfiguration('database_path'),
                'rviz': LaunchConfiguration('rviz'),
                'odom_frame_id': 'odom',
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='stationary_odometry',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
            condition=LaunchConfigurationEquals('odometry', 'stationary'),
            output='screen',
        ),
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            parameters=[os.path.join(os.path.dirname(launch_dir), 'config', 'icp.yaml')],
            remappings=[('scan_cloud', '/unilidar/cloud'), ('imu', '/unilidar/imu'), ('odom', '/odom')],
            condition=LaunchConfigurationEquals('odometry', 'icp'),
            output='screen',
        ),
    ])
