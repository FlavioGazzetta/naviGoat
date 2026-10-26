import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    p2os_share = get_package_share_directory('p2os_urdf')
    simulation_dir = os.path.dirname(os.path.dirname(__file__))
    robot_description = Command(['xacro ', os.path.join(simulation_dir, 'description', 'pioneer3at_lidar.xacro')])

    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(p2os_share)),
        ExecuteProcess(cmd=['gz', 'sim', '-r', os.path.join(simulation_dir, 'worlds', 'indoor.sdf')], output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'publish_frequency': 30.0, 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='p2os_urdf',
            executable='p2os_publisher_3at',
            name='wheel_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'pioneer3at/base_link', '--child-frame-id', 'base_link'],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'pioneer3at', '-topic', 'robot_description', '-z', '0.051'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': os.path.join(simulation_dir, 'config', 'gazebo_bridge.yaml')}],
            output='screen',
        ),
    ])
