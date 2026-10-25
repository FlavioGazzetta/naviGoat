import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    p2os_share = get_package_share_directory('p2os_urdf')

    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(p2os_share)),
        ExecuteProcess(cmd=['gz', 'sim', '-r', 'empty.sdf'], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(p2os_share, 'launch', 'pioneer3at_urdf_launch.py'))
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
            arguments=['/model/pioneer3at/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
            output='screen',
        ),
    ])
