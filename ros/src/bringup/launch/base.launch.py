from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/pioneer'),
        DeclareLaunchArgument('enable_motors', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('p2os_bringup'), 'launch', 'p2os_driver_launch.py'])),
            launch_arguments={'port': LaunchConfiguration('port')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('p2os_bringup'), 'launch', 'enable_motors_launch.py'])),
            condition=IfCondition(LaunchConfiguration('enable_motors')),
        ),
    ])
