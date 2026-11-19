import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# The base's serial port: /dev/pioneer if the container maps it (--device ...:/dev/pioneer), otherwise (container started
# with -v /dev:/dev) the base's /dev/serial/by-id/ name, which always points at its USB adapter whatever ttyUSB number it got
PIONEER_BY_ID = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CWDZb2A4517-if00-port0'
PIONEER_PORT = '/dev/pioneer' if os.path.exists('/dev/pioneer') else PIONEER_BY_ID


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=PIONEER_PORT, description='default: /dev/pioneer if it exists, else the by-id name'),
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
