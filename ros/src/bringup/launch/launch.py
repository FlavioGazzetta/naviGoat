from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_package_launch(package, filename):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))


def generate_launch_description():
    return LaunchDescription([
        # base (motors on), lidar, RTAB-Map, Nav2 and the navigation nodes. It includes bringup's base.launch.py itself,
        # so base.launch.py isn't listed here as well (two base drivers on one serial port fight). Needs database_path:=...
        include_package_launch('navigation_nodes', 'navigation_robot.launch.py'),
        include_package_launch('handle_nodes', 'handle_launch.py'),
        include_package_launch('wristband_nodes', 'wristband_launch.py'),
    ])
