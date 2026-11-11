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
        include_package_launch('handle_nodes', 'handle_launch.py'),
    ])
