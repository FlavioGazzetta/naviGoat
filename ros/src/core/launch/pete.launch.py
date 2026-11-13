import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    src = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
    launch_file = lambda *path: PythonLaunchDescriptionSource(os.path.join(src, *path))
    arg = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),  # false when the Orbbec ROS 2 driver runs the camera
        DeclareLaunchArgument('search_time', default_value='5.0'),
        DeclareLaunchArgument('use_qwen', default_value='false'),
        DeclareLaunchArgument('voice', default_value='false'),  # true needs the VOICE=true image and the host's sound socket
        DeclareLaunchArgument('whisper_url', default_value='http://127.0.0.1:8765'),
        DeclareLaunchArgument('listen', default_value='true'),
        DeclareLaunchArgument('device', default_value='pulse'),

        # camera, scene description, then object detection once Qwen has loaded
        IncludeLaunchDescription(launch_file('autonomy', 'perception', 'launch', 'perception.launch.py'),
                                 launch_arguments={'camera': arg('camera'), 'search_time': arg('search_time')}.items()),
        # the voice launch starts the middleman itself, so only one of these two runs
        IncludeLaunchDescription(launch_file('interaction', 'voice', 'launch', 'voice.launch.py'), condition=IfCondition(arg('voice')),
                                 launch_arguments={'use_qwen': arg('use_qwen'), 'whisper_url': arg('whisper_url'),
                                                   'listen': arg('listen'), 'device': arg('device')}.items()),
        IncludeLaunchDescription(launch_file('core', 'launch', 'middleman.launch.py'), condition=UnlessCondition(arg('voice')),
                                 launch_arguments={'use_qwen': arg('use_qwen')}.items()),
    ])
