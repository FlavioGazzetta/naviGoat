import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    voice_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    middleman_launch = os.path.normpath(os.path.join(voice_dir, '..', '..', 'core', 'launch', 'middleman.launch.py'))

    return LaunchDescription([
        DeclareLaunchArgument('use_qwen', default_value='false'),
        DeclareLaunchArgument('whisper_url', default_value='http://127.0.0.1:8765'),  # the Whisper GPU server; cpu runs Whisper on the CPU
        DeclareLaunchArgument('listen', default_value='true'),  # false: no microphone, the node only speaks (for tests)
        DeclareLaunchArgument('device', default_value='pulse'),  # the microphone: pulse is the host's sound system
        DeclareLaunchArgument('silence', default_value='1.0'),  # seconds of quiet that end a phrase; shorter cut sentences at a pause
        # decides what a request means and tells perception (also started by core/launch/middleman.launch.py alone)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(middleman_launch), launch_arguments={'use_qwen': LaunchConfiguration('use_qwen')}.items()),
        # publishes what was said on /speech/text and speaks /pete/response
        ExecuteProcess(
            cmd=['python3', os.path.join(voice_dir, 'voice.py'), '--ros-args',
                 '-p', ['whisper_url:=', LaunchConfiguration('whisper_url')],
                 '-p', ['listen:=', LaunchConfiguration('listen')],
                 '-p', ['device:=', LaunchConfiguration('device')],
                 '-p', ['silence:=', LaunchConfiguration('silence')], '-p', 'follow_up:=4.0', '-p', 'min_rms:=0.006'],
            output='screen',
        ),
    ])
