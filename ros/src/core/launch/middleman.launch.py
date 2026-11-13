import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    core_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    return LaunchDescription([
        # Qwen (through Ollama) understands needs like "I'm hungry"; the rules decide everything else either way
        DeclareLaunchArgument('use_qwen', default_value='false'),
        ExecuteProcess(
            cmd=['python3', os.path.join(core_dir, 'middleman_node.py'), '--ros-args', '-p', ['use_qwen:=', LaunchConfiguration('use_qwen')]],
            output='screen',
        ),
    ])
