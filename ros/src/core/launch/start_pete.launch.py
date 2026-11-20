"""What start_pete.sh starts, as a launch file: the API key, Ollama, Whisper on the GPU (with voice:=true) and then
pete.launch.py (camera, scene description, object detection, the middleman, and with their flags the voice, the people
tracker, person following and GPS). Run it in a container that is already running:

    source /opt/ros/jazzy/setup.bash
    ros2 launch /workspace/ros/src/core/launch/start_pete.launch.py
    ros2 launch /workspace/ros/src/core/launch/start_pete.launch.py voice:=true

Every argument of pete.launch.py works here under its own name.

Two things a launch file cannot do, because they belong to the host and not to the container (start_pete.sh did them
before it started the container):
  - the headset: on the Jetson, outside the container, run ros/src/interaction/voice/connect_airpods.sh first
  - the sound socket: the container must be started with
        -v /run/user/$(id -u)/pulse:/run/user/$(id -u)/pulse -e PULSE_SERVER=unix:/run/user/$(id -u)/pulse/native
    Without it the voice node has no microphone and no speaker.

The API key: start_pete.sh gave the container ~/pete.env. Here the key is used if the container already has it
(docker run --env-file ~/pete.env), otherwise it is read from pete.env in the repository folder (/workspace/pete.env,
git-ignored) or in the container's home folder. Without any of them Pete uses the local model only.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))  # ros/src
PETE = os.path.join(SRC_DIR, 'core', 'launch', 'pete.launch.py')
WHISPER_SERVER = os.path.join(SRC_DIR, 'interaction', 'voice', 'whisper_server.py')
WHISPER_URL = 'http://127.0.0.1:8765'
KEY_FILES = (os.path.join(SRC_DIR, '..', '..', 'pete.env'), os.path.expanduser('~/pete.env'))

# Ollama, unless one already answers (another terminal's, or the host's with --network=host)
OLLAMA = 'curl -s -m 2 127.0.0.1:11434 >/dev/null && { echo "[ollama] already running"; exit 0; }; exec ollama serve > /tmp/ollama.log 2>&1'
# Whisper on the GPU, unless one already answers (the whisper-gpu container of start_whisper_gpu.sh)
WHISPER = f'curl -s -m 2 {WHISPER_URL}/health | grep -q ok && {{ echo "[whisper] already running"; exit 0; }}; exec python3 "$0" --model "$1"'
# Loading Whisper takes up to a minute; the voice node must not start before it answers. 3 minutes at most
WAIT_FOR_WHISPER = (f'for i in $(seq 1 90); do curl -s -m 2 {WHISPER_URL}/health | grep -q ok && {{ echo "[whisper] ready"; exit 0; }}; sleep 2; done; '
                    'echo "[whisper] the GPU server did not start: Whisper will run on the CPU"; exit 1')


def key_from_file():
    """The lines of the first pete.env found (KEY=value), as environment variables for every node. Values are never logged."""
    if os.environ.get('ANTHROPIC_API_KEY'):
        return [LogInfo(msg='[start_pete] the container already has the API key')]
    for path in map(os.path.normpath, KEY_FILES):
        if os.path.isfile(path):
            with open(path) as file:
                lines = [line.strip().split('=', 1) for line in file if '=' in line and not line.lstrip().startswith('#')]
            return [LogInfo(msg=f'[start_pete] API key read from {path}')] + [
                SetEnvironmentVariable(name.strip(), value.strip().strip('"\'')) for name, value in lines]
    return [LogInfo(msg='[start_pete] no pete.env: Pete will use the local model only')]


def generate_launch_description():
    arg = LaunchConfiguration
    pete = lambda **arguments: IncludeLaunchDescription(PythonLaunchDescriptionSource(PETE), launch_arguments=arguments.items())
    # this file starts its own Whisper server only for voice on the default address (whisper_url:=cpu or another address: no)
    own_whisper = PythonExpression(["'", arg('voice'), "' == 'true' and '", arg('whisper_url'), f"' == '{WHISPER_URL}'"])
    wait_for_whisper = ExecuteProcess(cmd=['bash', '-c', WAIT_FOR_WHISPER], output='screen', condition=IfCondition(own_whisper))

    return LaunchDescription([
        DeclareLaunchArgument('voice', default_value='false'),
        DeclareLaunchArgument('whisper_url', default_value=WHISPER_URL),
        DeclareLaunchArgument('whisper_model', default_value='base.en'),
        *key_from_file(),
        # local_llm:=false (once pete.launch.py has that argument) also leaves Ollama out
        ExecuteProcess(cmd=['bash', '-c', OLLAMA], output='screen',
                       condition=UnlessCondition(PythonExpression(["'", arg('local_llm', default='true'), "' == 'false'"]))),

        # without voice: Pete straight away
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PETE), condition=UnlessCondition(own_whisper)),
        # with voice: Whisper first, Pete when it answers (on the CPU if it never does, as start_pete.sh did)
        ExecuteProcess(cmd=['bash', '-c', WHISPER, WHISPER_SERVER, arg('whisper_model')], output='screen', condition=IfCondition(own_whisper)),
        wait_for_whisper,
        RegisterEventHandler(OnProcessExit(target_action=wait_for_whisper, on_exit=lambda event, context: [
            pete(whisper_url='cpu') if event.returncode else pete()])),
    ])
