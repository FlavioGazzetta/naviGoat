import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    src = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
    launch_file = lambda *path: PythonLaunchDescriptionSource(os.path.join(src, *path))
    arg = LaunchConfiguration
    # the people tracker and its dashboard: their own Python (/opt/people_env, the image's SOCIAL=true layer), run from
    # their folder so the YOLO weights saved there are reused
    people_dir = os.path.join(src, 'autonomy', 'perception', 'social_perception', 'femto_id_velocity_tracker')
    is_true = lambda name: ["'", arg(name), "' == 'true'"]
    people = lambda *command: ExecuteProcess(cmd=['/opt/people_env/bin/python3', '-u', *command], cwd=people_dir, output='screen',
                                             condition=IfCondition(PythonExpression(is_true('people') + [' or '] + is_true('follow'))))
    # Person following (autonomy/person_following): locks onto the nearest tracked person when the middleman says
    # "follow" and works out a spot one metre short of them. As a preview it only publishes that spot
    # (/person_following/goal_preview) and treats the camera as the robot, so it needs no map and cannot move anything.
    # follow_drive:=true sends the spots to Nav2 instead, which needs SLAM, Nav2 and the base running.
    follow_script = os.path.join(src, 'autonomy', 'person_following', 'ros_nav2_follow.py')
    following = lambda drive, *options: ExecuteProcess(
        cmd=['python3', '-B', '-u', follow_script, '--input-timeout', arg('follow_input_timeout'), *options], output='screen',
        condition=IfCondition(PythonExpression(is_true('follow') + [' and '] + (is_true('follow_drive') if drive else ['not ('] + is_true('follow_drive') + [')']))))

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),  # false when the Orbbec ROS 2 driver runs the camera
        DeclareLaunchArgument('search_time', default_value='5.0'),
        DeclareLaunchArgument('use_qwen', default_value='true'),  # the middleman shares scene description's loaded Qwen
        DeclareLaunchArgument('voice', default_value='false'),  # true needs the VOICE=true image and the host's sound socket
        DeclareLaunchArgument('whisper_url', default_value='http://127.0.0.1:8765'),
        DeclareLaunchArgument('listen', default_value='true'),
        DeclareLaunchArgument('device', default_value='pulse'),
        DeclareLaunchArgument('silence', default_value='1.0'),  # seconds of quiet that end a spoken phrase
        DeclareLaunchArgument('people', default_value='false'),  # true also tracks people, on /social_perception/tracks
        DeclareLaunchArgument('people_port', default_value='8766'),  # the dashboard's; 8765 is Whisper's
        DeclareLaunchArgument('people_max_age', default_value='1.0'),  # seconds to retain an unobserved track; following uses 2.0
        DeclareLaunchArgument('follow', default_value=arg('people')),  # person following starts with the tracker; follow:=false leaves it out
        DeclareLaunchArgument('follow_drive', default_value='false'),  # true: send the goals to Nav2 (the robot moves)
        # How old (seconds since the camera took the picture) a tracking result may be before following gives up. The
        # node's own 0.5 is too tight on the Jetson: results are normally 0.2-0.3 s old and now and then over 0.5 s
        # (measured 0.51 and 0.64 s), which ended following about a second after it started.
        DeclareLaunchArgument('follow_input_timeout', default_value='1.0'),

        # camera, scene description, then object detection once Qwen has loaded
        IncludeLaunchDescription(launch_file('autonomy', 'perception', 'launch', 'perception.launch.py'),
                                 launch_arguments={'camera': arg('camera'), 'search_time': arg('search_time')}.items()),
        # the voice launch starts the middleman itself, so only one of these two runs
        IncludeLaunchDescription(launch_file('interaction', 'voice', 'launch', 'voice.launch.py'), condition=IfCondition(arg('voice')),
                                 launch_arguments={'use_qwen': arg('use_qwen'), 'whisper_url': arg('whisper_url'),
                                                   'listen': arg('listen'), 'device': arg('device'), 'silence': arg('silence')}.items()),
        IncludeLaunchDescription(launch_file('core', 'launch', 'middleman.launch.py'), condition=UnlessCondition(arg('voice')),
                                 launch_arguments={'use_qwen': arg('use_qwen')}.items()),
        # they read the camera topics that perception's camera already publishes, so no second driver is started
        # --yolo-url: the tracker asks perception's yolo_server.py, as object detection does, so YOLO is in memory once
        people('femto_track_people.py', '--yolo-url', 'http://127.0.0.1:8767', '--imgsz', '320', '--conf', '0.40',
               '--max-age', PythonExpression(["'2.0' if "] + is_true('follow') + [" else '", arg('people_max_age'), "'"]), '--no-plot'),
        people('social_tracking_dashboard.py', '--host', '0.0.0.0', '--port', arg('people_port')),
        following(False, '--global-frame', 'camera_link', '--base-frame', 'camera_link'),
        following(True, '--execute-nav2'),
    ])
