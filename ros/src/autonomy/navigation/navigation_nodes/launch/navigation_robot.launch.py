"""The whole navigation pipeline on the real robot: the base (with the motors turned on), the lidar and RTAB-Map
(from slam/launch/mapping.launch.py), Nav2, and the navigation nodes.

Map first (mode:=mapping, the default), then navigate on the saved map:

    ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_robot.launch.py \
        database_path:=/workspace/ros/src/bringup/maps/lab.db
    ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_robot.launch.py \
        database_path:=/workspace/ros/src/bringup/maps/lab.db mode:=localisation

The places are kept next to the map: lab.db -> lab_places.yaml. Only save places in localisation mode, on a finished
map: in mapping mode the map can still shift.

The base and lidar ports are found by themselves: /dev/pioneer and /dev/unilidar if the container maps them, otherwise
the /dev/serial/by-id/ names (container started with -v /dev:/dev, as in the navigation README). If the base or the
lidar still don't connect (SYNC failed, Setup() failed), check `ls -l /dev/serial/by-id/` and set pioneer_port:= and
lidar_port:= by hand.
"""

# everything for the real robot in one command: the base from bringup (so the motors get turned on), the team's
# slam/launch/mapping.launch.py for the lidar and RTAB-Map (its own base is switched off), Nav2 with the real-robot
# params, and our nodes from navigation.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, OrSubstitution, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))  # navigation_nodes/launch
SRC_DIR = os.path.abspath(os.path.join(LAUNCH_DIR, '..', '..', '..', '..'))  # ros/src
NAV2_PARAMS = os.path.join(SRC_DIR, 'autonomy', 'navigation', 'config', 'nav2_params.yaml')
# Behaviour trees without the Spin / BackUp recoveries: when Nav2 is stuck the robot stops and waits instead of turning
# on the spot. The params file can't hold a relative path, so the absolute one is written in at launch
BT_DIR = os.path.join(SRC_DIR, 'autonomy', 'navigation', 'behavior_trees')
BT_REWRITES = {
    'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(BT_DIR, 'navigate_to_pose_no_spin.xml'),
    'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': os.path.join(BT_DIR, 'navigate_through_poses_no_spin.xml'),
}
# The serial ports. /dev/pioneer and /dev/unilidar only exist when the container maps them (--device ...:/dev/pioneer).
# With -v /dev:/dev (the README's container, for the camera) they don't, so use the /dev/serial/by-id/ names: they always
# point at the right USB adapter, whichever /dev/ttyUSB number it got at boot
BY_ID = '/dev/serial/by-id/'
PIONEER_BY_ID = BY_ID + 'usb-Prolific_Technology_Inc._USB-Serial_Controller_CWDZb2A4517-if00-port0'
LIDAR_BY_ID = BY_ID + 'usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02AF3CBD-if00-port0'
PIONEER_PORT = '/dev/pioneer' if os.path.exists('/dev/pioneer') else PIONEER_BY_ID
LIDAR_PORT = '/dev/unilidar' if os.path.exists('/dev/unilidar') else LIDAR_BY_ID
PERCEPTION_DIR = os.path.join(SRC_DIR, 'autonomy', 'perception')
PEOPLE_DIR = os.path.join(PERCEPTION_DIR, 'social_perception', 'femto_id_velocity_tracker')


def social_nav():
    """Starts the YOLO server and Enid's people tracker with social:=true. The tracker publishes
    /social_perception/tracks for people_node. Needs the PETE=true image, the GPU and the camera."""
    social = IfCondition(LaunchConfiguration('social'))
    return [
        # YOLO, loaded once, asked over HTTP by the tracker (http://127.0.0.1:8767). --host 0.0.0.0: other machines can
        # ask it too, so Pete on the laptop uses this one (yolo_url:=http://192.168.55.1:8767) and loads none of its own
        ExecuteProcess(cmd=['python3', '-u', os.path.join(PERCEPTION_DIR, 'yolo_server.py'), '--host', '0.0.0.0'], output='screen', condition=social),
        # the tracker, with the same options as in pete.launch.py. Its own Python (the image's SOCIAL=true layer), run from
        # its folder so the YOLO weights saved there are reused
        ExecuteProcess(cmd=['/opt/people_env/bin/python3', '-u', 'femto_track_people.py', '--yolo-url', 'http://127.0.0.1:8767',
                            '--imgsz', '320', '--conf', '0.40', '--max-age', '1.0', '--no-plot'],
                       cwd=PEOPLE_DIR, output='screen', condition=social),
    ]


def launch_navigation_nodes(context):
    # the places file goes next to the map: /x/lab.db -> /x/lab_places.yaml. launch arguments only turn into text
    # inside an OpaqueFunction, that's why this is a separate function
    database = LaunchConfiguration('database_path').perform(context)
    places = LaunchConfiguration('places_file').perform(context) or os.path.splitext(database)[0] + '_places.yaml'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'navigation.launch.py')),
        launch_arguments={'places_file': places, 'use_sim_time': 'false'}.items(),
    )]


def require_navigation_sensor(context):
    raise RuntimeError('Enable use_lidar:=true or use_depth_camera:=true for navigation')


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    use_depth_camera = LaunchConfiguration('use_depth_camera')
    depth_sources = 'depth_camera depth_camera_clearing'
    nav2_params = RewrittenYaml(source_file=NAV2_PARAMS, param_rewrites=BT_REWRITES)
    depth_nav2_params = RewrittenYaml(
        source_file=NAV2_PARAMS,
        param_rewrites={
            **BT_REWRITES,
            'local_costmap.local_costmap.ros__parameters.voxel_layer.observation_sources': IfElseSubstitution(
                use_lidar, if_value=f'lidar lidar_clearing {depth_sources}', else_value=depth_sources),
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument('database_path', description='RTAB-Map database: a new file to make a map, an existing one to continue or navigate on it'),
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localisation']),
        DeclareLaunchArgument('places_file', default_value='', description='default: next to the map, <map>_places.yaml'),
        DeclareLaunchArgument('pioneer_port', default_value=PIONEER_PORT, description='default: /dev/pioneer if it exists, else the by-id name'),
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT, description='default: /dev/unilidar if it exists, else the by-id name'),
        DeclareLaunchArgument('filter', default_value='deskew', choices=['object', 'deskew', 'none'],
                              description='object: moving people are left out of the map (the costmaps still see them)'),
        DeclareLaunchArgument('enable_motors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', description="RTAB-Map's RViz on the Jetson (heavy: better on the laptop)"),
        DeclareLaunchArgument('teleop', default_value='false', choices=['true', 'false'],
                              description='true: no Nav2, so you can map with teleop without Nav2 fighting it over /cmd_vel. '
                                          'Ctrl+C saves the map; launch again without teleop (mode:=localisation) to navigate'),
        DeclareLaunchArgument('social', default_value='false', choices=['true', 'false'],
                              description='true: social nav with real people (camera, YOLO server, people tracker). Not '
                                          'together with pete.launch.py people:=true, which starts the same ones'),
        DeclareLaunchArgument('use_lidar', default_value='true', choices=['true', 'false'],
                              description='use lidar scans for SLAM and local costmap obstacles'),
        DeclareLaunchArgument('use_depth_camera', default_value='false', choices=['true', 'false'],
                              description='use Femto RGB-D input for SLAM and local costmap obstacles'),
        DeclareLaunchArgument('start_femto', default_value=OrSubstitution(use_depth_camera, LaunchConfiguration('social')),
                              choices=['true', 'false'], description='start Femto driver for depth or social navigation'),
        OpaqueFunction(function=require_navigation_sensor, condition=UnlessCondition(OrSubstitution(use_lidar, use_depth_camera))),

        # share one camera between rgb-d slam, nav2 and social perception
        ExecuteProcess(
            cmd=['ros2', 'launch', os.path.join(SRC_DIR, 'hardware', 'camera', 'launch', 'camera.launch.py'),
                 ['enable_point_cloud:=', use_depth_camera]],
            output='screen', condition=IfCondition(LaunchConfiguration('start_femto')),
        ),

        # the base, with the motors turned on once it's connected
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SRC_DIR, 'bringup', 'launch', 'base.launch.py')),
            launch_arguments={'port': LaunchConfiguration('pioneer_port'),
                              'enable_motors': LaunchConfiguration('enable_motors')}.items(),
        ),
        # lidar, robot model and RTAB-Map. start_base:=false because the base is started above.
        # odom_frame_id:=odom: RTAB-Map reads the wheel odometry from TF instead of matching /odom messages to scans.
        # mode goes through to slam.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SRC_DIR, 'autonomy', 'slam', 'launch', 'mapping.launch.py')),
            launch_arguments={
                'start_base': 'false',
                'database_path': LaunchConfiguration('database_path'),
                'mode': LaunchConfiguration('mode'),
                'odom_frame_id': 'odom',
                'use_rgbd': use_depth_camera,
                'subscribe_scan_cloud': use_lidar,
                'lidar_port': LaunchConfiguration('lidar_port'),
                'filter': LaunchConfiguration('filter'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
        # Nav2 with the real-robot params. Not with teleop:=true, so the keyboard has /cmd_vel to itself.
        # bond_timeout 0 = no heartbeat check. With it, Nav2's lifecycle manager switches ALL of Nav2 off as soon as one
        # server misses its heartbeat (4 s by default; 10 s wasn't enough either), and then every goal is rejected
        # ("Action server is inactive"). On the Jetson that kept happening at startup, while RTAB-Map loads the map.
        # Without the check, a really crashed controller isn't noticed by Nav2, but the robot still stops: the base
        # driver zeroes the speed when no cmd_vel arrives for 0.2 s. navigation_launch.py doesn't pass the params file
        # to the lifecycle manager, so it's set here, only for the nodes in this group
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('teleop')),
            scoped=True,
            actions=[
                SetParameter(name='bond_timeout', value=0.0),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
                    launch_arguments={'params_file': IfElseSubstitution(use_depth_camera, if_value=depth_nav2_params, else_value=nav2_params),
                                      'use_sim_time': 'false', 'use_composition': 'False'}.items(),
                ),
            ],
        ),
        # and our nodes
        OpaqueFunction(function=launch_navigation_nodes),
        # with social:=true, the people for the social layer
        *social_nav(),
    ])
