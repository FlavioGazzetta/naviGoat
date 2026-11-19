"""The GPS navigation layer: one launch file for the robot and for the simulator.

  ros2 launch guide_bringup guide.launch.py              # the robot (sim:=false)
  ros2 launch guide_bringup guide.launch.py sim:=true    # our simulator

Always starts our nodes: localizer, route follower, goal sender, route planner,
geocoder, mission manager and the GPS bridge (the only node the rest of the
robot talks to, over plain topics: see the README).

sim:=false (default) expects the team's RTAB-Map (TF map -> base_link), Nav2
(the navigate_to_pose action) and a GPS driver (/gps/fix). Without a GPS the
localizer never starts and a GPS request is refused.

sim:=true also starts guide_sim's simulated robot (sensors, TF, a fake Nav2)
and a fake Nominatim on 127.0.0.1:8088 with a few canned answers ("science
museum", "imperial college", "museum"), so nothing goes online. Its answers
are cached in a separate file, never mixed with real ones.

Settings: config/gps.yaml (origin, topic names, frames, Nav2 action), or your
own copy with params_file:=... Arguments: see
  ros2 launch guide_bringup guide.launch.py --show-args
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Where the geocoder keeps its answers, outside the workspace so updates never
# touch it. The simulator's fake answers go to a file of their own.
CACHE_FILE = '~/.local/share/navigoat/geocode_cache.json'
SIM_CACHE_FILE = '~/.local/share/navigoat/geocode_cache_sim.json'
FAKE_NOMINATIM_PORT = '8088'


def make_nodes(context):
    """Build the node list for sim:=true or sim:=false.

    OpaqueFunction calls this at launch time, so .perform(context) gives the
    arguments' actual text and plain Python can choose what to start. With
    sim:=false nothing here refers to guide_sim, so the robot's build (which
    leaves guide_sim out) still launches.
    """
    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    params_file = LaunchConfiguration('params_file')
    graph_file = LaunchConfiguration('graph_file')
    cache_file = (LaunchConfiguration('geocode_cache_file').perform(context)
                  or (SIM_CACHE_FILE if sim else CACHE_FILE))

    # Every node gets gps.yaml first, then the values given here, which win.
    def node(executable, **params):
        return Node(package='guide_nodes', executable=executable, output='screen',
                    parameters=[params_file, params])

    geocoder_params = {
        'graph_file': graph_file,
        'cache_file': cache_file,
        # ParameterValue(..., value_type=bool) turns 'true' / 'false' into a real bool.
        'online_enabled': ParameterValue(LaunchConfiguration('online_lookup'),
                                         value_type=bool),
    }
    if sim:
        geocoder_params['base_url'] = f'http://127.0.0.1:{FAKE_NOMINATIM_PORT}'

    nodes = [
        node('localizer'),
        node('route_follower'),       # no route_file: routes come from the mission manager
        node('goal_sender'),
        node('route_planner', graph_file=graph_file),
        node('geocoder', **geocoder_params),
        node('mission_manager', destinations_file=LaunchConfiguration('destinations_file')),
        node('gps_bridge'),
    ]
    if not sim:
        return nodes

    def as_float(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    sim_share = FindPackageShare('guide_sim')
    return nodes + [
        # The simulated robot: GPS, IMU, odometry, TF and a fake Nav2, with
        # exact sensors (mission.launch.py has the noisy presets).
        Node(package='guide_sim', executable='sim_robot', output='screen',
             parameters=[params_file, {
                 'start_x': as_float('start_x'),
                 'start_y': as_float('start_y'),
                 'start_yaw_deg': as_float('start_yaw_deg'),
                 'graph_file': graph_file,
                 'report_gps_covariance': True,
             }]),
        # fake_nominatim is a small web server, not a ROS node, so it is started
        # as a plain process (a launch Node would add --ros-args, which it rejects).
        ExecuteProcess(
            name='fake_nominatim', output='screen',
            cmd=['ros2', 'run', 'guide_sim', 'fake_nominatim',
                 '--port', FAKE_NOMINATIM_PORT,
                 '--data-dir', PathJoinSubstitution([sim_share, 'nominatim'])]),
    ]


def generate_launch_description():
    # ros2 launch calls this function and starts everything in the returned list.
    # FindPackageShare finds share/guide_bringup (the installed data files) at launch time.
    share = FindPackageShare('guide_bringup')

    # Launch arguments: values that can be changed on the command line (name:=value).
    args = [
        DeclareLaunchArgument('sim', default_value='false',
                              description="'true' adds our simulator and a fake Nominatim"),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([share, 'config', 'gps.yaml']),
            description='Parameter file: origin, topic names, frames, Nav2 action'),
        DeclareLaunchArgument(
            'graph_file',
            default_value=PathJoinSubstitution([share, 'osm', 'south_ken_walk.graphml']),
            description='Cached OSM walking graph (.graphml from fetch_osm_graph)'),
        DeclareLaunchArgument(
            'destinations_file',
            default_value=PathJoinSubstitution([share, 'destinations', 'campus.yaml']),
            description='Curated destination list for the mission manager'),
        # Online lookup of names not in the destination list (Nominatim usage
        # policy: https://operations.osmfoundation.org/policies/nominatim/).
        # 'false' = never go online; places already in the cache still work.
        DeclareLaunchArgument('online_lookup', default_value='true',
                              description="'false' switches online search off"),
        DeclareLaunchArgument(
            'geocode_cache_file', default_value='',
            description=f'Saved search answers (default {CACHE_FILE}, '
                        f'with sim:=true {SIM_CACHE_FILE})'),
        # Simulator only. The default is the start of the demo route to
        # South Kensington Station (the robot must start on a walkway).
        DeclareLaunchArgument('start_x', default_value='-26.86',
                              description='sim:=true: start, metres east of the origin'),
        DeclareLaunchArgument('start_y', default_value='-19.04',
                              description='sim:=true: start, metres north of the origin'),
        DeclareLaunchArgument('start_yaw_deg', default_value='-82.0',
                              description='sim:=true: start heading, degrees (0 = east)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description="'true' also starts RViz"),
    ]

    # Optional RViz with the planner's view (OSM graph, planned route).
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'route_planner.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        *args,
        OpaqueFunction(function=make_nodes),
        rviz,
    ])
