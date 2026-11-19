r"""Full mission in simulation: one action goal takes the robot to a destination.

Starts: sim robot, localizer, route follower (no route; routes come from the
mission manager), goal sender, route planner (cached OSM graph), geocoder
(names not in the curated list: Nominatim online + on-disk cache), mission
manager (curated destinations) and the GPS bridge (the team's topics in and
out, Step 10); optionally RViz and the run evaluator.

Usage:
  ros2 launch guide_bringup mission.launch.py
  ros2 launch guide_bringup mission.launch.py rviz:=true
  ros2 launch guide_bringup mission.launch.py start_x:=... start_y:=... start_yaw_deg:=...
  ros2 launch guide_bringup mission.launch.py online_lookup:=false   # never go online
  # Step 8: realistic sensors, repeatable with a seed, scored against ground truth:
  ros2 launch guide_bringup mission.launch.py sensors:=urban_canyon noise_seed:=2
      evaluate:=true auto_go:=true            (all on one line)

Then, in another terminal, as the team's middleman and handle would (Step 10):
  ros2 topic pub -w 1 --once /middleman/mode std_msgs/msg/String
      "data: '{\"mode\": \"gps\", \"target\": \"south ken\"}'"   (one line)
  ros2 topic pub -w 1 --once /gps/go std_msgs/msg/Empty      # the user's go
  ros2 topic echo /pete/response                             # what Pete says
Or with our own interfaces directly:
  ros2 action send_goal --feedback /guide/navigate_to
      guide_interfaces/action/NavigateTo "{destination: 'south ken'}"   (one line)
  ros2 service call /guide/user_go std_srvs/srv/Trigger      # the user's go
  ros2 service call /guide/user_pause std_srvs/srv/Trigger   # the user's pause

Ctrl+C stops all nodes (the goal sender cancels any active goal first).

This is our development launch (sensor presets, evaluator); guide.launch.py is
the one for the team. Both read config/gps.yaml (origin, topic names, frames),
or another file with params_file:=...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Sensor presets for the simulator (Step 8). Each maps to sim_robot parameters;
# 'gps_zones' names a file in share/guide_bringup/sim/ (None = the same GPS
# everywhere, given by gps_profile). Every preset reports the receiver's sigma
# (report_gps_covariance), so the localizer knows how good the GPS claims to be.
SENSOR_PRESETS = {
    # Regression: everything exact, map frame = enu (the M3 conditions).
    'perfect': {'gps_profile': 'perfect', 'gps_zones': None, 'odom_profile': 'perfect',
                'slam_profile': 'enu', 'imu_yaw_bias_deg': 0.0, 'imu_yaw_noise_deg': 0.0},
    # Frames only: exact sensors, but the map frame starts at the robot (as RTAB-Map's).
    'map_offset': {'gps_profile': 'perfect', 'gps_zones': None, 'odom_profile': 'perfect',
                   'slam_profile': 'perfect', 'imu_yaw_bias_deg': 0.0,
                   'imu_yaw_noise_deg': 0.0},
    # Noisy: open-sky GPS, no fix in tunnels, drifting odometry and SLAM, biased compass.
    'open': {'gps_profile': 'open', 'gps_zones': 'gps_open.yaml',
             'odom_profile': 'skid_steer', 'slam_profile': 'lidar',
             'imu_yaw_bias_deg': 5.0, 'imu_yaw_noise_deg': 2.0},
    # M4: as 'open', with urban-canyon GPS (bias, jitter, multipath jumps, dropouts).
    'urban_canyon': {'gps_profile': 'urban_canyon', 'gps_zones': 'gps_urban_canyon.yaml',
                     'odom_profile': 'skid_steer', 'slam_profile': 'lidar',
                     'imu_yaw_bias_deg': 5.0, 'imu_yaw_noise_deg': 2.0},
}


def make_sim(context, share, params_file, as_float):
    """Build the sim_robot node for the chosen sensors preset.

    OpaqueFunction calls this at launch time with the launch context, so
    .perform(context) turns a launch argument into its actual text: the preset
    has to be looked up in SENSOR_PRESETS (plain Python) before the node exists.
    """
    name = LaunchConfiguration('sensors').perform(context)
    if name not in SENSOR_PRESETS:
        raise RuntimeError(f"Unknown sensors preset '{name}'; "
                           f'choose one of: {", ".join(SENSOR_PRESETS)}')
    preset = SENSOR_PRESETS[name]
    zones = (PathJoinSubstitution([share, 'sim', preset['gps_zones']])
             if preset['gps_zones'] else 'none')
    return [Node(
        package='guide_sim', executable='sim_robot', output='screen',
        parameters=[params_file, {
            'start_x': as_float('start_x'),
            'start_y': as_float('start_y'),
            'start_yaw_deg': as_float('start_yaw_deg'),
            'noise_seed': ParameterValue(LaunchConfiguration('noise_seed'), value_type=int),
            'gps_profile': preset['gps_profile'],
            'gps_zones_file': zones,
            'graph_file': LaunchConfiguration('graph_file'),   # tunnels for the zones
            'report_gps_covariance': True,
            'odom_profile': preset['odom_profile'],
            'slam_profile': preset['slam_profile'],
            'imu_yaw_bias_deg': preset['imu_yaw_bias_deg'],
            'imu_yaw_noise_deg': preset['imu_yaw_noise_deg'],
        }])]


def generate_launch_description():
    # ros2 launch calls this function and starts everything in the returned list.

    # Files installed with this package (share/guide_bringup/...). FindPackageShare
    # finds that folder at launch time.
    share = FindPackageShare('guide_bringup')

    # Launch arguments: values the user can override on the command line (name:=value).
    args = [
        # The robot must start on a walkway (the planner snaps the start to one);
        # the default is the start of the demo route to South Kensington Station.
        DeclareLaunchArgument('start_x', default_value='-26.86',
                              description='Sim robot start, metres east of the origin'),
        DeclareLaunchArgument('start_y', default_value='-19.04',
                              description='Sim robot start, metres north of the origin'),
        DeclareLaunchArgument('start_yaw_deg', default_value='-82.0',
                              description='Sim robot start heading, degrees (0 = east)'),
        # The parameter file (Step 10): the enu origin, topic names and frames,
        # given ONCE there and read by every node, so they cannot disagree.
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
        # Online lookup of names not in the curated list (Nominatim usage policy:
        # https://operations.osmfoundation.org/policies/nominatim/). 'false' = the
        # geocoder never goes online; places already in its cache still work.
        DeclareLaunchArgument('online_lookup', default_value='true',
                              description="'false' switches online search off"),
        DeclareLaunchArgument(
            'geocode_cache_file', default_value='~/.local/share/navigoat/geocode_cache.json',
            description='Saved online answers (kept outside the workspace; survives updates)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description="'true' also starts RViz"),
        # Step 8: simulated sensors (see SENSOR_PRESETS) and the run evaluator.
        DeclareLaunchArgument('sensors', default_value='perfect',
                              description='Sim sensors: ' + ' | '.join(SENSOR_PRESETS)),
        DeclareLaunchArgument('noise_seed', default_value='0',
                              description='Seed of all simulated noise (same seed = same run)'),
        DeclareLaunchArgument('evaluate', default_value='false',
                              description="'true' starts run_evaluator (scores vs ground truth)"),
        DeclareLaunchArgument('auto_go', default_value='false',
                              description="'true': run_evaluator presses go (test runs only)"),
    ]

    # Launch arguments are text. ParameterValue(..., value_type=float) makes sure
    # '-82' or '51.4988' arrives as a float, because the nodes declare these
    # parameters as floats and ROS rejects an int (or a string) for a float.
    def as_float(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    params_file = LaunchConfiguration('params_file')

    # Each Node action starts one executable (the same as 'ros2 run package executable').
    # output='screen' shows its log lines in this terminal, prefixed with the
    # process name, e.g. [mission_manager-6]. parameters=[{...}] is the launch-file
    # equivalent of --ros-args -p name:=value; a file name in the list (params_file)
    # loads that parameter file first, and the dicts after it win.
    # The simulator depends on the sensors preset, so it is built at launch time.
    sim = OpaqueFunction(function=make_sim, args=[share, params_file, as_float])
    localizer = Node(package='guide_nodes', executable='localizer', output='screen',
                     parameters=[params_file])
    # No route_file: the follower waits for routes on guide/set_route.
    route_follower = Node(package='guide_nodes', executable='route_follower',
                          output='screen', parameters=[params_file])
    goal_sender = Node(package='guide_nodes', executable='goal_sender', output='screen',
                       parameters=[params_file])
    route_planner = Node(
        package='guide_nodes', executable='route_planner', output='screen',
        parameters=[params_file, {'graph_file': LaunchConfiguration('graph_file')}])
    # The geocoder reads only the small .meta.yaml next to graph_file (the area
    # to search in). ParameterValue(..., value_type=bool) turns 'true'/'false'
    # into a real bool, as the node declares online_enabled as a bool.
    geocoder = Node(
        package='guide_nodes', executable='geocoder', output='screen',
        parameters=[params_file, {
            'graph_file': LaunchConfiguration('graph_file'),
            'cache_file': LaunchConfiguration('geocode_cache_file'),
            'online_enabled': ParameterValue(LaunchConfiguration('online_lookup'),
                                             value_type=bool),
        }])
    mission_manager = Node(
        package='guide_nodes', executable='mission_manager', output='screen',
        parameters=[params_file,
                    {'destinations_file': LaunchConfiguration('destinations_file')}])
    # The GPS bridge: the team's topics (/middleman/mode, /gps/go, ...) in and
    # out. Its topic names are in the parameter file.
    gps_bridge = Node(package='guide_nodes', executable='gps_bridge', output='screen',
                      parameters=[params_file])

    # Optional RViz with the planner's configuration (OSM graph, planned route).
    # condition=IfCondition(...) starts the node only if the argument is 'true'.
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'route_planner.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')))

    # Optional run evaluator (dev tool in guide_sim): compares the stack with the
    # simulator's ground truth and prints a summary when the trip ends.
    evaluator = Node(
        package='guide_sim', executable='run_evaluator', output='screen',
        parameters=[params_file,
                    {'auto_go': ParameterValue(LaunchConfiguration('auto_go'), value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('evaluate')))

    return LaunchDescription([
        *args,
        sim,
        localizer,
        route_follower,
        goal_sender,
        route_planner,
        geocoder,
        mission_manager,
        gps_bridge,
        rviz,
        evaluator,
    ])
