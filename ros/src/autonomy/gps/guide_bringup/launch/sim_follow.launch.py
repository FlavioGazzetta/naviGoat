"""Simulation run: sim robot + localizer + route follower + goal sender.

Usage:
  ros2 launch guide_bringup sim_follow.launch.py
  ros2 launch guide_bringup sim_follow.launch.py route_file:=/path/to/route.yaml
  ros2 launch guide_bringup sim_follow.launch.py route_file:=/tmp/planned.yaml \
      start_x:=-26.86 start_y:=-19.04 start_yaw_deg:=-82.0
  ros2 launch guide_bringup sim_follow.launch.py follow_route:=false \
      start_x:=-26.86 start_y:=-19.04 start_yaw_deg:=-82.0

The robot follows the route on its own until the route follower logs "Arrived".
With follow_route:=false the follower starts without a route and waits for one
on guide/set_route (from the mission manager or "plan_route ... --send").
Ctrl+C stops all nodes (the goal sender cancels any active goal first).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ros2 launch calls this function and starts everything in the returned list.

    # Launch argument: a value the user can override on the command line
    # (route_file:=...). The default is the L route installed with this package.
    # FindPackageShare finds install/guide_bringup/share/guide_bringup at launch
    # time, the same folder 'ros2 pkg prefix' pointed to in the manual runs.
    route_file_arg = DeclareLaunchArgument(
        'route_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('guide_bringup'), 'routes', 'l_route.yaml']),
        description='Route YAML file (frame_id: map, waypoints in metres)')

    # Step 7: whether the follower loads route_file at startup. 'false' starts it
    # without a route, so routes come only from guide/set_route (mission manager).
    # A separate switch is needed because ros2 launch does not accept an empty
    # value (route_file:='' is an error).
    follow_route_arg = DeclareLaunchArgument(
        'follow_route', default_value='true',
        description="'true': follow route_file; 'false': start without a route")

    # Where the simulated robot starts (map frame). Defaults = the origin,
    # facing east. plan_route prints values for the start of a planned route.
    start_args = [
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='Sim robot start, metres east of the origin'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='Sim robot start, metres north of the origin'),
        DeclareLaunchArgument('start_yaw_deg', default_value='0.0',
                              description='Sim robot start heading, degrees '
                                          '(0 = east, 90 = north)'),
    ]

    # Each Node action starts one executable (the same as 'ros2 run package executable').
    # output='screen' shows its log lines in this terminal, prefixed with the
    # process name, e.g. [route_follower-3].
    # Launch arguments are text; ParameterValue(..., value_type=float) makes sure
    # '-82' arrives as the float -82.0, because the sim declares these parameters
    # as floats and ROS rejects an int for a float parameter.
    sim = Node(
        package='guide_sim', executable='sim_robot', output='screen',
        parameters=[{
            name: ParameterValue(LaunchConfiguration(name), value_type=float)
            for name in ('start_x', 'start_y', 'start_yaw_deg')
        }])
    localizer = Node(package='guide_nodes', executable='localizer', output='screen')

    # Two alternative ways to start the route follower; exactly one of them runs.
    # condition=IfCondition(...) starts the node only if the argument is 'true',
    # UnlessCondition(...) only if it is 'false'.
    # parameters=[{...}] is the launch-file equivalent of --ros-args -p name:=value.
    # LaunchConfiguration('route_file') is replaced by the argument's value at launch.
    route_follower_with_file = Node(
        package='guide_nodes', executable='route_follower', output='screen',
        parameters=[{'route_file': LaunchConfiguration('route_file')}],
        condition=IfCondition(LaunchConfiguration('follow_route')))
    route_follower_without_route = Node(
        package='guide_nodes', executable='route_follower', output='screen',
        condition=UnlessCondition(LaunchConfiguration('follow_route')))

    goal_sender = Node(package='guide_nodes', executable='goal_sender', output='screen')

    return LaunchDescription([
        route_file_arg,
        follow_route_arg,
        *start_args,
        sim,
        localizer,
        route_follower_with_file,
        route_follower_without_route,
        goal_sender,
    ])
