"""Command-line client for the guide/plan_route service.

Plans a route, prints a summary, can save it as a route YAML that the
route follower loads (route_file parameter) and can send it straight to a
running route follower (guide/set_route), as the mission manager does.

Usage:
  ros2 run guide_nodes plan_route START_LAT START_LON GOAL_LAT GOAL_LON [--save FILE]
      [--send [--route-id N]]
"""

import argparse
import math
import os
import sys

from guide_core.crossings import (
    crossings_from_dict, crossings_to_dicts, prepare_crossings, RouteCrossing as CoreCrossing,
)
from guide_core.route import InvalidRouteError, Route, route_from_dict
from guide_core.walk_costs import CrossingType
from guide_interfaces.msg import RouteCrossing
from guide_interfaces.srv import PlanRoute, SetRoute
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import yaml

# RouteCrossing.msg's uint8 constants -> readable names.
TYPE_NAMES = {
    RouteCrossing.SIGNALLED: 'signalled',
    RouteCrossing.UNSIGNALLED: 'unsignalled',
    RouteCrossing.UNKNOWN: 'unknown',
}

# RouteCrossing.msg's uint8 constants -> guide_core's CrossingType, so the
# crossings can be written with guide_core's route-file helpers.
MSG_TO_CROSSING_TYPE = {
    RouteCrossing.SIGNALLED: CrossingType.SIGNALLED,
    RouteCrossing.UNSIGNALLED: CrossingType.UNSIGNALLED,
    RouteCrossing.UNKNOWN: CrossingType.UNKNOWN,
}


def parse_args(argv):
    """Read the command-line options (ROS arguments already removed)."""
    parser = argparse.ArgumentParser(description='Plan a walking route via guide/plan_route.')
    parser.add_argument('start_lat', type=float)
    parser.add_argument('start_lon', type=float)
    parser.add_argument('goal_lat', type=float)
    parser.add_argument('goal_lon', type=float)
    parser.add_argument('--save', metavar='FILE',
                        help='save the route as a route YAML for the route follower')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='seconds to wait for the service and for the answer (default 10)')
    parser.add_argument('--send', action='store_true',
                        help='also send the route to the route follower (guide/set_route)')
    parser.add_argument('--route-id', type=int, default=1,
                        help='route id for --send (> 0, default 1)')
    return parser.parse_args(argv)


def save_route_yaml(path, response):
    """Write the route in the follower's format: {frame_id, waypoints, crossings}.

    The file is read back with the same guide_core functions the route
    follower uses, so a file the follower cannot load is reported here.
    Returns (number of waypoints, number of crossings).
    """
    waypoints = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                 for p in response.path.poses]
    # Rebuild the route from the rounded points, exactly as the follower
    # will. Its length can differ from the planner's by rounding (mm), so
    # crossing ends are clamped to it.
    route = Route(waypoints)
    crossings = [CoreCrossing(start_s=max(0.0, c.start_s),
                              end_s=min(route.total_length, c.end_s),
                              crossing_type=MSG_TO_CROSSING_TYPE[c.crossing_type],
                              crossed_road=c.crossed_road,
                              tactile_paving=c.tactile_paving,
                              has_island=c.has_island)
                 for c in response.crossings]
    data = {'frame_id': response.path.header.frame_id,
            'waypoints': waypoints,
            'crossings': crossings_to_dicts(prepare_crossings(crossings, route.total_length))}
    header = ['# Planned by route_planner (guide/plan_route).',
              f'# {response.message}']
    # default_flow_style=None writes each waypoint and each crossing on one
    # line; sort_keys=False keeps the keys in the order above.
    body = yaml.safe_dump(data, default_flow_style=None, sort_keys=False, width=120)
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(header) + '\n' + body)

    # Read it back the way the route follower will.
    with open(path, encoding='utf-8') as f:
        loaded = yaml.safe_load(f)
    loaded_route = route_from_dict(loaded)
    return len(waypoints), len(crossings_from_dict(loaded, loaded_route))


def send_route(node, response, route_id, timeout):
    """Hand the route to the follower (guide/set_route); return 0 if accepted, else 1.

    Sends the planner's path and crossings unchanged, exactly as the mission
    manager will.
    """
    client = node.create_client(SetRoute, 'guide/set_route')
    if not client.wait_for_service(timeout_sec=timeout):
        print('Service guide/set_route not available. Is route_follower running?',
              file=sys.stderr)
        return 1
    future = client.call_async(SetRoute.Request(
        route_id=route_id, path=response.path, crossings=response.crossings))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        print('No answer from guide/set_route.', file=sys.stderr)
        return 1
    answer = future.result()
    print(f"Sent to follower: {'OK' if answer.success else 'REJECTED'}: {answer.message}")
    return 0 if answer.success else 1


def main(argv=None):
    """Call the service once; return 0 on success, 1 on any failure."""
    argv = sys.argv if argv is None else argv
    # remove_ros_args drops "--ros-args ..." so argparse only sees our own arguments.
    args = parse_args(remove_ros_args(argv)[1:])

    rclpy.init(args=argv)
    node = Node('plan_route_cli')
    # A service client: create_client(type, name) must match the server's type
    # and name exactly, or the call never connects.
    client = node.create_client(PlanRoute, 'guide/plan_route')
    try:
        if not client.wait_for_service(timeout_sec=args.timeout):
            print(f'Service guide/plan_route not available after {args.timeout:.0f} s. '
                  'Is route_planner running (and this terminal sourced)?', file=sys.stderr)
            return 1

        request = PlanRoute.Request(start_latitude=args.start_lat, start_longitude=args.start_lon,
                                    goal_latitude=args.goal_lat, goal_longitude=args.goal_lon)
        # call_async sends the request and returns a "future" that is filled
        # in when the answer arrives; spinning the node lets that happen.
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            print(f'No answer within {args.timeout:.0f} s.', file=sys.stderr)
            return 1
        response = future.result()

        if not response.success:
            print(f'FAILED: {response.message}')
            return 1
        print(response.message)
        # The same warnings as at the end of the message, but from the
        # separate warnings field that programs (the mission manager) read.
        print(f'Warnings field: {len(response.warnings)} item(s)')
        for i, warning in enumerate(response.warnings, start=1):
            print(f'  {i}. {warning}')
        print(f'Snapped: start {response.start_snap_distance:.1f} m, '
              f'goal {response.goal_snap_distance:.1f} m from the requested points')
        for c in response.crossings:
            print(f'  crossing at s = {c.start_s:.1f} .. {c.end_s:.1f} m: '
                  f'{TYPE_NAMES[c.crossing_type]}, {c.crossed_road} road, '
                  f'tactile paving: {c.tactile_paving}, island: {c.has_island}')
        first, last = response.path.poses[0].pose.position, response.path.poses[-1].pose.position
        print(f'Map frame: starts at ({first.x:.1f}, {first.y:.1f}) m, '
              f'ends at ({last.x:.1f}, {last.y:.1f}) m')

        # Path check: rebuild a Route from the UNROUNDED path points, as the
        # route follower does for a route received at runtime. If its length
        # matches the planner's, the crossings' s values line up with it.
        path_route = Route([(p.pose.position.x, p.pose.position.y)
                            for p in response.path.poses])
        difference_mm = (path_route.total_length - response.length) * 1000.0
        print(f'Path check: route rebuilt from the path is {path_route.total_length:.3f} m '
              f'(planner {response.length:.3f} m, difference {difference_mm:.3f} mm)')

        # Ready-made sim launch arguments: first route point, facing along the
        # first segment (the route always has at least 2 points).
        p0, p1 = response.path.poses[0].pose.position, response.path.poses[1].pose.position
        yaw_deg = math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))
        print(f'Sim start on this route: start_x:={p0.x:.2f} start_y:={p0.y:.2f} '
              f'start_yaw_deg:={yaw_deg:.1f}')
        if args.save:
            path = os.path.abspath(os.path.expanduser(args.save))
            try:
                count, crossing_count = save_route_yaml(path, response)
            except (InvalidRouteError, OSError) as error:
                # InvalidRouteError: the file would not load in the follower
                # (CrossingError is a subclass). OSError: could not write it.
                print(f'Could not save a valid route file: {error}', file=sys.stderr)
                return 1
            print(f'Saved {count} waypoints and {crossing_count} crossings to {path}')
        if args.send:
            return send_route(node, response, args.route_id, args.timeout)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
