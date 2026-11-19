"""ROS 2 node: plans walking routes on the cached OpenStreetMap graph.

Serves guide/plan_route (guide_interfaces/srv/PlanRoute). All routing
decisions live in guide_core.route_planner; this node only loads the graph,
converts between ROS messages and guide_core types, and publishes things to
look at in RViz.

Frames (Step 8): the planned path and the graph markers are in route_frame
(default 'enu': metres east / north of the configured origin), not in the SLAM
map frame. The simulator broadcasts TF enu -> map so RViz can show them.
"""

from collections import Counter
from dataclasses import fields
import math
import os
import sys
import time

from geometry_msgs.msg import Point, PoseStamped
from guide_core.angles import yaw_to_quaternion
from guide_core.geo import latlon_to_enu
from guide_core.route_planner import PlannerConfig, PlanningError, RoutePlanner
from guide_core.walk_costs import (
    classify_crossing, CostConfig, CrossingType, walkway_class, WalkwayClass,
)
from guide_interfaces.msg import RouteCrossing
from guide_interfaces.srv import PlanRoute
from nav_msgs.msg import Path
import osmnx as ox
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Latched QoS for rarely-changing state topics: RELIABLE (every message
# arrives) + TRANSIENT_LOCAL (the publisher keeps the last message and hands
# it to subscribers that join later, e.g. RViz started after this node).
# Depth 1 = keep only the latest message.
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

# guide_core's crossing types -> the uint8 constants in RouteCrossing.msg.
CROSSING_TYPE_TO_MSG = {
    CrossingType.SIGNALLED: RouteCrossing.SIGNALLED,
    CrossingType.UNSIGNALLED: RouteCrossing.UNSIGNALLED,
    CrossingType.UNKNOWN: RouteCrossing.UNKNOWN,
}


class StartupError(Exception):
    """The node cannot start (bad parameter, missing graph file)."""


def make_marker(ns, marker_type, points, rgba, scale, frame_id, stamp):
    """Build one RViz Marker from a list of (x, y) points in the route (enu) frame.

    LINE_LIST draws a separate line between each consecutive pair of points
    (scale.x = line width in metres). SPHERE_LIST draws a sphere at every
    point (scale = diameter). One marker per layer keeps RViz fast.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns          # namespace: RViz lets you toggle each one separately
    marker.id = 0           # (ns, id) identifies a marker; re-publishing replaces it
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0  # identity pose: points are already in frame_id
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    marker.points = [Point(x=x, y=y, z=0.0) for x, y in points]
    return marker


def build_graph_markers(graph, origin, frame_id, stamp):
    """Return (MarkerArray, counts) drawing the walking graph in layers."""
    xy = {node: latlon_to_enu(float(d['y']), float(d['x']), *origin)
          for node, d in graph.nodes(data=True)}

    # Each walkway piece is stored in both directions; draw each pair once.
    lines = {'walkways': [], 'roads': [], 'forbidden': [], 'islands': []}
    seen = set()
    for u, v, data in graph.edges(data=True):
        pair = (min(u, v), max(u, v))
        if pair in seen:
            continue
        seen.add(pair)
        walk_class = walkway_class(data.get('highway'))
        if str(data.get('footway', '')).strip().lower() == 'traffic_island':
            layer = 'islands'  # pedestrian island between two halves of a crossing
        else:
            layer = ('forbidden' if walk_class is WalkwayClass.FORBIDDEN
                     else 'walkways' if walk_class is WalkwayClass.WALKWAY else 'roads')
        lines[layer] += [xy[u], xy[v]]

    dots = {t: [] for t in CrossingType}
    for node, data in graph.nodes(data=True):
        if str(data.get('highway', '')).strip().lower() == 'crossing':
            dots[classify_crossing(data)].append(xy[node])

    # (namespace, type, points, colour RGBA, scale)
    layers = [
        ('walkways', Marker.LINE_LIST, lines['walkways'], (0.65, 0.65, 0.65, 1.0), 0.6),
        ('roads', Marker.LINE_LIST, lines['roads'], (0.35, 0.35, 0.35, 1.0), 1.0),
        ('forbidden', Marker.LINE_LIST, lines['forbidden'], (0.9, 0.1, 0.1, 1.0), 1.0),
        ('islands', Marker.LINE_LIST, lines['islands'], (0.0, 0.8, 1.0, 1.0), 1.5),
        ('crossings_signalled', Marker.SPHERE_LIST, dots[CrossingType.SIGNALLED],
         (0.1, 0.8, 0.2, 1.0), 4.0),
        ('crossings_unsignalled', Marker.SPHERE_LIST, dots[CrossingType.UNSIGNALLED],
         (1.0, 0.5, 0.0, 1.0), 4.0),
        ('crossings_unknown', Marker.SPHERE_LIST, dots[CrossingType.UNKNOWN],
         (1.0, 0.9, 0.1, 1.0), 4.0),
    ]
    array = MarkerArray()
    array.markers = [make_marker(ns, mtype, pts, rgba, scale, frame_id, stamp)
                     for ns, mtype, pts, rgba, scale in layers if pts]
    counts = {
        'walkway pieces': len(lines['walkways']) // 2,
        'road pieces': len(lines['roads']) // 2,
        'forbidden pieces': len(lines['forbidden']) // 2,
        'crossings': sum(len(p) for p in dots.values()),
        'island pieces': len(lines['islands']) // 2,
    }
    return array, counts


class RoutePlannerNode(Node):
    """Loads the graph, serves guide/plan_route, publishes the route and graph."""

    def __init__(self):
        super().__init__('route_planner')

        # --- Parameters ---
        # Defaults come from guide_core's dataclasses, so they live in one place.
        # Float parameters must be given as floats on the command line (10.0, not 10).
        defaults = PlannerConfig()
        self.declare_parameter('graph_file', '')
        self.declare_parameter('origin_lat', 51.4988)
        self.declare_parameter('origin_lon', -0.1749)
        self.declare_parameter('route_frame', 'enu')   # frame of paths and markers
        self.declare_parameter('max_snap_distance', defaults.max_snap_distance)
        self.declare_parameter('walkway_snap_distance', defaults.walkway_snap_distance)
        self.declare_parameter('goal_snap_slack', defaults.goal_snap_slack)
        self.declare_parameter('goal_snap_weight', defaults.goal_snap_weight)
        self.declare_parameter('simplify_tolerance', defaults.simplify_tolerance)
        self.declare_parameter('crossing_half_width', defaults.crossing_half_width)
        self.declare_parameter('island_max_length', defaults.island_max_length)
        self.declare_parameter('junction_half_width', defaults.junction_half_width)
        self.declare_parameter('side_street_half_width', defaults.side_street_half_width)
        for f in fields(CostConfig):
            self.declare_parameter(f.name, getattr(defaults.costs, f.name))
        self.declare_parameter('publish_graph', True)

        graph_file = os.path.expanduser(self.get_parameter('graph_file').value)
        self._route_frame = self.get_parameter('route_frame').value
        origin = (self.get_parameter('origin_lat').value, self.get_parameter('origin_lon').value)
        try:
            costs = CostConfig(**{f.name: self.get_parameter(f.name).value
                                  for f in fields(CostConfig)})
            config = PlannerConfig(
                max_snap_distance=self.get_parameter('max_snap_distance').value,
                simplify_tolerance=self.get_parameter('simplify_tolerance').value,
                crossing_half_width=self.get_parameter('crossing_half_width').value,
                island_max_length=self.get_parameter('island_max_length').value,
                junction_half_width=self.get_parameter('junction_half_width').value,
                walkway_snap_distance=self.get_parameter('walkway_snap_distance').value,
                side_street_half_width=self.get_parameter('side_street_half_width').value,
                goal_snap_slack=self.get_parameter('goal_snap_slack').value,
                goal_snap_weight=self.get_parameter('goal_snap_weight').value,
                costs=costs)
        except ValueError as e:
            raise StartupError(f'Invalid planner parameter: {e}') from e

        # --- Load the graph and build the planner ---
        # The graph was downloaded once by fetch_osm_graph; no internet needed here.
        if not graph_file:
            raise StartupError('Parameter graph_file is required (a .graphml made with '
                               '"ros2 run guide_nodes fetch_osm_graph --out <file>").')
        t0 = time.perf_counter()
        try:
            graph = ox.load_graphml(graph_file)
        except Exception as e:  # missing file, corrupt XML, ...
            raise StartupError(f'Could not load graph_file {graph_file!r}: {e}') from e
        try:
            self._planner = RoutePlanner(graph, *origin, config)
        except ValueError as e:
            raise StartupError(f'Graph in {graph_file!r} is unusable: {e}') from e

        # --- Publishers (latched, see LATCHED_QOS) ---
        # guide/planned_route: the most recent successful plan, for RViz.
        # guide/osm_graph: the whole walking graph, published once.
        self._route_pub = self.create_publisher(Path, 'guide/planned_route', LATCHED_QOS)
        self._graph_pub = self.create_publisher(MarkerArray, 'guide/osm_graph', LATCHED_QOS)

        if self.get_parameter('publish_graph').value:
            markers, counts = build_graph_markers(
                graph, origin, self._route_frame, self.get_clock().now().to_msg())
            self._graph_pub.publish(markers)
            self.get_logger().info('Published guide/osm_graph: ' +
                                   ', '.join(f'{v} {k}' for k, v in counts.items()))

        # --- Service ---
        # A service is a request/response call: a client sends a PlanRoute
        # request, _on_plan_route fills in the response, which goes back to
        # that client only. Single-threaded executor: one request at a time.
        self._service = self.create_service(PlanRoute, 'guide/plan_route', self._on_plan_route)
        # One startup summary, logged once the service is up (time includes the RViz markers).
        self.get_logger().info(
            f'Loaded {graph_file}: {graph.number_of_nodes()} nodes, '
            f'{graph.number_of_edges()} directed edges, '
            f'{self._planner.usable_node_count} usable nodes, '
            f'{self._planner.island_node_count} pedestrian island nodes '
            f'({time.perf_counter() - t0:.1f} s)')

    def _on_plan_route(self, request, response):
        """Plan a route; a PlanningError becomes success=false (never a crash)."""
        t0 = time.perf_counter()
        try:
            result = self._planner.plan(request.start_latitude, request.start_longitude,
                                        request.goal_latitude, request.goal_longitude)
        except PlanningError as e:
            response.success = False
            response.message = str(e)
            self.get_logger().warn(
                f'Plan failed ({(time.perf_counter() - t0) * 1000:.0f} ms): {e}')
            return response

        stamp = self.get_clock().now().to_msg()
        # The Path holds exactly result.route's points (no rounding or
        # re-simplifying), so crossing s values measured along result.route
        # stay valid for anyone who rebuilds a Route from this Path.
        response.path = self._to_path(result.route, stamp)
        response.latitudes = [lat for lat, _ in result.latlons]
        response.longitudes = [lon for _, lon in result.latlons]
        response.length = result.length
        response.start_snap_distance = result.start_snap_distance
        response.goal_snap_distance = result.goal_snap_distance
        response.crossings = [self._to_crossing_msg(c) for c in result.crossings]

        counts = Counter(c.crossing_type for c in result.crossings)
        islands = sum(1 for c in result.crossings if c.has_island)
        response.message = (
            f'Route {result.length:.1f} m, {len(result.route.points)} points, '
            f'{len(result.crossings)} crossings ({counts[CrossingType.SIGNALLED]} signalled, '
            f'{counts[CrossingType.UNSIGNALLED]} unsignalled, '
            f'{counts[CrossingType.UNKNOWN]} unknown, {islands} with an island)')
        # Reasons to doubt the route (goal snapped to a road, long detour,
        # tunnel) are returned two ways; the route is still returned:
        # - response.warnings: one string per warning, for programs (mission manager);
        # - appended to the message as " WARNING: ...", for people (CLI, logs, saved files).
        response.warnings = list(result.warnings)
        for warning in result.warnings:
            response.message += f' WARNING: {warning}.'
        if result.warnings:
            self.get_logger().warning('; '.join(result.warnings))
        response.success = True

        self._route_pub.publish(response.path)
        # Timing and snaps first, so nothing is glued after the message's own punctuation.
        self.get_logger().info(
            f'Planned in {(time.perf_counter() - t0) * 1000:.0f} ms '
            f'(snapped {result.start_snap_distance:.1f} m / '
            f'{result.goal_snap_distance:.1f} m): {response.message}')
        return response

    def _to_path(self, route, stamp):
        """Convert a guide_core Route into a nav_msgs/Path in the route (enu) frame.

        Each pose's orientation is the heading of the segment starting at it
        (the last pose keeps the final segment's heading). Headings follow
        REP 103: 0 = east, counter-clockwise positive.
        """
        path = Path()
        path.header.frame_id = self._route_frame
        path.header.stamp = stamp
        points = route.points
        for i, (x, y) in enumerate(points):
            j = min(i, len(points) - 2)  # segment j runs from points[j] to points[j + 1]
            yaw = math.atan2(points[j + 1][1] - points[j][1], points[j + 1][0] - points[j][0])
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        return path

    @staticmethod
    def _to_crossing_msg(crossing):
        """Copy a guide_core RouteCrossing into a guide_interfaces RouteCrossing."""
        msg = RouteCrossing()
        msg.start_s = float(crossing.start_s)
        msg.end_s = float(crossing.end_s)
        msg.crossing_type = CROSSING_TYPE_TO_MSG[crossing.crossing_type]
        msg.crossed_road = crossing.crossed_road
        msg.tactile_paving = bool(crossing.tactile_paving)
        msg.has_island = bool(crossing.has_island)
        return msg


def main(args=None):
    """Start the node; exit with code 1 if it cannot start."""
    rclpy.init(args=args)
    try:
        node = RoutePlannerNode()
    except StartupError as e:
        rclpy.logging.get_logger('route_planner').fatal(str(e))
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C: normal shutdown
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
