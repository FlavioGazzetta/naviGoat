"""Route follower node: follows a route and chooses goals for Nav2.

Gets a route (and its road crossings) either from a YAML file at startup
(route_file parameter, route id 0) or at runtime from the mission manager
(guide/set_route service, route id > 0; guide/clear_route removes it).
Projects each guide/pose onto the current route and publishes progress
(guide/route_progress), the route itself (guide/route) and the next goal for
the goal sender (guide/next_goal). The robot stops before every crossing
until the user's go arrives on the guide/confirm_crossing service. All
geometry is done in guide_core.route_follower; this node only converts
messages.

Frames (Step 8): the route, guide/pose, progress and goals are all in the
route frame (route_frame, default 'enu': metres east / north of the configured
origin). The goal sender converts goals into Nav2's map frame. Route files
saved before Step 8 say 'map', which then meant the same enu coordinates, so
they are read with route_file_frame (default 'map').

Uncertainty (Step 8): each guide/pose carries the localizer's covariance; its
sigma moves the next crossing's stop earlier and widens the arrival tolerance
(see guide_core.route_follower). Off-route detection uses the poses' stamps as
its clock and is reported in guide/route_progress (off_route).
"""

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from guide_core.angles import yaw_to_quaternion
from guide_core.crossings import crossings_from_dict
from guide_core.crossings import RouteCrossing as CoreCrossing
from guide_core.route import InvalidRouteError, Route, route_from_dict
from guide_core.route_follower import FollowerConfig, Goal, RouteFollower
from guide_core.walk_costs import CrossingType
from guide_interfaces.msg import NavGoalStatus, RouteCrossing, RouteProgress
from guide_interfaces.srv import SetRoute
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
import yaml

# "Latched" QoS for topics that describe current state and change rarely.
# RELIABLE: messages are re-sent until received (no silent drops).
# TRANSIENT_LOCAL: the publisher keeps its last message and delivers it to
# subscribers that connect later (e.g. RViz started after this node).
# depth=1: only the latest message is kept. A subscriber must also request
# TRANSIENT_LOCAL to receive the stored message.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# RouteCrossing.msg's uint8 constants -> guide_core's CrossingType.
MSG_TO_CROSSING_TYPE = {
    RouteCrossing.SIGNALLED: CrossingType.SIGNALLED,
    RouteCrossing.UNSIGNALLED: CrossingType.UNSIGNALLED,
    RouteCrossing.UNKNOWN: CrossingType.UNKNOWN,
}


class RouteLoadError(Exception):
    """The route file or the follower parameters could not be used."""


def set_yaw(pose_msg, yaw: float) -> None:
    """Write a heading into a geometry_msgs/Pose orientation (a quaternion)."""
    # ROS stores orientation as a quaternion (x, y, z, w); for a pure
    # heading only z and w are non-zero.
    q = yaw_to_quaternion(yaw)
    pose_msg.orientation.x, pose_msg.orientation.y, \
        pose_msg.orientation.z, pose_msg.orientation.w = q


class RouteFollowerNode(Node):
    """ROS wrapper around guide_core.route_follower.RouteFollower.

    Holds at most one RouteFollower at a time (None = no route). A new route
    replaces it with a brand-new RouteFollower; nothing of the old route is
    kept.
    """

    def __init__(self):
        super().__init__('route_follower')

        # Parameters. declare_parameter fixes each parameter's name, type and
        # default; values can be overridden with --ros-args -p name:=value.
        # Floats must be given as floats (20.0, not 20). Distance defaults
        # come from FollowerConfig so they are defined in one place.
        defaults = FollowerConfig()
        self.declare_parameter('route_file', '')   # optional since Step 7
        # route_frame: the frame of routes, poses, progress and goals (Step 8: enu).
        # route_file_frame: the frame written in route files (pre-Step-8 files say
        # 'map', which then meant these same enu coordinates).
        self.declare_parameter('route_frame', 'enu')
        self.declare_parameter('route_file_frame', 'map')
        self.declare_parameter('lookahead', defaults.lookahead)
        self.declare_parameter('corner_angle_deg', 30.0)  # degrees, for humans
        self.declare_parameter('corner_release_distance', defaults.corner_release_distance)
        self.declare_parameter('goal_update_distance', defaults.goal_update_distance)
        self.declare_parameter('arrival_tolerance', defaults.arrival_tolerance)
        self.declare_parameter('window_behind', defaults.window_behind)
        self.declare_parameter('window_ahead', defaults.window_ahead)
        self.declare_parameter('crossing_stop_margin', defaults.crossing_stop_margin)
        self.declare_parameter('crossing_wait_distance', defaults.crossing_wait_distance)
        # Step 8: uncertainty-aware stops / arrival and off-route detection. The
        # meaning of each value is documented in guide_core FollowerConfig.
        noise_fields = ('stop_sigma_factor', 'stop_sigma_included', 'stop_sigma_step',
                        'stop_latch_distance', 'arrival_sigma_factor', 'off_route_distance',
                        'back_on_route_distance', 'off_route_time', 'off_route_max_sigma')
        for name in noise_fields:
            self.declare_parameter(name, getattr(defaults, name))

        self._route_frame = self.get_parameter('route_frame').value
        self._route_file_frame = self.get_parameter('route_file_frame').value
        route_file = self.get_parameter('route_file').value

        # Follower settings: the same for every route this node follows.
        try:
            self._config = FollowerConfig(
                lookahead=self.get_parameter('lookahead').value,
                corner_angle=math.radians(self.get_parameter('corner_angle_deg').value),
                corner_release_distance=self.get_parameter('corner_release_distance').value,
                goal_update_distance=self.get_parameter('goal_update_distance').value,
                arrival_tolerance=self.get_parameter('arrival_tolerance').value,
                window_behind=self.get_parameter('window_behind').value,
                window_ahead=self.get_parameter('window_ahead').value,
                crossing_stop_margin=self.get_parameter('crossing_stop_margin').value,
                crossing_wait_distance=self.get_parameter('crossing_wait_distance').value,
                **{name: self.get_parameter(name).value for name in noise_fields},
            )
        except ValueError as e:
            raise RouteLoadError(f'invalid follower parameter: {e}') from e

        # Current route. _follower is None while there is no route;
        # _route_id is 0 for the startup file (or no route), > 0 for a route
        # from guide/set_route, and is copied into every RouteProgress so the
        # mission manager can ignore messages about an older route.
        self._follower = None
        self._route_id = 0
        self._reset_route_state()

        # Publishers. Route and next goal are state, so latched; progress is
        # a 5 Hz stream, so default reliable QoS with a queue of 10.
        self._route_pub = self.create_publisher(Path, 'guide/route', LATCHED_QOS)
        self._goal_pub = self.create_publisher(PoseStamped, 'guide/next_goal', LATCHED_QOS)
        self._progress_pub = self.create_publisher(RouteProgress, 'guide/route_progress', 10)

        # Subscriber to our own localizer's pose. It is published with default
        # reliable QoS, so we subscribe with the default too (depth 10).
        self.create_subscription(
            PoseWithCovarianceStamped, 'guide/pose', self._on_pose, 10)

        # Subscriber to the goal sender's status (corner safety net). It is
        # published latched, so we subscribe with the same LATCHED_QOS:
        # TRANSIENT_LOCAL so we still get the latest status if we start later,
        # RELIABLE so no status change is silently dropped.
        self.create_subscription(
            NavGoalStatus, 'guide/goal_status', self._on_goal_status, LATCHED_QOS)

        # Service for the user's "go" at a crossing. A service is a
        # request/response call (unlike a topic, the caller gets an answer).
        # std_srvs/Trigger has an empty request and a response with
        # 'bool success' and 'string message'. Called from the CLI in sim:
        #   ros2 service call /guide/confirm_crossing std_srvs/srv/Trigger
        # and by the mission manager (the user's go button).
        # The single-threaded executor runs this callback between pose
        # callbacks, never at the same time, so no lock is needed.
        self.create_service(Trigger, 'guide/confirm_crossing', self._on_confirm_crossing)

        # Services to change the route at runtime (called by the mission
        # manager; for tests: plan_route ... --send). Same single-threaded
        # rule: a new route never appears in the middle of a pose update.
        self.create_service(SetRoute, 'guide/set_route', self._on_set_route)
        self.create_service(Trigger, 'guide/clear_route', self._on_clear_route)

        # Optional startup route from a file (route id 0), as before Step 7.
        if route_file:
            route, crossings = self._load_route(route_file)
            try:
                follower = RouteFollower(route, self._config, crossings)
            except InvalidRouteError as e:   # includes CrossingError
                raise RouteLoadError(f'invalid crossings in {route_file!r}: {e}') from e
            self._install(follower, 0, f'route file {route_file}')
        else:
            self._publish_route()  # empty Path: RViz shows no stale route
            self.get_logger().info('No route_file: waiting for a route on guide/set_route')

    # --- helpers ---

    def _reset_route_state(self) -> None:
        """Forget all bookkeeping that belongs to one route."""
        self._arrival_logged = False
        # Index of the crossing whose "Waiting at crossing" line was last
        # logged, so it is logged once per crossing, not on every pose.
        self._waiting_logged = -1
        # Stamp of the last goal published on guide/next_goal. The goal sender
        # copies it into guide/goal_status, so it tells us which goal a status
        # is about (see _on_goal_status). None until the route's first goal,
        # so a status about the previous route's goal can never match.
        self._last_goal_stamp = None
        # Robot's s from the latest pose, for log messages and service replies.
        # None until the first pose has been processed on this route.
        self._last_robot_s = None
        # Last logged stop shift (m) and off-route state, to log only changes.
        self._logged_offset = 0.0
        self._logged_off_route = False

    def _install(self, follower, route_id: int, source: str) -> None:
        """Make `follower` the current route, publish it and log its crossings."""
        replaced = self._follower is not None
        old_id = self._route_id
        self._follower = follower
        self._route_id = route_id
        self._reset_route_state()
        self._publish_route()
        route = follower.route
        self.get_logger().info(
            (f'Replaced route {old_id} with ' if replaced else 'Loaded ')
            + f'route {route_id} ({source}): {len(route.points)} waypoints, '
            f'{route.total_length:.1f} m, '
            f'{len(route.corners(self._config.corner_angle))} corner(s), '
            f'{len(follower.crossings)} crossing(s)')
        for i, (c, stop_s) in enumerate(zip(follower.crossings, follower.stop_points)):
            self.get_logger().info(
                f'  {self._crossing_name(i)}: {self._crossing_kind(c)}, '
                f's={c.start_s:.1f}..{c.end_s:.1f} m, stop at s={stop_s:.1f} m')

    def _load_route(self, route_file: str):
        """Read and validate the route file; return (route, crossings).

        A file without a 'crossings' key gives no crossings. Raises
        RouteLoadError on any problem.
        """
        try:
            with open(route_file) as f:
                data = yaml.safe_load(f)
            route = route_from_dict(data, expected_frame=self._route_file_frame)
            return route, crossings_from_dict(data, route)
        except OSError as e:
            raise RouteLoadError(f'cannot read route file {route_file!r}: {e}') from e
        except yaml.YAMLError as e:
            raise RouteLoadError(f'route file {route_file!r} is not valid YAML: {e}') from e
        except InvalidRouteError as e:   # includes CrossingError
            raise RouteLoadError(f'invalid route in {route_file!r}: {e}') from e

    def _crossing_name(self, index: int) -> str:
        """'crossing 2/4' style name (1-based, as people count)."""
        return f'crossing {index + 1}/{len(self._follower.crossings)}'

    @staticmethod
    def _crossing_kind(crossing) -> str:
        """'signalled, major road' style description of a crossing."""
        return f'{crossing.crossing_type.value}, {crossing.crossed_road} road'

    def _publish_route(self) -> None:
        """Publish the current route as a nav_msgs/Path (latched).

        With no route, an empty Path is published, so RViz (and anyone who
        connects later) does not keep showing an old route.
        """
        # A Path is a header plus a list of PoseStamped. Each waypoint's
        # orientation is the direction of the segment leaving it.
        path = Path()
        path.header.frame_id = self._route_frame
        path.header.stamp = self.get_clock().now().to_msg()
        if self._follower is not None:
            route = self._follower.route
            for s, (x, y) in zip(route.cumulative, route.points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                set_yaw(pose.pose, route.heading_at(s))
                path.poses.append(pose)
        self._route_pub.publish(path)

    # --- route changes at runtime ---

    def _on_set_route(self, request, response):
        """guide/set_route: replace the current route with the one in the request.

        Everything is checked BEFORE the old route is touched: on any problem
        the answer is success=false with the reason and the old route (if
        any) stays as it was.
        """
        try:
            if request.route_id == 0:
                raise ValueError('route_id must be > 0 (0 means "loaded from route_file")')
            frame = request.path.header.frame_id
            if frame != self._route_frame:
                raise ValueError(f"path is in frame '{frame}', expected '{self._route_frame}'")
            # The path's points are used exactly as sent (no rounding), so the
            # crossings' s values, measured by the planner along the same
            # points, line up (checked in Step 7.3).
            route = Route([(p.pose.position.x, p.pose.position.y)
                           for p in request.path.poses])
            crossings = []
            for i, c in enumerate(request.crossings):
                if c.crossing_type not in MSG_TO_CROSSING_TYPE:
                    raise ValueError(f'crossing {i}: unknown crossing_type {c.crossing_type}')
                crossings.append(CoreCrossing(
                    start_s=c.start_s, end_s=c.end_s,
                    crossing_type=MSG_TO_CROSSING_TYPE[c.crossing_type],
                    crossed_road=c.crossed_road, tactile_paving=c.tactile_paving,
                    has_island=c.has_island))
            # The constructor validates, clamps and sorts the crossings.
            follower = RouteFollower(route, self._config, crossings)
        except ValueError as e:   # InvalidRouteError and CrossingError are ValueErrors
            response.success = False
            response.message = f'Route {request.route_id} rejected: {e}'
            self.get_logger().warning(response.message)
            return response

        self._install(follower, request.route_id, 'guide/set_route')
        response.success = True
        response.message = (
            f'Route {request.route_id}: {follower.route.total_length:.1f} m, '
            f'{len(follower.route.points)} points, {len(follower.crossings)} crossings')
        return response

    def _on_clear_route(self, request, response):
        """guide/clear_route: stop following; no more progress or goals.

        Does NOT stop the robot by itself: the goal sender still holds the
        last goal. The mission manager therefore also calls the goal
        sender's guide/cancel.
        """
        if self._follower is None:
            response.success, response.message = True, 'No route to clear'
            return response
        old_id = self._route_id
        self._follower = None
        self._route_id = 0
        self._reset_route_state()
        self._publish_route()
        response.success, response.message = True, f'Route {old_id} cleared'
        self.get_logger().info(response.message)
        return response

    # --- following ---

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Update the follower with a new pose and publish the results."""
        if self._follower is None:
            return  # no route: nothing to follow, no progress to report

        # Only poses in the route frame can be compared with the route; anything
        # else is a configuration error, reported at most every 5 s.
        if msg.header.frame_id != self._route_frame:
            self.get_logger().warning(
                f"ignoring pose in frame '{msg.header.frame_id}', "
                f"expected '{self._route_frame}'",
                throttle_duration_sec=5.0)
            return

        # PoseWithCovarianceStamped nests as msg.pose (with covariance)
        # .pose (the Pose) .position; only x and y are used. The covariance is 36
        # numbers (6x6, row by row, over x y z roll pitch yaw): [0] is var(x) and
        # [7] is var(y). The larger one, square-rooted, is the position sigma.
        position = msg.pose.pose.position
        cov = msg.pose.covariance
        sigma = math.sqrt(max(cov[0], cov[7], 0.0))
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            out = self._follower.update(position.x, position.y, sigma=sigma, now=now)
        except ValueError as e:
            self.get_logger().warning(f'ignoring pose: {e}', throttle_duration_sec=5.0)
            return

        self._last_robot_s = out.s

        # Progress on every pose. The stamp is copied from the pose, so each
        # output can be traced back to the measurement it came from.
        progress = RouteProgress()
        progress.header.stamp = msg.header.stamp
        progress.header.frame_id = self._route_frame
        progress.route_id = self._route_id
        progress.segment_index = out.segment_index
        progress.distance_along = out.s
        progress.distance_remaining = out.distance_remaining
        progress.total_length = out.total_length
        progress.cross_track_error = out.cross_track_error
        progress.arrived = out.arrived
        progress.waiting_at_crossing = out.waiting_at_crossing
        progress.on_crossing = out.on_crossing
        progress.next_crossing_index = out.next_crossing_index
        progress.distance_to_next_crossing = out.distance_to_next_crossing
        progress.off_route = out.off_route
        self._progress_pub.publish(progress)

        # Log changes of the stop shift and of the off-route state (not every pose).
        offset = self._follower.stop_offset
        if offset != self._logged_offset and out.next_crossing_index >= 0:
            self.get_logger().info(
                f'Stop before {self._crossing_name(out.next_crossing_index)} moved '
                f'{offset:.1f} m earlier (position uncertainty {sigma:.1f} m): '
                f'now at s={self._follower.active_stop_s:.1f} m')
        self._logged_offset = offset
        if out.off_route != self._logged_off_route:
            self._logged_off_route = out.off_route
            if out.off_route:
                self.get_logger().warning(
                    f'Off the route: {abs(out.cross_track_error):.1f} m from it '
                    f'(robot at s={out.s:.1f} m)')
            else:
                self.get_logger().info(f'Back on the route (robot at s={out.s:.1f} m)')

        if out.new_goal is not None:
            self._publish_goal(out.new_goal, msg.header.stamp, out.s)

        if out.waiting_at_crossing and out.next_crossing_index != self._waiting_logged:
            self._waiting_logged = out.next_crossing_index
            self.get_logger().info(
                f'Waiting at {self._crossing_name(out.next_crossing_index)} '
                f'({self._crossing_kind(self._follower.crossings[out.next_crossing_index])}) '
                f'for the user to confirm (robot at s={out.s:.1f} m)')

        if out.arrived and not self._arrival_logged:
            self.get_logger().info(
                f'Arrived (route {self._route_id}): {out.distance_remaining:.2f} m '
                'remaining along the route')
            self._arrival_logged = True

    def _publish_goal(self, goal: Goal, stamp, robot_s: float) -> None:
        """Publish a new goal for the goal sender (latched)."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._route_frame
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        set_yaw(msg.pose, goal.yaw)
        self._goal_pub.publish(msg)
        self._last_goal_stamp = stamp

        if goal.is_crossing_stop:
            kind = 'crossing stop'
        elif goal.is_crossing_exit:
            kind = 'far kerb, final' if goal.is_final else 'far kerb'
        elif goal.is_final:
            kind = 'final'
        elif goal.is_corner:
            kind = 'corner'
        else:
            kind = 'lookahead'
        self.get_logger().info(
            f'New goal ({kind}) at s={goal.s:.1f} m: x={goal.x:.1f} y={goal.y:.1f} '
            f'yaw={math.degrees(goal.yaw):.0f} deg (robot at s={robot_s:.1f} m)')

    def _on_goal_status(self, msg: NavGoalStatus) -> None:
        """Safety net: Nav2 reports the current goal reached.

        Nav2 stops ~0.25 m short of a goal and reports SUCCEEDED. If that
        happens before the robot is within corner_release_distance of a held
        corner (or far kerb), the robot would wait there forever. Here a
        SUCCEEDED for our current corner / far-kerb goal releases it; the next
        pose then chooses a goal past it (goals are only ever chosen in
        _on_pose). A SUCCEEDED for a crossing stop goal NEVER releases the
        crossing (only the user can): it only makes the robot count as
        waiting, even if GPS puts it a little short of the stop.
        """
        # Only successes can release a corner. PENDING/ACTIVE/ABORTED/... are
        # ignored here (the mission manager uses ABORTED/REJECTED).
        if self._follower is None or msg.status != NavGoalStatus.SUCCEEDED:
            return

        goal = msg.goal
        if goal.header.frame_id != self._route_frame:
            self.get_logger().warning(
                f"ignoring goal status in frame '{goal.header.frame_id}', "
                f"expected '{self._route_frame}'",
                throttle_duration_sec=5.0)
            return

        # Layer 1 against stale messages: the status must be about the goal we
        # published last. Each goal's stamp is unique (copied from a pose), and
        # the goal sender sends it back unchanged. ROS message objects compare
        # field by field with ==, so this compares sec and nanosec. A status
        # left over from an older goal, an older route, or from before this
        # node started, fails here.
        if self._last_goal_stamp is None or goal.header.stamp != self._last_goal_stamp:
            return

        # Layer 2 (guide_core): position matches the current goal, and it is
        # a corner that is not yet released. Read the goal first for the log.
        current = self._follower.goal
        position = goal.pose.position
        robot_s = self._last_robot_s if self._last_robot_s is not None else math.nan
        if self._follower.confirm_goal_reached(position.x, position.y):
            what = 'Far kerb' if current.is_crossing_exit else 'Corner'
            self.get_logger().info(
                f'{what} at s={current.s:.1f} m confirmed reached by Nav2: released '
                f'(robot at s={robot_s:.1f} m)')
        elif current is not None and current.is_crossing_stop:
            self.get_logger().info(
                f'Crossing stop at s={current.s:.1f} m reached (Nav2): waiting for '
                f'the user, not released (robot at s={robot_s:.1f} m)')

    def _on_confirm_crossing(self, request, response):
        """guide/confirm_crossing: the user's go at the crossing the robot waits at.

        The follower accepts it only while the robot is waiting at a crossing;
        a go given anywhere else is rejected and not remembered. The new goal
        (towards the far kerb) is sent with the next pose, within ~0.2 s.
        """
        # The request is empty (std_srvs/Trigger); fill in the response and
        # return it.
        follower = self._follower
        if follower is None:
            response.success, response.message = False, 'No route loaded'
            self.get_logger().info(f'Crossing confirm rejected: {response.message}')
            return response
        index = follower.next_crossing_index
        if follower.arrived:
            response.success, response.message = False, 'Already arrived'
        elif self._last_robot_s is None:
            response.success, response.message = False, 'No pose received yet'
        elif index < 0:
            response.success, response.message = False, 'No crossings left on this route'
        elif follower.confirm_crossing():
            crossing = follower.crossings[index]
            response.success = True
            response.message = (
                f'{self._crossing_name(index).capitalize()} '
                f'({self._crossing_kind(crossing)}) confirmed: crossing to far kerb '
                f'at s={crossing.end_s:.1f} m')
            self.get_logger().info(
                f'{self._crossing_name(index).capitalize()} confirmed by user: released '
                f'(robot at s={self._last_robot_s:.1f} m)')
        else:
            crossing = follower.crossings[index]
            response.success = False
            response.message = (
                f'Not at a crossing: next is {self._crossing_name(index)} in '
                f'{crossing.start_s - self._last_robot_s:.1f} m '
                f'(stop point at s={follower.stop_points[index]:.1f} m)')
        if not response.success:
            self.get_logger().info(f'Crossing confirm rejected: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)

    # A node that cannot load its startup route exits with a FATAL log
    # instead of running and silently publishing nothing.
    try:
        node = RouteFollowerNode()
    except RouteLoadError as e:
        get_logger('route_follower').fatal(str(e))
        rclpy.shutdown()
        return 1

    # spin() runs callbacks until Ctrl-C. A single-threaded executor is
    # enough: the callbacks (pose, goal status, services) never block, and
    # running one at a time means they never change the follower at the
    # same moment.
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
