"""Mission manager node: one command takes the user to a destination.

Serves the guide/navigate_to action (guide_interfaces/action/NavigateTo).
All decisions are made by guide_core.mission.Mission; this node only:
- feeds it events (the goal, the robot's position, the planner's and the
  follower's answers, guide/route_progress, guide/paused, guide/goal_status),
- carries out the commands it returns (call guide/plan_route, guide/set_route,
  guide/clear_route, the goal sender's guide/pause, guide/resume, guide/cancel,
  the follower's guide/confirm_crossing, and finish the action),
- publishes action feedback and the latched guide/mission_state,
- serves the user's two inputs (guide/user_go, guide/user_pause) and
  guide/resolve_destination (look a name up without starting a trip).

Names not in the curated list are looked up with the geocoder node
(guide/geocode: Nominatim online + its on-disk cache), both for trips
(Mission's LookUpOnline command) and for guide/resolve_destination.

Nothing here ever blocks: every service call is asynchronous and its answer
arrives later as one more event (a blocking call inside a callback would
deadlock this single-threaded node).

Step 8: guide/pose is in the enu frame (metres east / north of the origin),
so it converts straight to lat/lon. guide/localization_status (GOOD /
DEGRADED / LOST) and guide/route_progress's off_route flag go to Mission, which
reports GPS_DEGRADED / POSITION_LOST and plans a new route when needed.
"""

import os
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from guide_core.destinations import (
    Destination, DestinationError, destinations_from_dict, MatchConfig, MatchType, Outcome,
    resolve,
)
from guide_core.geo import enu_to_latlon
from guide_core.geocoder import Status as GeocodeStatus
from guide_core.mission import (
    CallPlanner, ClearRoute, ConfirmCrossing, CrossingInfo, Finish, LookUpOnline, Mission,
    MissionConfig, MissionError, MissionGoal, NavStatus, Pause, PlanSummary, Progress, Resume,
    SendRoute, StopRobot,
)
from guide_core.online_resolve import resolve_online
from guide_interfaces.action import NavigateTo
from guide_interfaces.msg import (
    DestinationCandidate, LocalizationStatus, MissionState, NavGoalStatus, RouteCrossing,
    RouteProgress,
)
from guide_interfaces.srv import Geocode, PlanRoute, ResolveDestination, SetRoute
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import yaml

# Latched QoS, as in the other nodes: RELIABLE + TRANSIENT_LOCAL, depth 1.
# Needed to receive guide/paused and guide/goal_status, which are latched.
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

# RouteCrossing.msg's uint8 constants -> the words the mission uses in messages.
CROSSING_KIND = {
    RouteCrossing.SIGNALLED: 'signalled',
    RouteCrossing.UNSIGNALLED: 'unsignalled',
    RouteCrossing.UNKNOWN: 'unknown',
}


# guide_core's resolve() outcomes / match types -> the srv / msg constants.
OUTCOME_TO_MSG = {
    Outcome.UNIQUE: ResolveDestination.Response.UNIQUE,
    Outcome.AMBIGUOUS: ResolveDestination.Response.AMBIGUOUS,
    Outcome.NOT_FOUND: ResolveDestination.Response.NOT_FOUND,
}
MATCH_TYPE_TO_MSG = {
    MatchType.EXACT: DestinationCandidate.EXACT,
    MatchType.FUZZY: DestinationCandidate.FUZZY,
}

# MissionState constants -> names, for log lines.
STATE_NAMES = {
    MissionState.IDLE: 'IDLE', MissionState.PLANNING: 'PLANNING',
    MissionState.READY: 'READY', MissionState.NAVIGATING: 'NAVIGATING',
    MissionState.WAITING_AT_CROSSING: 'WAITING_AT_CROSSING',
    MissionState.PAUSED: 'PAUSED', MissionState.ARRIVED: 'ARRIVED',
    MissionState.FAILED: 'FAILED', MissionState.GPS_DEGRADED: 'GPS_DEGRADED',
    MissionState.POSITION_LOST: 'POSITION_LOST', MissionState.REROUTING: 'REROUTING',
}


class StartupError(Exception):
    """The node cannot start (bad parameter or destination file)."""


def stamp_to_sec(stamp):
    """Convert a builtin_interfaces/Time to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class MissionManagerNode(Node):
    """ROS wrapper around guide_core.mission.Mission."""

    def __init__(self):
        super().__init__('mission_manager')

        # --- Parameters ---
        # Defaults of the mission behaviour come from MissionConfig (one place).
        # Floats must be given as floats on the command line (2.0, not 2).
        defaults = MissionConfig()
        self.declare_parameter('destinations_file', '')
        self.declare_parameter('origin_lat', 51.4988)
        self.declare_parameter('origin_lon', -0.1749)
        self.declare_parameter('max_pose_age', 2.0)        # s; older = "no position"
        self.declare_parameter('feedback_period', 1.0)     # s between routine feedbacks
        self.declare_parameter('max_nav_failures', defaults.max_nav_failures)
        self.declare_parameter('failure_reset_distance', defaults.failure_reset_distance)
        self.declare_parameter('min_go_interval', defaults.min_go_interval)
        self.declare_parameter('snap_report_distance', defaults.snap_report_distance)
        # Names not in the curated list: ask the geocoder node (true), or
        # answer "I don't know that place" at once (false). Whether the
        # geocoder may go online is ITS parameter (online_enabled).
        self.declare_parameter('use_geocoder', True)
        # True: several places with the name ("pret") -> go to the nearest and say so.
        self.declare_parameter('pick_nearest', False)
        self.declare_parameter('geocoder_timeout', 8.0)  # s; then "couldn't search"

        self._origin = (self.get_parameter('origin_lat').value,
                        self.get_parameter('origin_lon').value)
        self._max_pose_age = float(self.get_parameter('max_pose_age').value)
        self._feedback_period = float(self.get_parameter('feedback_period').value)
        self._use_geocoder = bool(self.get_parameter('use_geocoder').value)
        self._geocoder_timeout = float(self.get_parameter('geocoder_timeout').value)
        if not self._geocoder_timeout > 0.0:
            raise StartupError(f'geocoder_timeout must be > 0, got {self._geocoder_timeout}')
        try:
            config = MissionConfig(
                max_nav_failures=self.get_parameter('max_nav_failures').value,
                failure_reset_distance=self.get_parameter('failure_reset_distance').value,
                min_go_interval=self.get_parameter('min_go_interval').value,
                snap_report_distance=self.get_parameter('snap_report_distance').value,
                online_lookup=self._use_geocoder,
                pick_nearest=bool(self.get_parameter('pick_nearest').value))
        except ValueError as e:
            raise StartupError(f'Invalid mission parameter: {e}') from e
        destinations = self._load_destinations(self.get_parameter('destinations_file').value)
        self._destinations = destinations          # also used by guide/resolve_destination
        # With pick_nearest, more matches are worth having: the nearest may not be
        # among the first few the map search returns.
        match_config = MatchConfig(max_candidates=20) if config.pick_nearest else None
        self._mission = Mission(destinations, config, match_config=match_config)

        # --- State kept by this node (plumbing only; decisions are in Mission) ---
        # _pose: (x, y, receive time) of the latest guide/pose, for planning.
        # _plan_response: the latest successful PlanRoute answer; its path and
        #   crossings are what SendRoute hands to the follower.
        # _progress: latest guide/route_progress for the current route (feedback).
        # _goal_handle / _finish: the running NavigateTo goal and the Future its
        #   execute callback waits on; a Finish command completes it.
        self._pose = None
        self._plan_response = None
        self._progress = None
        self._goal_handle = None
        self._finish = None
        self._started_at = 0.0
        self._last_feedback = 0.0
        # Mission.version last published on guide/mission_state (-1 = never).
        self._published_version = -1

        # One reentrant callback group for everything that awaits (the action's
        # execute, guide/resolve_destination) and for the geocoder client they
        # wait on. In rclpy a callback holds its group until it finishes, even
        # while it awaits; in the default (mutually exclusive) group the answer
        # it waits for could then never run: a deadlock. Single-threaded
        # executor: callbacks still run one at a time, never in parallel.
        self._reentrant = ReentrantCallbackGroup()

        # --- Service clients (all called asynchronously, see _execute) ---
        # create_client(type, name): the name must match the server's exactly.
        self._plan_client = self.create_client(PlanRoute, 'guide/plan_route')
        self._set_route_client = self.create_client(SetRoute, 'guide/set_route')
        self._clear_route_client = self.create_client(Trigger, 'guide/clear_route')
        self._confirm_client = self.create_client(Trigger, 'guide/confirm_crossing')
        self._pause_client = self.create_client(Trigger, 'guide/pause')
        self._resume_client = self.create_client(Trigger, 'guide/resume')
        self._stop_client = self.create_client(Trigger, 'guide/cancel')
        self._geocode_client = self.create_client(Geocode, 'guide/geocode',
                                                  callback_group=self._reentrant)

        # --- Subscriptions (events for Mission) ---
        # guide/pose and guide/route_progress are ordinary reliable streams;
        # guide/paused and guide/goal_status are latched, so we subscribe with
        # LATCHED_QOS to also get their current value when we start.
        self.create_subscription(PoseWithCovarianceStamped, 'guide/pose', self._on_pose, 10)
        self.create_subscription(RouteProgress, 'guide/route_progress', self._on_progress, 10)
        self.create_subscription(Bool, 'guide/paused', self._on_paused, LATCHED_QOS)
        self.create_subscription(NavGoalStatus, 'guide/goal_status', self._on_goal_status,
                                 LATCHED_QOS)
        # The localizer's quality, 2 Hz with default reliable QoS (not latched).
        self.create_subscription(LocalizationStatus, 'guide/localization_status',
                                 self._on_localization, 10)

        # --- Action server ---
        # An action = goal + stream of feedback + final result, and the client
        # can cancel it. goal_callback decides accept/reject, cancel_callback
        # accept/reject of a cancel request; execute_callback runs the goal.
        # It is an "async def": while it awaits, the executor keeps running all
        # the other callbacks of this node (subscriptions, service answers).
        self._action_server = ActionServer(
            self, NavigateTo, 'guide/navigate_to',
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            # Reentrant group (see self._reentrant): the awaiting execute
            # callback must never stop the other callbacks from running.
            callback_group=self._reentrant)

        # --- Mission state topic (latched) ---
        # Republished after every event that changes what the user should
        # know, so a speech / app node that starts later still gets the
        # current state at once.
        self._state_pub = self.create_publisher(MissionState, 'guide/mission_state',
                                                LATCHED_QOS)

        # --- The user's inputs and destination lookup (std_srvs/Trigger:
        # empty request; response 'bool success' + 'string message') ---
        # Called by the pole-handle button / speech later; from the CLI in sim:
        #   ros2 service call /guide/user_go std_srvs/srv/Trigger
        self.create_service(Trigger, 'guide/user_go', self._on_user_go)
        self.create_service(Trigger, 'guide/user_pause', self._on_user_pause)
        # resolve_destination may await the geocoder, hence the reentrant group.
        self.create_service(ResolveDestination, 'guide/resolve_destination',
                            self._on_resolve_destination, callback_group=self._reentrant)

        # Timer: routine feedback every feedback_period while a goal runs
        # (distance updates); changes are published at once by _run.
        self.create_timer(0.2, self._on_timer)
        self._publish_state()  # IDLE, so late subscribers have a state at once

        self.get_logger().info(
            f'Mission manager ready: {len(destinations)} destination(s), action '
            f"'guide/navigate_to', origin {self._origin}, other names "
            + ('looked up with guide/geocode' if self._use_geocoder else 'not looked up'))

    # ---- Startup helpers ------------------------------------------------------------

    def _load_destinations(self, path):
        """Load the curated destination list; empty path = no named destinations."""
        if not path:
            self.get_logger().warning(
                'No destinations_file: only lat/lon goals will work')
            return ()
        path = os.path.expanduser(path)
        try:
            with open(path, encoding='utf-8') as f:
                return destinations_from_dict(yaml.safe_load(f))
        except OSError as e:
            raise StartupError(f'Cannot read destinations_file {path!r}: {e}') from e
        except (yaml.YAMLError, DestinationError) as e:
            raise StartupError(f'Invalid destinations_file {path!r}: {e}') from e

    # ---- Action server callbacks ----------------------------------------------------

    def _on_goal_request(self, goal_request):
        """Accept every goal.

        A goal that cannot run now is ended with BUSY in _execute, so the
        client gets a result with a spoken message.
        """
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle):
        """Handle the client cancelling the goal (e.g. Ctrl+C on ros2 action send_goal).

        Mission.cancel() returns StopRobot / ClearRoute / Finish(CANCELED);
        the Finish completes the Future that _execute is waiting on.
        """
        if goal_handle is not self._goal_handle:
            return CancelResponse.REJECT  # not the running goal (e.g. already ended)
        self.get_logger().info('NavigateTo goal cancel requested')
        self._run(self._mission.cancel())
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        """Run one NavigateTo goal from start to result."""
        goal = goal_handle.request
        result = NavigateTo.Result()

        # Busy: another goal is running (or still waiting for its result).
        if self._goal_handle is not None or not self._mission.can_accept():
            result.error_code = MissionError.BUSY
            result.message = 'Another trip is in progress. Cancel it first.'
            self.get_logger().warning(f'Goal rejected: {result.message}')
            goal_handle.abort()
            return result

        # Translate the ROS goal into guide_core's MissionGoal.
        if goal.target_type == NavigateTo.Goal.TARGET_NAME:
            mission_goal = MissionGoal(destination=goal.destination,
                                       skip_confirmation=goal.skip_confirmation)
            wanted = f"'{goal.destination}'"
        elif goal.target_type == NavigateTo.Goal.TARGET_LATLON:
            mission_goal = MissionGoal(latitude=goal.latitude, longitude=goal.longitude,
                                       skip_confirmation=goal.skip_confirmation)
            wanted = f'({goal.latitude:.7f}, {goal.longitude:.7f})'
        else:
            result.error_code = MissionError.INVALID_GOAL
            result.message = f'Unknown target_type {goal.target_type}.'
            goal_handle.abort()
            return result

        self._goal_handle = goal_handle
        self._finish = Future()
        self._started_at = time.monotonic()
        self._progress = None
        self.get_logger().info(f'New goal: {wanted}, skip_confirmation='
                               f'{goal.skip_confirmation}')

        # Start the mission; the first command is CallPlanner (or a Finish if
        # the goal cannot even start). Everything else happens in callbacks.
        self._run(self._mission.start(mission_goal, self._robot_latlon()))

        # Wait without blocking: "await" hands control back to the executor
        # until a Finish command completes the Future.
        finish = await self._finish

        result.error_code = int(finish.error_code)
        result.message = finish.message
        result.distance_remaining = float(finish.distance_remaining)
        result.elapsed_time = time.monotonic() - self._started_at
        if finish.error_code == MissionError.NONE:
            goal_handle.succeed()
        elif finish.error_code == MissionError.CANCELED and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().info(
            f'Goal finished: {finish.error_code.name}: {finish.message} '
            f'({result.elapsed_time:.0f} s)')
        self._goal_handle = None
        self._finish = None
        return result

    # ---- Events from other nodes ----------------------------------------------------

    def _on_pose(self, msg):
        """Remember the latest map-frame position (used when a goal starts)."""
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, time.monotonic())

    def _robot_latlon(self):
        """Return the robot's (lat, lon), or None if no pose in the last max_pose_age s.

        guide/pose is in the enu frame around the origin (localizer v1 aligns
        GPS with the SLAM map), so it converts straight to lat/lon.
        """
        if self._pose is None:
            return None
        x, y, received = self._pose
        if time.monotonic() - received > self._max_pose_age:
            return None
        return enu_to_latlon(x, y, *self._origin)

    def _on_progress(self, msg):
        """Pass guide/route_progress to Mission (it ignores other routes' progress).

        The robot's position goes along: Mission needs it to plan a new route
        when off_route says the robot has left this one.
        """
        if msg.route_id == self._mission.route_id and msg.route_id != 0:
            self._progress = msg
        self._run(self._mission.on_progress(Progress(
            route_id=msg.route_id, distance_remaining=msg.distance_remaining,
            arrived=msg.arrived, waiting_at_crossing=msg.waiting_at_crossing,
            on_crossing=msg.on_crossing, next_crossing_index=msg.next_crossing_index,
            off_route=msg.off_route), robot_latlon=self._robot_latlon()))

    def _on_localization(self, msg):
        """Pass the localizer's quality (GOOD / DEGRADED / LOST) to Mission."""
        try:
            commands = self._mission.on_localization(msg.quality)
        except ValueError:
            self.get_logger().warning(f'Unknown localization quality {msg.quality}',
                                      throttle_duration_sec=10.0)
            return
        self._run(commands)

    def _on_paused(self, msg):
        self._run(self._mission.on_paused(msg.data))

    def _on_goal_status(self, msg):
        """Pass guide/goal_status to Mission (only ABORTED / REJECTED matter to it)."""
        try:
            status = NavStatus(msg.status)
        except ValueError:
            return
        self._run(self._mission.on_nav_status(status, stamp_to_sec(msg.header.stamp)))

    # ---- Carrying out Mission's commands --------------------------------------------

    def _run(self, commands):
        """Carry out each command; answers come back later as new events."""
        for command in commands:
            if isinstance(command, CallPlanner):
                self._call_planner(command)
            elif isinstance(command, LookUpOnline):
                self._look_up(command)
            elif isinstance(command, SendRoute):
                self._send_route(command)
            elif isinstance(command, Finish):
                if self._finish is not None and not self._finish.done():
                    self._finish.set_result(command)
            elif isinstance(command, ClearRoute):
                self._call_trigger(self._clear_route_client, command)
            elif isinstance(command, StopRobot):
                self._call_trigger(self._stop_client, command)
            elif isinstance(command, Pause):
                self._call_trigger(self._pause_client, command)
            elif isinstance(command, Resume):
                self._call_trigger(self._resume_client, command)
            elif isinstance(command, ConfirmCrossing):
                self._call_trigger(
                    self._confirm_client, command,
                    lambda ok, text: self._mission.on_confirm_result(ok, text))
            else:
                self.get_logger().error(f'Unknown command {command!r}')
        # Every event passes through here, so this is the one place that
        # notices a change and tells the user (topic, feedback, log).
        self._publish_state()

    def _call_planner(self, command):
        """Call guide/plan_route for CallPlanner; the answer goes to Mission."""
        if not self._plan_client.service_is_ready():
            self._run(self._mission.on_plan_result(
                command.plan_id, False, 'the route planner is not running'))
            return
        request = PlanRoute.Request(
            start_latitude=command.start_lat, start_longitude=command.start_lon,
            goal_latitude=command.goal_lat, goal_longitude=command.goal_lon)
        self.get_logger().info(
            f'Planning #{command.plan_id}: ({command.start_lat:.7f}, {command.start_lon:.7f})'
            f' -> ({command.goal_lat:.7f}, {command.goal_lon:.7f})')
        future = self._plan_client.call_async(request)
        future.add_done_callback(lambda f, c=command: self._on_plan_answer(c, f))

    def _on_plan_answer(self, command, future):
        try:
            response = future.result()
        except Exception as e:  # service vanished mid-call
            self._run(self._mission.on_plan_result(command.plan_id, False, str(e)))
            return
        if not response.success:
            self.get_logger().warning(f'Plan #{command.plan_id} failed: {response.message}')
            self._run(self._mission.on_plan_result(command.plan_id, False, response.message))
            return
        self.get_logger().info(f'Plan #{command.plan_id}: {response.message}')
        self._plan_response = response
        summary = PlanSummary(
            length=response.length,
            goal_snap_distance=response.goal_snap_distance,
            crossings=tuple(CrossingInfo(CROSSING_KIND.get(c.crossing_type, 'unknown'),
                                         c.crossed_road, c.has_island)
                            for c in response.crossings),
            warnings=tuple(response.warnings))
        self._run(self._mission.on_plan_result(command.plan_id, True, response.message,
                                               summary))

    def _call_geocoder(self, query, on_done):
        """Call guide/geocode for query; on_done(response, '') or on_done(None, why).

        on_done is called exactly once: with the answer, or with None if the
        geocoder is not running, the call failed, or no answer came within
        geocoder_timeout (a one-shot timer), so nothing can wait forever.
        """
        if not self._geocode_client.service_is_ready():
            on_done(None, 'the geocoder is not running')
            return
        done = False
        timer = None

        def finish(response, why):
            nonlocal done
            if done:
                return              # the answer came after the timeout (or vice versa)
            done = True
            timer.cancel()
            self.destroy_timer(timer)
            on_done(response, why)

        def on_answer(future):
            try:
                finish(future.result(), '')
            except Exception as e:  # service vanished mid-call
                finish(None, f'geocoder call failed: {e}')

        timer = self.create_timer(
            self._geocoder_timeout,
            lambda: finish(None, f'no answer from the geocoder in {self._geocoder_timeout:.0f} s'))
        self._geocode_client.call_async(Geocode.Request(query=query)).add_done_callback(on_answer)

    @staticmethod
    def _destinations_from(response):
        """Convert Geocode response candidates into Destinations (source 'nominatim')."""
        return tuple(Destination(name=c.name, lat=c.latitude, lon=c.longitude, note=c.note,
                                 source=c.source or 'nominatim')
                     for c in response.candidates)

    def _look_up(self, command):
        """Call guide/geocode for LookUpOnline; the answer goes to Mission."""
        self.get_logger().info(f"Looking up #{command.lookup_id} '{command.query}'")

        def done(response, why):
            if response is None:
                self.get_logger().warning(f'Lookup #{command.lookup_id} failed: {why}')
                self._run(self._mission.on_lookup_result(command.lookup_id,
                                                         GeocodeStatus.OFFLINE))
                return
            self.get_logger().info(
                f'Lookup #{command.lookup_id}: {GeocodeStatus(response.status).name}, '
                f'{len(response.candidates)} candidate(s): {response.message}')
            self._run(self._mission.on_lookup_result(
                command.lookup_id, response.status, self._destinations_from(response),
                response.from_cache))

        self._call_geocoder(command.query, done)

    def _send_route(self, command):
        """Call guide/set_route with the last plan for SendRoute; the answer goes to Mission."""
        sent = stamp_to_sec(self.get_clock().now().to_msg())
        if not self._set_route_client.service_is_ready() or self._plan_response is None:
            self._run(self._mission.on_route_set(
                command.route_id, False, 'the route follower is not running', sent))
            return
        request = SetRoute.Request(route_id=command.route_id,
                                   path=self._plan_response.path,
                                   crossings=self._plan_response.crossings)
        future = self._set_route_client.call_async(request)
        future.add_done_callback(
            lambda f, c=command, s=sent: self._on_set_route_answer(c, s, f))

    def _on_set_route_answer(self, command, sent, future):
        try:
            response = future.result()
            ok, message = response.success, response.message
        except Exception as e:
            ok, message = False, str(e)
        self.get_logger().info(f'Route {command.route_id} to follower: '
                               f"{'accepted' if ok else 'REJECTED'}: {message}")
        self._run(self._mission.on_route_set(command.route_id, ok, message, sent))

    def _call_trigger(self, client, command, on_answer=None):
        """Call a std_srvs/Trigger service for a command; report failures to Mission."""
        name = client.srv_name
        if not client.service_is_ready():
            self.get_logger().error(f'{name} not available for {type(command).__name__}')
            self._run(self._mission.on_service_failed(command, f'{name} is not available'))
            return

        def done(future):
            try:
                response = future.result()
            except Exception as e:
                self._run(self._mission.on_service_failed(command, f'{name}: {e}'))
                return
            self.get_logger().info(
                f"{name}: {'OK' if response.success else 'refused'}: {response.message}")
            if on_answer is not None:
                self._run(on_answer(response.success, response.message))

        client.call_async(Trigger.Request()).add_done_callback(done)

    # ---- The user's inputs ----------------------------------------------------------

    def _on_user_go(self, request, response):
        """Handle guide/user_go: start / cross / resume, whatever "go" means right now."""
        reply = self._mission.on_go(time.monotonic())
        self.get_logger().info(f"User go: {'accepted' if reply.accepted else 'ignored'}: "
                               f'{reply.text}')
        self._run(reply.commands)
        response.success, response.message = reply.accepted, reply.text
        return response

    def _on_user_pause(self, request, response):
        """Handle guide/user_pause: stop where we are (always obeyed while moving)."""
        reply = self._mission.on_pause()
        self.get_logger().info(f"User pause: {'accepted' if reply.accepted else 'ignored'}: "
                               f'{reply.text}')
        self._run(reply.commands)
        response.success, response.message = reply.accepted, reply.text
        return response

    async def _on_resolve_destination(self, request, response):
        """Handle guide/resolve_destination: look a name up, ranked, without a trip.

        The same chain as a trip: the curated list, then (if nothing matched)
        the geocoder. "async def" + await: while waiting for the geocoder the
        executor keeps running this node's other callbacks.
        """
        config = MatchConfig(max_candidates=request.max_candidates or 5)
        robot = self._robot_latlon()
        resolution = resolve(request.query, self._destinations, robot_latlon=robot,
                             config=config)
        if (resolution.outcome is Outcome.NOT_FOUND and self._use_geocoder
                and resolution.query):
            answer = Future()   # completed by _call_geocoder's on_done
            self._call_geocoder(resolution.query,
                                lambda r, why: answer.set_result((r, why)))
            geocoded, why = await answer
            if geocoded is None:
                self.get_logger().warning(f"Resolve '{request.query}': {why}")
                status, found, cached = GeocodeStatus.OFFLINE, (), False
            else:
                status = geocoded.status
                found = self._destinations_from(geocoded)
                cached = geocoded.from_cache
            resolution = resolve_online(resolution.query, status, found, robot_latlon=robot,
                                        from_cache=cached, config=config)
        response.outcome = OUTCOME_TO_MSG[resolution.outcome]
        response.message = resolution.message
        for c in resolution.candidates:
            msg = DestinationCandidate()
            msg.name = c.destination.name
            msg.matched_text = c.matched_text
            msg.match_type = MATCH_TYPE_TO_MSG[c.match_type]
            msg.score = float(c.score)
            msg.note = c.destination.note
            msg.latitude = float(c.destination.lat)
            msg.longitude = float(c.destination.lon)
            msg.straight_line_distance = (-1.0 if c.distance is None
                                          else float(c.distance))
            msg.source = c.destination.source
            response.candidates.append(msg)
        self.get_logger().info(f"Resolve '{request.query}': {resolution.outcome.name}: "
                               f'{resolution.message}')
        return response

    # ---- Mission state and feedback -------------------------------------------------

    def _on_timer(self):
        """Send routine feedback (distance updates) every feedback_period."""
        if (self._goal_handle is not None
                and time.monotonic() - self._last_feedback >= self._feedback_period):
            self._publish_feedback(self._mission_state_msg())

    def _publish_state(self):
        """Publish guide/mission_state if the mission's snapshot changed.

        Also sends feedback right away and logs the new message.
        """
        if self._mission.version == self._published_version:
            return
        self._published_version = self._mission.version
        msg = self._mission_state_msg()
        self._state_pub.publish(msg)
        self.get_logger().info(f'[{STATE_NAMES[msg.state]}] {msg.message}')
        if self._goal_handle is not None:
            self._publish_feedback(msg)

    def _publish_feedback(self, state_msg):
        """Publish NavigateTo feedback: the mission state and our route's progress."""
        self._last_feedback = time.monotonic()
        feedback = NavigateTo.Feedback()
        feedback.mission = state_msg
        if self._progress is not None and self._progress.route_id == self._mission.route_id:
            feedback.progress = self._progress
        self._goal_handle.publish_feedback(feedback)

    def _mission_state_msg(self):
        """Convert the mission snapshot into a guide_interfaces/MissionState."""
        snap = self._mission.snapshot()
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = int(snap.state)
        msg.route_id = snap.route_id
        msg.destination_name = snap.destination_name
        msg.destination_note = snap.destination_note
        msg.route_length = float(snap.route_length)
        msg.num_crossings = snap.num_crossings
        msg.summary = snap.summary
        msg.warnings = list(snap.warnings)
        msg.message = snap.message
        return msg


def main(args=None):
    """Start the node; exit with code 1 if it cannot start."""
    rclpy.init(args=args)
    try:
        node = MissionManagerNode()
    except StartupError as e:
        rclpy.logging.get_logger('mission_manager').fatal(str(e))
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
