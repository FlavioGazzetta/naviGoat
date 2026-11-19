r"""GPS bridge node: the only part of the GPS layer the rest of the robot talks to.

The team's nodes speak plain std_msgs topics. This node turns them into our
own interfaces and back:
- /middleman/mode (JSON) "gps" + target  -> a goal on our guide/navigate_to action;
  "stop" or any other mode                -> cancel that goal
- /gps/go (Empty, e.g. the handle button) -> our guide/user_go service
- the action's feedback and result        -> /pete/response (spoken lines),
  /gps/status, /gps/crossing_wait (latched), /middleman/done, /middleman/failed
- once a second, what is running          -> /gps/health (JSON, latched): GPS,
  position, Nav2, our planner and mission manager, the geocoder. While a part
  needed for a trip is missing, a GPS request is refused at once, aloud.

All decisions are made by guide_core.bridge.Bridge; this node only feeds it
events and carries out the commands it returns. Every external topic name
is a parameter, as in the team's middleman.

Try it by hand (the mission manager and the rest of our stack must be running):
  ros2 topic pub -w 1 --once /middleman/mode std_msgs/msg/String
      "data: '{\"mode\": \"gps\", \"target\": \"south ken\"}'"   (one line)
  ros2 topic pub -w 1 --once /gps/go std_msgs/msg/Empty
"""

import json
import math
import time

from geometry_msgs.msg import PoseStamped
from guide_core.bridge import (
    Bridge, CancelMission, CrossingWait, Done, Failed, parse_mode, Say, SendGo, StartMission,
    Status,
)
from guide_core.health import evaluate, HealthInputs
from guide_core.polyline import resample_polyline
from guide_interfaces.action import NavigateTo
from guide_interfaces.msg import LocalizationStatus
from guide_interfaces.srv import Geocode, PlanRoute, ResolveDestination
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

# Latched QoS, as in our other nodes: a subscriber that starts later still
# gets the last value at once (here: whether we are waiting at a crossing).
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

# The team's topics: parameter name -> default. Change them with
# -p name:=value (or in the launch file) if the team renames a topic.
TOPICS = {
    'mode_topic': '/middleman/mode',             # in:  the middleman's mode (JSON)
    'go_topic': '/gps/go',                       # in:  the user's go
    'response_topic': '/pete/response',          # out: what Pete says
    'done_topic': '/middleman/done',             # out: target name on arrival
    'failed_topic': '/middleman/failed',         # out: why we gave up
    'status_topic': '/gps/status',               # out: 'going:x', 'waiting:crossing', ...
    'crossing_wait_topic': '/gps/crossing_wait',  # out: true while waiting at a crossing
    'health_topic': '/gps/health',               # out: what is running (JSON, latched)
    'here_topic': '/gps/here',                   # in:  "GPS, I'm at X": the place the user is at
    'waypoints_topic': '/gps/waypoints',         # out: the route as points every few metres
    'gps_topic': '/gps/fix',                     # in:  the GPS receiver, for the health check
}

# Our own (private) interfaces: the mission manager's action and go service,
# and the services whose presence the health check looks for.
ACTION_NAME = 'guide/navigate_to'
USER_GO_SERVICE = 'guide/user_go'
PLAN_SERVICE = 'guide/plan_route'
GEOCODE_SERVICE = 'guide/geocode'
STATUS_TOPIC = 'guide/localization_status'
RESOLVE_SERVICE = 'guide/resolve_destination'   # a place name -> where it is
ASSUMED_TOPIC = 'guide/assumed_position'        # to the localizer: where the user says we are
ROUTE_TOPIC = 'guide/route'   # the route follower's current route (latched; empty = none)
HEALTH_PERIOD = 1.0   # s between health checks

# Said when the user presses go but our mission manager is not running.
GO_UNAVAILABLE_TEXT = "GPS navigation isn't running."


class GoalSlot:
    """One goal sent to the mission manager, until its result arrives."""

    def __init__(self, goal_id):
        self.goal_id = goal_id
        self.handle = None          # set once the action server accepts the goal
        self.cancel_wanted = False  # cancel asked for before the handle arrived


class GpsBridgeNode(Node):
    """ROS wrapper around guide_core.bridge.Bridge."""

    def __init__(self):
        super().__init__('gps_bridge')

        # --- Parameters ---
        topic = {name: self.declare_parameter(name, default).value
                 for name, default in TOPICS.items()}
        # The middleman's mode name for "GPS take me to ...".
        gps_mode = self.declare_parameter('gps_mode', 'gps').value
        # Nav2's action, only to see whether Nav2 is up (the goal sender uses it).
        # Keep it the same as the goal sender's action_name.
        nav2_action = self.declare_parameter('nav2_action', 'navigate_to_pose').value
        self._gps_topic = topic['gps_topic']
        # True for a robot with no go button: the trip starts once the route is
        # planned and carries on at every road crossing without waiting for a go.
        auto_go = self.declare_parameter('auto_go', False).value
        # Metres between the points published on waypoints_topic.
        self._waypoint_spacing = self.declare_parameter('waypoint_spacing', 5.0).value
        # False: start a trip even when Nav2 is not running. The route is then only
        # published on waypoints_topic, for navigation to drive.
        self._require_nav2 = self.declare_parameter('require_nav2', True).value
        self._bridge = Bridge(gps_mode, auto_go)
        self._route = None            # the route follower's latest route (a Path)
        self._waypoints_sent = False  # True while waypoints_topic holds a route

        # --- Goals in flight ---
        # The mission manager runs one goal at a time and answers a goal sent
        # while another is still ending with BUSY. So a new goal waits in
        # _queued until the previous one (_current) has sent its result.
        self._current = None
        self._queued = None

        # --- Clients of our own interfaces ---
        # An action client sends a goal, receives feedback while it runs and a
        # result at the end. Everything is asynchronous: answers arrive later
        # in callbacks, so this node never blocks.
        self._action_client = ActionClient(self, NavigateTo, ACTION_NAME)
        self._go_client = self.create_client(Trigger, USER_GO_SERVICE)
        # Clients used only to ask "is the server there?" (nothing is ever sent).
        self._nav2_client = ActionClient(self, NavigateToPose, nav2_action)
        self._plan_client = self.create_client(PlanRoute, PLAN_SERVICE)
        self._geocode_client = self.create_client(Geocode, GEOCODE_SERVICE)
        self._resolve_client = self.create_client(ResolveDestination, RESOLVE_SERVICE)

        # --- What the health check has heard (times in seconds, None = never) ---
        self._gps_message_time = None
        self._gps_fix_time = None
        self._status = None
        self._status_time = None
        # The localizer starts at LOST and needs a few good seconds to say GOOD:
        # until it has once been usable, LOST means "settling", not "lost".
        self._ever_localised = False
        self._health_text = ''

        # --- Publishers (the team's topics) ---
        self._response_pub = self.create_publisher(String, topic['response_topic'], 10)
        self._done_pub = self.create_publisher(String, topic['done_topic'], 10)
        self._failed_pub = self.create_publisher(String, topic['failed_topic'], 10)
        self._status_pub = self.create_publisher(String, topic['status_topic'], 10)
        self._crossing_pub = self.create_publisher(Bool, topic['crossing_wait_topic'],
                                                   LATCHED_QOS)
        self._crossing_pub.publish(Bool(data=False))  # a known value from the start
        self._health_pub = self.create_publisher(String, topic['health_topic'], LATCHED_QOS)
        self._waypoints_pub = self.create_publisher(Path, topic['waypoints_topic'], LATCHED_QOS)
        self._waypoints_pub.publish(Path())  # no trip yet: an empty path
        self._assumed_pub = self.create_publisher(String, ASSUMED_TOPIC, 10)

        # --- Subscriptions (the team's topics) ---
        self.create_subscription(String, topic['mode_topic'], self._on_mode, 10)
        self.create_subscription(Empty, topic['go_topic'], self._on_go, 10)
        self.create_subscription(String, topic['here_topic'], self._on_here, 10)
        # For the health check: the GPS receiver (sensor QoS, as its driver
        # publishes with) and the localizer's status (2 Hz, reliable).
        self.create_subscription(NavSatFix, self._gps_topic, self._on_gps_fix,
                                 qos_profile_sensor_data)
        self.create_subscription(LocalizationStatus, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(Path, ROUTE_TOPIC, self._on_route, LATCHED_QOS)
        self.create_timer(HEALTH_PERIOD, self._check_health)

        self.get_logger().info(
            f"GPS bridge ready: mode '{gps_mode}' on {topic['mode_topic']}, "
            f"go on {topic['go_topic']}, goals to {ACTION_NAME}")
        self.get_logger().info(
            f"Waypoints every {self._waypoint_spacing:g} m on {topic['waypoints_topic']}; "
            + ('auto go: trips start and cross without a go' if auto_go else 'waiting for go'))

    # ---- The team's inputs ----------------------------------------------------------

    def _on_mode(self, msg):
        """Pass a /middleman/mode message to the bridge."""
        if parse_mode(msg.data) is None:
            self.get_logger().warning(f'Not a JSON object on the mode topic: {msg.data!r}')
            return
        self._run(self._bridge.on_mode(msg.data))

    def _on_go(self, msg):
        """Pass the user's go to the bridge (ignored when no GPS trip is running)."""
        self._run(self._bridge.on_go())

    # ---- "GPS, I'm at X" -------------------------------------------------------------

    def _on_here(self, msg):
        """Look up the place the user says they are at, and tell the localizer.

        The look-up is the one a destination gets (our list, then OpenStreetMap).
        The localizer keeps the place on disk and uses it while there is no real
        position; a real GPS position always wins.
        """
        place = ' '.join(msg.data.split())
        if not place:
            return
        if self._gps_fix_time is not None and self._now() - self._gps_fix_time < 10.0:
            self._say('The GPS already knows where we are.')   # a stated place never beats a fix
            return
        if not self._resolve_client.service_is_ready():
            self._say(f"I can't look up {place}: GPS navigation isn't running.")
            return
        request = ResolveDestination.Request()
        request.query = place
        request.max_candidates = 1

        def done(future):
            try:
                response = future.result()
            except Exception as e:  # service vanished mid-call
                self.get_logger().error(f'{RESOLVE_SERVICE} failed: {e}')
                self._say(f"I couldn't look up {place}.")
                return
            not_found = response.outcome == ResolveDestination.Response.NOT_FOUND
            if not_found or not response.candidates:
                self._say(f"I couldn't find {place} on the map. Say it another way.")
                return
            found = response.candidates[0]   # the best match, also when there are several
            self._assumed_pub.publish(String(data=json.dumps(
                {'name': found.name, 'lat': found.latitude, 'lon': found.longitude})))
            self._say(f'Okay, starting from {found.name}.')

        self._resolve_client.call_async(request).add_done_callback(done)

    def _say(self, text):
        self.get_logger().info(f'Say: {text}')
        self._response_pub.publish(String(data=text))

    # ---- The route as waypoints ------------------------------------------------------

    def _on_route(self, msg):
        """Keep the route follower's route (new at every start and reroute; empty = none)."""
        self._route = msg
        self._publish_waypoints()

    def _publish_waypoints(self):
        """Publish the route as points every waypoint_spacing metres, or an empty path.

        Only while a GPS trip is running: the moment it ends (stop, arrived,
        failed) an empty path goes out, so nobody keeps driving an old route.
        The points are in the route's own frame (header.frame_id), with the
        direction of travel as the orientation; the last point is the destination.
        """
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        if self._bridge.active and self._route is not None and self._route.poses:
            path.header.frame_id = self._route.header.frame_id
            line = [(p.pose.position.x, p.pose.position.y) for p in self._route.poses]
            for x, y, heading in resample_polyline(line, self._waypoint_spacing):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(heading / 2.0)
                pose.pose.orientation.w = math.cos(heading / 2.0)
                path.poses.append(pose)
        if not path.poses and not self._waypoints_sent:
            return  # already empty
        self._waypoints_sent = bool(path.poses)
        self.get_logger().info(
            f'Waypoints: {len(path.poses)} points' if path.poses else 'Waypoints: cleared')
        self._waypoints_pub.publish(path)

    # ---- Health ---------------------------------------------------------------------

    def _now(self):
        # A monotonic clock for "how long ago did X arrive": it never jumps. The
        # ROS clock follows the system clock, which can jump (e.g. WSL2 re-syncing
        # with Windows) and would make a fresh message look seconds old.
        return time.monotonic()

    def _on_gps_fix(self, msg):
        """Note when a GPS message, and a GPS fix, last arrived."""
        now = self._now()
        self._gps_message_time = now
        if msg.status.status >= NavSatStatus.STATUS_FIX:   # STATUS_NO_FIX is -1
            self._gps_fix_time = now

    def _on_status(self, msg):
        """Keep the localizer's latest status."""
        self._status = msg
        self._status_time = self._now()
        if msg.initialised and msg.quality != LocalizationStatus.LOST:
            self._ever_localised = True

    def _check_health(self):
        """Work out the health, publish it when it changes, and tell the bridge."""
        status = self._status
        inputs = HealthInputs(
            now=self._now(), gps_topic=self._gps_topic,
            gps_message_time=self._gps_message_time, gps_fix_time=self._gps_fix_time,
            status_time=self._status_time,
            quality=status.quality if status else LocalizationStatus.LOST,
            initialised=status.initialised if status else False,
            ever_localised=self._ever_localised,
            mission_ready=self._action_client.server_is_ready(),
            planner_ready=self._plan_client.service_is_ready(),
            nav2_ready=self._nav2_client.server_is_ready(),
            nav2_required=self._require_nav2,
            geocoder_ready=self._geocode_client.service_is_ready())
        health = evaluate(inputs)
        self._bridge.set_blocker('' if health.ready else health.summary)
        text = health.to_json()
        if text == self._health_text:
            return  # latched topic: publish only what is new
        self._health_text = text
        self._health_pub.publish(String(data=text))
        not_ok = ', '.join(f'{p.name}: {p.detail}' for p in health.parts if not p.ok)
        self.get_logger().info(f"Health: {health.summary} ({not_ok or 'all parts ok'})")

    # ---- Carrying out the bridge's commands -----------------------------------------

    def _run(self, commands):
        """Carry out each command; answers come back later as new events."""
        for command in commands:
            if isinstance(command, StartMission):
                self._route = None          # an older trip's route is not this trip's
                self._publish_waypoints()
                self._start(command)
            elif isinstance(command, CancelMission):
                self._cancel(command)
            elif isinstance(command, SendGo):
                self._send_go(command)
            elif isinstance(command, Say):
                self.get_logger().info(f'Say: {command.text}')
                self._response_pub.publish(String(data=command.text))
            elif isinstance(command, Status):
                self.get_logger().info(f'Status: {command.text}')
                self._status_pub.publish(String(data=command.text))
            elif isinstance(command, Done):
                self.get_logger().info(f'Done: {command.target}')
                self._done_pub.publish(String(data=command.target))
            elif isinstance(command, Failed):
                self.get_logger().warning(f'Failed: {command.text}')
                self._failed_pub.publish(String(data=command.text))
            elif isinstance(command, CrossingWait):
                self._crossing_pub.publish(Bool(data=command.waiting))
            else:
                self.get_logger().error(f'Unknown command {command!r}')
        if self._waypoints_sent and not self._bridge.active:
            self._route = None
            self._publish_waypoints()  # the trip has just ended: clear them

    def _start(self, command):
        """Send the goal now, or once the previous goal has ended."""
        if self._current is not None:
            self._queued = command  # an older queued start is dropped: the bridge moved on
            return
        self._send_goal(command)

    def _send_goal(self, command):
        """Send a NavigateTo goal by name for a StartMission command."""
        if not self._action_client.server_is_ready():
            self._run(self._bridge.on_server_unavailable(command.goal_id))
            return
        goal = NavigateTo.Goal()
        goal.target_type = NavigateTo.Goal.TARGET_NAME
        goal.destination = command.target
        # False: the user hears the route and says go. True (auto_go): it sets off at once.
        goal.skip_confirmation = command.skip_confirmation
        slot = GoalSlot(command.goal_id)
        self._current = slot
        self.get_logger().info(f"Goal {slot.goal_id}: '{command.target}'")
        # feedback_callback runs for every feedback message of this goal;
        # the goal_id goes with it so the bridge can ignore older goals.
        future = self._action_client.send_goal_async(
            goal, feedback_callback=lambda fb, s=slot: self._on_feedback(s, fb))
        future.add_done_callback(lambda f, s=slot: self._on_goal_response(s, f))

    def _on_goal_response(self, slot, future):
        """Handle the action server accepting or refusing the goal."""
        try:
            handle = future.result()
        except Exception as e:  # server vanished mid-call
            self.get_logger().error(f'Goal {slot.goal_id} not sent: {e}')
            self._run(self._bridge.on_server_unavailable(slot.goal_id))
            self._slot_ended(slot)
            return
        if not handle.accepted:
            self._run(self._bridge.on_goal_rejected(slot.goal_id))
            self._slot_ended(slot)
            return
        slot.handle = handle
        if slot.cancel_wanted:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f, s=slot: self._on_result(s, f))

    def _on_feedback(self, slot, feedback_msg):
        """Pass the mission's state and message from the action feedback to the bridge."""
        mission = feedback_msg.feedback.mission
        self._run(self._bridge.on_state(slot.goal_id, mission.state, mission.message))

    def _on_result(self, slot, future):
        """Pass the goal's result to the bridge, then send any queued goal."""
        try:
            result = future.result().result
        except Exception as e:  # server vanished mid-call
            self.get_logger().error(f'Goal {slot.goal_id}: no result: {e}')
            self._run(self._bridge.on_server_unavailable(slot.goal_id))
        else:
            self.get_logger().info(
                f'Goal {slot.goal_id} ended: code {result.error_code}: {result.message}')
            self._run(self._bridge.on_result(slot.goal_id, result.error_code,
                                             result.message))
        self._slot_ended(slot)

    def _slot_ended(self, slot):
        """Forget a finished goal and send the queued one, if the bridge still wants it."""
        if self._current is not slot:
            return
        self._current = None
        queued, self._queued = self._queued, None
        if (queued is not None and self._bridge.active
                and queued.goal_id == self._bridge.goal_id):
            self._send_goal(queued)

    def _cancel(self, command):
        """Cancel a goal: drop it if not sent yet, else ask the action server."""
        if self._queued is not None and self._queued.goal_id == command.goal_id:
            self._queued = None
            return
        slot = self._current
        if slot is None or slot.goal_id != command.goal_id:
            return
        self.get_logger().info(f'Cancelling goal {slot.goal_id}')
        if slot.handle is None:
            slot.cancel_wanted = True  # cancelled as soon as the server accepts it
        else:
            slot.handle.cancel_goal_async()

    def _send_go(self, command):
        """Call guide/user_go; a refused go is spoken by the bridge."""
        if not self._go_client.service_is_ready():
            self._run(self._bridge.on_go_reply(command.goal_id, False, GO_UNAVAILABLE_TEXT))
            return

        def done(future):
            try:
                response = future.result()
            except Exception as e:  # service vanished mid-call
                self.get_logger().error(f'{USER_GO_SERVICE} failed: {e}')
                self._run(self._bridge.on_go_reply(command.goal_id, False,
                                                   GO_UNAVAILABLE_TEXT))
                return
            self.get_logger().info(
                f"Go: {'accepted' if response.success else 'refused'}: {response.message}")
            self._run(self._bridge.on_go_reply(command.goal_id, response.success,
                                               response.message))

        self._go_client.call_async(Trigger.Request()).add_done_callback(done)


def main(args=None):
    """Start the node."""
    rclpy.init(args=args)
    node = GpsBridgeNode()
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
