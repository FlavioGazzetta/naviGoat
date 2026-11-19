"""Goal sender node: forwards route goals to Nav2's NavigateToPose action.

Subscribes to guide/next_goal (from the route follower), sends each goal to the
navigate_to_pose action server (Nav2 on the robot, the fake server in guide_sim),
and publishes what happened to it on guide/goal_status.

The user can pause anywhere (guide/pause): the active Nav2 goal is cancelled
so the robot stops, and new goals are held. guide/resume re-sends the latest
goal. The route follower keeps running throughout, so no route progress or
guidance is lost. guide/paused reports the current state.

guide/cancel ends everything (used by the mission manager when a mission
stops early): the active goal is cancelled, the latest goal is forgotten (so
nothing can be resumed) and pause is cleared.

Frames (Step 8): the route follower's goals are in the enu frame (metres east /
north of the configured origin), but Nav2 needs goals in its map frame
(RTAB-Map's). This node is the boundary: an enu goal is converted into the map
frame with the localizer's latest guide/map_in_enu at the moment it is sent
(also when re-sent on resume). Until the first guide/map_in_enu arrives, enu
goals wait. Goals already in the map frame are sent unchanged.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from guide_core.angles import quaternion_to_yaw, yaw_to_quaternion
from guide_core.se2 import Pose2D, relative
from guide_interfaces.msg import NavGoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


# Latched QoS for rarely-changing state topics (same profile as the route follower uses).
# RELIABLE: every message is delivered (resent if lost).
# TRANSIENT_LOCAL: the publisher keeps its last message (depth 1) and hands it to any
# subscriber that connects later, so late-starting nodes still get the current state.
# Both sides must use TRANSIENT_LOCAL for the late-joiner delivery to happen.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# When an action goal ends, the server reports an action_msgs/GoalStatus code
# (STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6). This maps them to our own
# NavGoalStatus codes. Anything unexpected is treated as ABORTED.
RESULT_TO_STATUS = {
    GoalStatus.STATUS_SUCCEEDED: NavGoalStatus.SUCCEEDED,
    GoalStatus.STATUS_ABORTED: NavGoalStatus.ABORTED,
    GoalStatus.STATUS_CANCELED: NavGoalStatus.CANCELED,
}

# Readable names for log lines.
ACTION_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


class GoalSenderNode(Node):
    """Forwards every goal from guide/next_goal to NavigateToPose and reports the outcome."""

    def __init__(self):
        super().__init__('goal_sender')

        # Parameters (all optional). action_name is the action server to talk to;
        # retry_period is how often we check whether a waiting goal can be sent yet;
        # feedback_log_period limits how often feedback is logged; cancel_timeout is
        # how long shutdown waits for the server to answer our cancel request;
        # max_goal_age: goals stamped longer ago than this (s) are ignored as stale
        # (e.g. an old latched goal re-delivered after this node restarts).
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('retry_period', 1.0)
        self.declare_parameter('feedback_log_period', 5.0)
        self.declare_parameter('cancel_timeout', 1.0)
        self.declare_parameter('max_goal_age', 5.0)
        # Frames (Step 8): goals in enu_frame are converted into nav_frame (the frame
        # Nav2 plans in) before sending; goals already in nav_frame go unchanged.
        self.declare_parameter('enu_frame', 'enu')
        self.declare_parameter('nav_frame', 'map')
        self._action_name = self.get_parameter('action_name').value
        retry_period = float(self.get_parameter('retry_period').value)
        self._feedback_log_period = float(self.get_parameter('feedback_log_period').value)
        self._cancel_timeout = float(self.get_parameter('cancel_timeout').value)
        self._max_goal_age = float(self.get_parameter('max_goal_age').value)
        self._enu_frame = self.get_parameter('enu_frame').value
        self._nav_frame = self.get_parameter('nav_frame').value
        # Latest "map frame expressed in enu" from the localizer (a Pose2D), or None.
        self._map_in_enu = None

        # Goal bookkeeping.
        # _seq: number of the newest goal received (1, 2, 3...). Every callback remembers
        #   the number of the goal it belongs to; if that is not _seq, the callback is
        #   about an older goal and is "stale" (decision 3).
        # _goal: newest goal exactly as received (its original stamp is kept for matching);
        #   None before the first goal and after guide/cancel.
        # _unsent: True while the newest goal has not been sent yet (e.g. server not up).
        # _active_handle / _active_seq: the accepted goal Nav2 is currently working on,
        #   so we can cancel it on pause, guide/cancel and shutdown.
        self._seq = 0
        self._goal = None
        self._unsent = False
        self._active_handle = None
        self._active_seq = 0
        # _paused: True between guide/pause and guide/resume; no goal is sent then.
        # _succeeded_seq: number of the last goal Nav2 reported SUCCEEDED, so resume
        #   does not re-send a goal that was already reached (e.g. a crossing stop).
        # _cancelled_upto: goals numbered up to this were ended by guide/cancel; if
        #   the server accepts one of them late, it is cancelled straight away.
        self._paused = False
        self._succeeded_seq = 0
        self._cancelled_upto = 0

        # Action client for nav2_msgs/action/NavigateToPose. An action is a long-running
        # request: the client sends a goal, the server accepts or rejects it, streams
        # feedback while working, and finally returns a result with a status
        # (SUCCEEDED / ABORTED / CANCELED). Nav2 serves it under the name navigate_to_pose.
        self._client = ActionClient(self, NavigateToPose, self._action_name)

        # Status publisher: latched, published only when the status changes.
        self._status_pub = self.create_publisher(NavGoalStatus, 'guide/goal_status', LATCHED_QOS)

        # Pause state publisher: latched like the status, so a node that starts later
        # (mission manager, RViz) still learns whether we are paused. std_msgs/Bool
        # has a single field, 'data'.
        self._paused_pub = self.create_publisher(Bool, 'guide/paused', LATCHED_QOS)
        self._paused_pub.publish(Bool(data=False))

        # Pause / resume / cancel services. std_srvs/Trigger has an empty request and a
        # response with 'bool success' and 'string message'. Called from the CLI in
        # sim (ros2 service call /guide/pause std_srvs/srv/Trigger) and by the mission
        # manager. The single-threaded executor runs them between the other callbacks,
        # never at the same time, so no lock.
        self.create_service(Trigger, 'guide/pause', self._on_pause)
        self.create_service(Trigger, 'guide/resume', self._on_resume)
        self.create_service(Trigger, 'guide/cancel', self._on_cancel)

        # Goal subscriber: guide/next_goal is latched by the route follower, so we must
        # subscribe with TRANSIENT_LOCAL too, to receive the current goal even if we
        # start after the follower published it.
        self._goal_sub = self.create_subscription(
            PoseStamped, 'guide/next_goal', self._on_next_goal, LATCHED_QOS)

        # The localizer's estimate of where the map frame sits in enu (10 Hz, default
        # reliable QoS). Only the latest value is kept; it is used at send time.
        self.create_subscription(
            TransformStamped, 'guide/map_in_enu', self._on_map_in_enu, 10)

        # Timer: retries sending a goal that is waiting for the action server. It also
        # wakes the executor at least once per retry_period, which lets Ctrl+C get
        # through quickly (see main()).
        self._timer = self.create_timer(retry_period, self._try_send)

        self.get_logger().info(
            f"Goal sender started: guide/next_goal -> action '{self._action_name}'")

    # ---- Incoming goals ------------------------------------------------------------

    # Called for every message on guide/next_goal. A new goal always replaces the old
    # one: we do not cancel the old goal, we just send the new one and the action
    # server preempts the old one (decision 3: no stop between goals).
    def _on_next_goal(self, msg):
        p = msg.pose.position
        if not msg.header.frame_id or not (math.isfinite(p.x) and math.isfinite(p.y)):
            self.get_logger().warn(
                'Ignoring invalid goal (empty frame_id or non-finite position)',
                throttle_duration_sec=5.0)
            return

        # Stale-goal guard. guide/next_goal is latched and cannot be withdrawn: if
        # this node (re)starts after the follower's route was cleared, it still
        # receives the follower's last goal. The follower stamps each goal with the
        # pose it came from, so a fresh goal is milliseconds old; an old stamp means
        # a re-delivered leftover, which must not make the robot drive. A zero
        # stamp (hand-made test goal) is accepted.
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds > 0:
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self._max_goal_age:
                self.get_logger().warn(
                    f'Ignoring stale goal x={p.x:.2f} y={p.y:.2f}: stamped {age:.1f} s ago '
                    f'(max_goal_age {self._max_goal_age:.1f} s)')
                return

        self._seq += 1
        self._goal = msg
        self._unsent = True

        q = msg.pose.orientation
        yaw_deg = math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w))
        held = ', held while paused' if self._paused else ''
        self.get_logger().info(
            f'Goal #{self._seq} received: x={p.x:.2f} y={p.y:.2f} yaw={yaw_deg:.1f} deg '
            f'(frame {msg.header.frame_id}){held}')

        self._publish_status(NavGoalStatus.PENDING)
        self._try_send()

    # Remember the newest map-frame position from the localizer.
    def _on_map_in_enu(self, msg):
        t = msg.transform
        q = t.rotation
        self._map_in_enu = Pose2D(t.translation.x, t.translation.y,
                                  quaternion_to_yaw(q.x, q.y, q.z, q.w))

    # ---- Sending -------------------------------------------------------------------

    # Returns the newest goal as a Pose in nav_frame, or None if it cannot be
    # converted yet (an enu goal before the localizer's first guide/map_in_enu).
    # relative(T, goal) expresses the enu goal in the map frame: the inverse of
    # "map frame in enu" applied to the goal's position and heading.
    def _goal_in_nav_frame(self):
        frame = self._goal.header.frame_id
        if frame != self._enu_frame:
            if frame != self._nav_frame:
                self.get_logger().warn(
                    f"Goal #{self._seq} is in frame '{frame}', sending it unchanged",
                    throttle_duration_sec=5.0)
            return self._goal.pose
        if self._map_in_enu is None:
            self.get_logger().warn(
                f'Goal #{self._seq} waiting: no guide/map_in_enu from the localizer yet',
                throttle_duration_sec=5.0)
            return None
        p = self._goal.pose.position
        q = self._goal.pose.orientation
        converted = relative(self._map_in_enu,
                             Pose2D(p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)))
        pose = PoseStamped().pose
        pose.position.x = converted.x
        pose.position.y = converted.y
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
            yaw_to_quaternion(converted.yaw)
        return pose

    # Sends the newest goal if it is unsent and the action server is available.
    # Called right after a goal arrives and again by the timer, so a goal received
    # before Nav2 is up gets sent as soon as it is up. Never blocks (decision 8).
    def _try_send(self):
        if not self._unsent or self._paused:
            return  # nothing to send, or held until guide/resume
        # server_is_ready() only checks the current state; it does not wait.
        if not self._client.server_is_ready():
            self.get_logger().warn(
                f"Goal #{self._seq} waiting: action server '{self._action_name}' "
                'not available', throttle_duration_sec=5.0)
            return

        pose = self._goal_in_nav_frame()
        if pose is None:
            return  # retried by the timer once the alignment is known

        self._unsent = False
        seq = self._seq

        # Build the NavigateToPose goal, in Nav2's frame. The stamp is left at zero,
        # meaning "use the latest available transform" (decision 6).
        # behavior_tree = '' tells Nav2 to use its default behaviour tree.
        goal = NavigateToPose.Goal()
        converted = self._goal.header.frame_id == self._enu_frame
        goal.pose.header.frame_id = self._nav_frame if converted else self._goal.header.frame_id
        goal.pose.pose = pose
        goal.behavior_tree = ''

        # send_goal_async() returns immediately with a future (a placeholder for a reply
        # that arrives later). The executor calls the done-callback when the server
        # answers accept/reject. The lambdas capture seq via "seq=seq" so each callback
        # remembers which goal it belongs to (a plain closure would see later values).
        send_future = self._client.send_goal_async(
            goal, feedback_callback=lambda fb, seq=seq: self._on_feedback(seq, fb))
        send_future.add_done_callback(
            lambda future, seq=seq: self._on_goal_response(seq, future))
        where = (f' as x={pose.position.x:.2f} y={pose.position.y:.2f} ({self._nav_frame})'
                 if converted else '')
        self.get_logger().info(f"Goal #{seq} sent to '{self._action_name}'{where}")

    # ---- Action callbacks ----------------------------------------------------------

    # Stage 2: the server accepted or rejected the goal. future.result() is a goal
    # handle; .accepted tells which. If accepted, we ask for the result (stage 3),
    # which arrives later through another future.
    def _on_goal_response(self, seq, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            if seq == self._seq:
                self.get_logger().error(f"Goal #{seq} REJECTED by '{self._action_name}'")
                self._publish_status(NavGoalStatus.REJECTED,
                                     message='goal rejected by action server')
            return

        # Still ask for the result of an accepted stale goal, only so its end is logged.
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, seq=seq: self._on_result(seq, f))

        # Sent before guide/cancel but accepted after it: nothing newer will
        # replace it, so cancel it now or the robot would drive to it.
        if seq <= self._cancelled_upto:
            self.get_logger().info(f'Goal #{seq} accepted after guide/cancel: cancelling it')
            goal_handle.cancel_goal_async()
            return

        if seq != self._seq:
            self.get_logger().info(
                f'Goal #{seq} accepted, but goal #{self._seq} is newer and will replace it')
            return

        self._active_handle = goal_handle
        self._active_seq = seq
        self.get_logger().info(f'Goal #{seq} accepted')
        self._publish_status(NavGoalStatus.ACTIVE)

        # Paused while this goal was on its way to the server: cancel it now.
        if self._paused:
            self._cancel_active('accepted after pause')

    # Called repeatedly while a goal runs (about 10 Hz). feedback_msg.feedback holds
    # the NavigateToPose feedback fields. Logged at most once per feedback_log_period.
    def _on_feedback(self, seq, feedback_msg):
        if seq != self._seq:
            return
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Goal #{seq}: distance_remaining={fb.distance_remaining:.1f} m',
            throttle_duration_sec=self._feedback_log_period)

    # Stage 3: the goal has ended. future.result() has .status (an action_msgs/GoalStatus
    # code) and .result (NavigateToPose result: error_code, error_msg).
    def _on_result(self, seq, future):
        response = future.result()
        action_status = response.status
        result = response.result
        name = ACTION_STATUS_NAMES.get(action_status, f'status {action_status}')

        if seq == self._active_seq:
            self._active_handle = None

        # Stale: a newer goal was sent after this one (its ABORTED is just the
        # preemption by our own newer goal, not a navigation failure, decision 3),
        # or guide/cancel ended it (nothing to report: the mission is over).
        if seq != self._seq or seq <= self._cancelled_upto:
            reason = ('after guide/cancel' if seq <= self._cancelled_upto
                      else f'after being replaced by goal #{self._seq}')
            self.get_logger().info(f'Goal #{seq} ended ({name}) {reason}; ignored')
            return

        status = RESULT_TO_STATUS.get(action_status, NavGoalStatus.ABORTED)
        message = result.error_msg or ('' if status == NavGoalStatus.SUCCEEDED else name)
        if status == NavGoalStatus.SUCCEEDED:
            self._succeeded_seq = seq
            self.get_logger().info(f'Goal #{seq} SUCCEEDED')
        elif status == NavGoalStatus.CANCELED:
            self.get_logger().warn(f'Goal #{seq} CANCELED')
        else:
            self.get_logger().error(
                f'Goal #{seq} {name}: error_code={result.error_code} {result.error_msg}')
        self._publish_status(status, error_code=result.error_code, message=message)

    # ---- Status --------------------------------------------------------------------

    # Publishes NavGoalStatus for the newest goal. goal is copied exactly as received
    # (original stamp included) so the route follower can match it to its own goal.
    def _publish_status(self, status, error_code=0, message=''):
        if self._goal is None:
            return  # no current goal (after guide/cancel): nothing to report
        msg = NavGoalStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._goal.header.frame_id
        msg.goal = self._goal
        msg.status = status
        msg.error_code = error_code
        msg.message = message
        self._status_pub.publish(msg)

    # ---- Pause / resume / cancel ---------------------------------------------------

    # guide/pause: stop the robot where it is. The active goal is cancelled (the
    # server then reports CANCELED, which we publish as usual; the follower ignores
    # it). Goals arriving while paused are kept as the newest goal but not sent.
    def _on_pause(self, request, response):
        if self._paused:
            response.success, response.message = False, 'Already paused'
            return response
        self._paused = True
        self._paused_pub.publish(Bool(data=True))
        if self._active_handle is not None:
            self._cancel_active('paused by request')
            response.message = f'Paused: cancelling goal #{self._active_seq}'
        else:
            response.message = 'Paused (no active Nav2 goal)'
        response.success = True
        self.get_logger().info(response.message)
        return response

    # guide/resume: send the newest goal again, unless Nav2 already reached it.
    # It gets a new sequence number, so any late result of the cancelled goal is
    # stale and ignored. The goal message itself is unchanged (original stamp), so
    # the route follower still recognises its status.
    def _on_resume(self, request, response):
        if not self._paused:
            response.success, response.message = False, 'Not paused'
            return response
        self._paused = False
        self._paused_pub.publish(Bool(data=False))
        response.success = True
        if self._goal is None:
            response.message = 'Resumed (no goal to send)'
        elif self._succeeded_seq == self._seq:
            response.message = 'Resumed (current goal already reached, nothing to send)'
        else:
            self._seq += 1
            self._unsent = True
            p = self._goal.pose.position
            response.message = f'Resumed: re-sending goal x={p.x:.2f} y={p.y:.2f} as #{self._seq}'
            self._publish_status(NavGoalStatus.PENDING)
            self._try_send()
        self.get_logger().info(response.message)
        return response

    # guide/cancel: end everything, back to the state at startup. Unlike pause,
    # nothing is kept to resume: the active goal is cancelled, the newest goal is
    # forgotten, and pause is cleared. Every goal numbered up to now counts as
    # cancelled (see _on_goal_response / _on_result). Always succeeds, so the
    # mission manager can call it whenever a mission ends early.
    def _on_cancel(self, request, response):
        parts = []
        if self._active_handle is not None:
            self._cancel_active('guide/cancel')
            parts.append(f'cancelling goal #{self._active_seq}')
        elif self._goal is not None and self._unsent:
            parts.append(f'dropped unsent goal #{self._seq}')
        self._cancelled_upto = self._seq
        self._goal = None
        self._unsent = False
        if self._paused:
            self._paused = False
            self._paused_pub.publish(Bool(data=False))
            parts.append('pause cleared')
        response.success = True
        response.message = 'Stopped: ' + ', '.join(parts) if parts else 'Stopped (nothing active)'
        self.get_logger().info(response.message)
        return response

    # Asks the server to cancel the goal it is working on, without waiting: we are
    # inside a callback, and waiting here (spin_until_future_complete) would
    # deadlock the executor. The answer is only logged.
    def _cancel_active(self, reason):
        seq = self._active_seq
        self.get_logger().info(f'Cancelling goal #{seq} ({reason})')
        future = self._active_handle.cancel_goal_async()
        future.add_done_callback(
            lambda f, seq=seq: self.get_logger().info(
                f'Cancel of goal #{seq} answered (return_code={f.result().return_code})'))

    # ---- Shutdown ------------------------------------------------------------------

    # Best-effort cancel of the goal Nav2 is working on, so the robot does not keep
    # driving towards a point up to 25 m away with no guidance layer running.
    # Called from main() after spinning has stopped, so waiting here with
    # spin_until_future_complete is safe (inside a callback it would deadlock).
    def cancel_active_goal(self):
        if self._active_handle is None:
            return
        self.get_logger().info(f'Shutting down: cancelling goal #{self._active_seq}')
        future = self._active_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._cancel_timeout)
        if future.done():
            self.get_logger().info(
                f'Cancel request answered (return_code={future.result().return_code})')
        else:
            self.get_logger().warn(
                f'No answer to cancel request within {self._cancel_timeout:.1f} s')


def main(args=None):
    # SignalHandlerOptions.NO: do not install rclpy's own Ctrl+C handler. That handler
    # shuts the ROS context down immediately, which would make it impossible to send
    # the cancel request afterwards. Without it, Ctrl+C raises a normal Python
    # KeyboardInterrupt (within about a second, because our timer wakes the executor),
    # and the ROS context is still alive in the finally block.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GoalSenderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cancel_active_goal()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
