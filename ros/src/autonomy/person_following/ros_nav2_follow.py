"""Follow using fixed-frame goal previews, optionally NavigateToPose actions.

Default: publish PoseStamped previews only. --execute-nav2 sends navigation goals
and can move a robot. This node never publishes cmd_vel. Run --help without ROS.
"""

from dataclasses import replace
import argparse
import json
import math
import signal
import time
import uuid

from goal_generator import GoalConfig, GoalGenerator, camera_point_on_map
from mode_bridge import FollowingModeBridge, ModeCommandError
from nav2_goal_client import Nav2GoalClient
from perception_adapter import PerceptionAdapter, PerceptionError
from state_machine import FollowingConfig, FollowingState, FollowingStateMachine


def positive_number(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("Value must be finite and positive")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-nav2", action="store_true")
    parser.add_argument("--topic", default="/social_perception/tracks")
    parser.add_argument("--mode-topic", default="/middleman/mode")
    parser.add_argument("--status-topic", default="/person_following/status")
    parser.add_argument("--preview-topic", default="/person_following/goal_preview")
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--camera-frame", default="camera_link")
    parser.add_argument("--action-name", default="navigate_to_pose")
    parser.add_argument("--following-distance", type=positive_number, default=1.0)
    parser.add_argument("--goal-hz", type=positive_number, default=2.0)
    parser.add_argument("--control-hz", type=positive_number, default=20.0)
    parser.add_argument("--input-timeout", type=positive_number, default=0.5)
    parser.add_argument("--loss-timeout", type=positive_number, default=2.0)
    parser.add_argument("--action-timeout", type=positive_number, default=2.0)
    parser.add_argument("--enable-on-start", action="store_true")
    args, ros_args = parser.parse_known_args()
    try:
        goal_config = GoalConfig(distance_m=args.following_distance, update_hz=args.goal_hz)
        tracking_config = FollowingConfig(input_timeout_s=args.input_timeout,
                                          loss_timeout_s=args.loss_timeout)
    except ValueError as exc:
        parser.error(str(exc))
    if args.execute_nav2 and args.global_frame in (args.base_frame, args.camera_frame):
        parser.error("Nav2 goals require a fixed navigation frame, such as map or odom")
    try:
        import rclpy
        from rclpy.clock import Clock, ClockType
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.time import Time
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
        from tf2_ros import Buffer, TransformException, TransformListener
        if args.execute_nav2:
            from rclpy.action import ActionClient
            from nav2_msgs.action import NavigateToPose
    except ImportError as exc:
        parser.exit(1, f"ROS 2 Jazzy/TF environment required: {exc}\n")

    class Nav2Following(Node):
        def __init__(self):
            super().__init__("person_following_nav2")
            self.machine = FollowingStateMachine(config=tracking_config)
            self.bridge = FollowingModeBridge(self.machine)
            self.adapter = PerceptionAdapter(args.input_timeout)
            self.generator = GoalGenerator(goal_config)
            self.pending = None
            self.latest = None
            self.manual_session = uuid.uuid4().hex
            self.last_report = float("-inf")
            self.stopping = False
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.preview_pub = self.create_publisher(PoseStamped, args.preview_topic, 10)
            self.status_pub = self.create_publisher(String, args.status_topic, 10)
            self.navigation = None
            if args.execute_nav2:
                self.action_client = ActionClient(self, NavigateToPose, args.action_name)
                self.navigation = Nav2GoalClient(self.action_client, self.action_message,
                                                 args.action_timeout, 1 / args.goal_hz)
            self.create_subscription(String, args.topic, self.on_tracks,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(String, args.mode_topic, self.on_mode, 10)
            self.create_service(SetBool, "/person_following/set_enabled", self.on_enable)
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1 / args.control_hz, self.tick, clock=self.steady)
            mode = "NAV2 ACTIONS ENABLED" if args.execute_nav2 else "PREVIEW ONLY"
            self.get_logger().info(f"{mode}; goals in {args.global_frame}; no velocity publisher. "
                                   "Run only one following runner. Tracker still requires a level camera.")
            if args.enable_on_start:
                self.report(self.bridge.set_enabled(True), time.monotonic())

        @property
        def session(self):
            return self.bridge.request_id or self.manual_session

        def cancel_navigation(self, now):
            if self.navigation is not None:
                self.navigation.cancel(now)

        def on_mode(self, msg):
            if self.stopping:
                return
            now = time.monotonic()
            old_session = self.session
            self.pending = None
            try:
                output = self.bridge.handle_command(msg.data)
            except ModeCommandError as exc:
                output = self.bridge.reject_command()
                self.get_logger().error(str(exc))
            if output.state != FollowingState.FOLLOWING or self.session != old_session:
                self.generator.reset()
                self.cancel_navigation(now)
            self.report(output, now)

        def on_enable(self, request, response):
            if self.stopping:
                response.success, response.message = False, "Shutting down"
                return response
            now = time.monotonic()
            self.pending = None
            if request.data and self.machine.state == FollowingState.IDLE:
                self.manual_session = uuid.uuid4().hex
                self.generator.reset()
            output = self.bridge.set_enabled(request.data)
            if output.state != FollowingState.FOLLOWING:
                self.cancel_navigation(now)
            self.report(output, now)
            response.success = True
            response.message = output.state.value
            return response

        def on_tracks(self, msg):
            if self.stopping:
                return
            now = time.monotonic()
            try:
                frame = self.adapter.parse(msg.data,
                    source_now_s=self.get_clock().now().nanoseconds * 1e-9,
                    monotonic_now_s=now)
            except PerceptionError as exc:
                self.pending = None
                self.cancel_navigation(now)
                self.report(self.machine.update(now, input_healthy=False), now)
                self.get_logger().error(str(exc), throttle_duration_sec=2.0)
                return
            if frame is not None:
                self.latest = self.pending = frame

        def positions(self, target_id):
            person = next(p for p in self.latest.people if p.track_id == target_id and p.observed)
            captured = Time(seconds=self.latest.source_timestamp_s)
            # No fallback to latest TF for a historical camera observation.
            camera_tf = self.buffer.lookup_transform(args.global_frame, args.camera_frame, captured)
            robot_tf = self.buffer.lookup_transform(args.global_frame, args.base_frame, Time())
            stamp = robot_tf.header.stamp.sec + robot_tf.header.stamp.nanosec * 1e-9
            if stamp and abs(self.get_clock().now().nanoseconds * 1e-9 - stamp) > args.input_timeout:
                raise ValueError("Robot transform is stale")
            t, q = camera_tf.transform.translation, camera_tf.transform.rotation
            target = camera_point_on_map(person.forward_m, person.lateral_m,
                                         (t.x, t.y, q.x, q.y, q.z, q.w))
            robot = robot_tf.transform.translation
            return (robot.x, robot.y), target

        def pose_message(self, goal):
            pose = PoseStamped()
            pose.header.frame_id = args.global_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = goal.x, goal.y
            pose.pose.orientation.z = math.sin(goal.yaw / 2)
            pose.pose.orientation.w = math.cos(goal.yaw / 2)
            return pose

        def action_message(self, goal):
            message = NavigateToPose.Goal()
            message.pose = self.pose_message(goal)
            return message

        def tick(self):
            now = time.monotonic()
            frame, self.pending = self.pending, None
            output = self.machine.update(now, frame)
            if output.state != FollowingState.FOLLOWING:
                self.cancel_navigation(now)
                self.generator.reset()
            else:
                try:
                    robot, target = self.positions(output.target_id)
                    action, goal = self.generator.update(robot, target, now)
                    if action == "hold":
                        self.cancel_navigation(now)
                    elif action == "send":
                        self.preview_pub.publish(self.pose_message(goal))
                        self.get_logger().info(f"Goal ({goal.x:.2f},{goal.y:.2f}), yaw={goal.yaw:.2f} "
                                               f"in {args.global_frame}; target={output.target_id}")
                        if self.navigation is not None:
                            self.navigation.request_goal(goal, self.session, now)
                except (TransformException, ValueError, StopIteration) as exc:
                    self.cancel_navigation(now)
                    self.generator.reset()
                    output = replace(self.machine.disable(), event="transform_unavailable")
                    self.get_logger().error(f"Cannot form a navigation goal: {exc}")
            if self.navigation is not None:
                # Evaluate fresh tracking/hold/stop conditions BEFORE allowing a
                # queued replacement goal to be dispatched.
                self.navigation.tick(now)
                events, self.navigation.events = self.navigation.events, []
                for session, event in events:
                    if event == "goal_reached":
                        continue  # reaching one waypoint does not end following
                    self.get_logger().error(f"Nav2: {event} (session {session})")
                    if session == self.session and output.state != FollowingState.IDLE:
                        output = replace(self.machine.disable(), event=event)
                        self.cancel_navigation(now)
                        self.generator.reset()
            self.report(output, now)

        def report(self, output, now):
            if output.event is None and now - self.last_report < 0.5:
                return
            self.last_report = now
            payload = self.bridge.status_payload(output)
            payload.update(navigation="execute" if args.execute_nav2 else "preview",
                           navigation_pending=self.navigation.busy if self.navigation else False)
            self.status_pub.publish(String(data=json.dumps(payload)))
            self.get_logger().info(f"state={output.state.value} target={output.target_id} "
                                   f"event={output.event or '-'} navigation={payload['navigation']}")

    # Keep ROS alive long enough to receive cancellation results on SIGINT/TERM.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Nav2Following()
    stop_requested = [False]

    def request_stop(_signal, _frame):
        stop_requested[0] = True

    handlers = {name: signal.signal(name, request_stop) for name in (signal.SIGINT, signal.SIGTERM)}
    try:
        while rclpy.ok() and not stop_requested[0]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stopping = True
        now = time.monotonic()
        node.cancel_navigation(now)
        if rclpy.ok():
            node.report(node.bridge.set_enabled(False), now)
            deadline = now + args.action_timeout + 0.1
            while node.navigation and node.navigation.busy and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.navigation and node.navigation.busy:
                node.get_logger().error("Shutdown: could not confirm all navigation goals stopped")
        for name, handler in handlers.items():
            signal.signal(name, handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
