"""Simulated robot: stands in for the Pioneer 3-AT, its sensors, RTAB-Map and Nav2.

In this file we describe our robot simulator.
In __init__ we set up all our parameters (the origin point, the hz rates, the robot's speed
limits, the sensor error settings), create the motion model, the error models, the lock, the
publishers, the timers and the action server.
The physics tick moves the robot one step using the model, works out what its wheel odometry,
IMU and SLAM would report, and publishes them (odometry, IMU, TF, ground truth).
The GPS tick looks up how good the GPS is where the robot is, adds the matching error and
publishes the fix as lat/lon. execute_goal takes each goal, converts it from the map frame
into the robot's true position frame, hands it to the model, and then supervises it:
it keeps checking whether the goal has been reached, cancelled or replaced by a newer one,
and sends progress feedback until it finishes.
The model is what actually drives the robot towards the goal.
main turns everything on: it starts ROS, creates our node,
and runs an executor that calls our timers and execute_goal whenever they're due,
until we hit Ctrl+C, then shuts everything down cleanly.

Frames (Step 8):
- enu: the true world frame, metres east (x) / north (y) of the configured origin. The motion
  model, the GPS and the ground truth live here. Not part of the TF tree.
- map: the fake RTAB-Map's frame. With slam_profile 'enu' (default) it is the enu frame, as in
  v0; with 'perfect' or 'lidar' it starts at the robot's start pose ('lidar' also drifts).
- odom: the drifting wheel-odometry frame; always starts at (0, 0) where the robot starts.
- base_link: the robot. TF: map -> odom (fake RTAB-Map) -> base_link (fake base driver).
- TF enu -> map (the TRUE alignment) is also broadcast, for RViz only, so routes and
  markers in the enu frame can be drawn. No guide node may use it: the localizer
  estimates the alignment itself (guide/map_in_enu). The real robot has no enu TF.

All sensor errors are off by default, so the sim behaves as before Step 8 unless the
parameters below switch them on.
"""

import math
import random
import threading
import time

# ROS message and action types. Each ROS package generates Python classes from its
# .msg/.action files; importing them here lets us create and fill those messages.
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
# Our own ROS-free code from guide_core (unit-tested separately).
from guide_core.angles import yaw_to_quaternion
from guide_core.geo import enu_to_latlon
from guide_core.gps_error import GpsErrorModel
from guide_core.gps_zones import zone_map_from_dict
from guide_core.odom_error import (
    body_motion, FakeSlam, map_to_odom, odom_config_from_dict, OdomErrorModel,
    slam_config_from_dict)
from guide_core.se2 import Pose2D, transform_point, wrap_angle
from guide_core.sim_model import RobotState, SimRobot
from nav2_msgs.action import NavigateToPose  # Nav2's "drive to this pose" action
from nav_msgs.msg import Odometry            # robot position + velocity from wheel motion
import networkx
# rclpy is the ROS Python library: everything needed to create nodes,
# publishers, timers and action servers, and to run them.
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus  # IMU reading, GPS fix, status codes
# tf2_ros sends coordinate-frame transforms (TF), which nodes like RViz and Nav2 use to
# convert positions between frames.
from tf2_ros import TransformBroadcaster
import yaml

MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
ENU_FRAME = 'enu'

# Launch files cannot pass empty strings, so 'none' also means "not set".
UNSET = ('', 'none')


def quaternion_msg(yaw):
    """Build a ROS Quaternion message from a heading angle.

    ROS messages store orientation as a quaternion (x, y, z, w) rather than an
    angle. guide_core gives us the four numbers; this wraps them in the ROS
    message type. It lives here, not in guide_core, because it uses a ROS type.
    """
    q = Quaternion()
    q.x, q.y, q.z, q.w = yaw_to_quaternion(yaw)
    return q


def transform_msg(parent, child, pose, stamp):
    """Build a TransformStamped saying where frame `child` is in frame `parent`."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = pose.x
    t.transform.translation.y = pose.y
    t.transform.rotation = quaternion_msg(pose.yaw)
    return t


def pose_msg(frame, pose, stamp):
    """Build a PoseStamped from a Pose2D."""
    p = PoseStamped()
    p.header.stamp = stamp
    p.header.frame_id = frame
    p.pose.position.x = pose.x
    p.pose.position.y = pose.y
    p.pose.orientation = quaternion_msg(pose.yaw)
    return p


class SimRobotNode(Node):
    # A ROS node in Python is a class that inherits from rclpy's Node. Inheriting
    # gives us all of Node's methods (create_publisher, create_timer,
    # declare_parameter, get_logger, get_clock...), which we call on `self`.

    def __init__(self):
        # Run Node's own setup first: registers this node with ROS under the
        # name 'sim_robot' (the name shown by `ros2 node list`).
        super().__init__('sim_robot')

        # --- Parameters ---
        # Settings that can be changed when the node is started, without editing
        # code (e.g. `ros2 run guide_sim sim_robot --ros-args -p max_speed:=1.5`).
        # declare_parameter(name, default) registers the setting; .value reads its
        # current value. ROS fixes each parameter's type from its default, so
        # defaults are written as floats (20.0, not 20).
        self.origin_lat = self.declare_parameter('origin_lat', 51.4988).value
        self.origin_lon = self.declare_parameter('origin_lon', -0.1749).value

        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        gps_rate_hz = self.declare_parameter('gps_rate_hz', 5.0).value

        # The robot's motion model (pure Python, from guide_core). Its limits come
        # from parameters, so the simulated robot can be tuned from the command line.
        self.robot = SimRobot(
            max_speed=self.declare_parameter('max_speed', 1.0).value,
            max_turn_rate=self.declare_parameter('max_turn_rate', 1.0).value,
            goal_tolerance=self.declare_parameter('goal_tolerance', 0.3).value,
        )

        # Where the robot starts, in the true enu frame: metres east (x) and north (y)
        # of the origin, heading in degrees (0 = east, 90 = north, REP 103).
        # plan_route prints values that put it on the first point of a planned route.
        # Replacing the model's state is safe here: no timer has run yet.
        start_x = self.declare_parameter('start_x', 0.0).value
        start_y = self.declare_parameter('start_y', 0.0).value
        start_yaw_deg = self.declare_parameter('start_yaw_deg', 0.0).value
        self.robot.state = RobotState(x=start_x, y=start_y, yaw=math.radians(start_yaw_deg))
        start_pose = Pose2D(start_x, start_y, math.radians(start_yaw_deg))

        # Seconds of simulated time per physics step (0.05 s at 20 Hz).
        self.dt = 1.0 / rate_hz

        # --- Sensor error parameters (Step 8; all off by default) ---
        # noise_seed: one number that makes a whole noisy run repeatable. Each error
        #   model gets its own seed derived from it.
        # gps_profile: GPS conditions everywhere ('perfect', 'open', 'urban_canyon').
        # gps_zones_file: optional YAML of zones (tunnels, canyons...); its 'default'
        #   entry, if present, replaces gps_profile. graph_file: the OSM graph, needed
        #   only if the zones file says tunnels_from_graph: true.
        # report_gps_covariance: put the receiver's reported sigma in NavSatFix
        #   (false = covariance 'unknown', as before Step 8).
        # gps_publish_no_fix: with no fix, publish STATUS_NO_FIX (true) or nothing (false).
        # odom_profile: 'perfect' or 'skid_steer'. slam_profile: 'enu' (map = enu, as in
        #   v0), 'perfect' (map frame at the start pose) or 'lidar' (and slowly drifting).
        # imu_yaw_bias_deg / imu_yaw_noise_deg: error of the IMU heading (e.g. magnetic
        #   disturbance near buildings).
        # The value of a string parameter must be a string, so its default is ''.
        noise_seed = int(self.declare_parameter('noise_seed', 0).value)
        gps_profile = self.declare_parameter('gps_profile', 'perfect').value
        gps_zones_file = self.declare_parameter('gps_zones_file', '').value
        graph_file = self.declare_parameter('graph_file', '').value
        self.report_gps_covariance = self.declare_parameter(
            'report_gps_covariance', False).value
        self.gps_publish_no_fix = self.declare_parameter('gps_publish_no_fix', True).value
        odom_profile = self.declare_parameter('odom_profile', 'perfect').value
        slam_profile = self.declare_parameter('slam_profile', 'enu').value
        self.imu_yaw_bias = math.radians(self.declare_parameter('imu_yaw_bias_deg', 0.0).value)
        self.imu_yaw_noise = math.radians(
            self.declare_parameter('imu_yaw_noise_deg', 0.0).value)
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.publish_truth = self.declare_parameter('publish_ground_truth', True).value

        # --- Error models (pure Python, from guide_core) ---
        # A wrong profile name or zones file raises ValueError here, so the node
        # stops at startup with a clear message instead of running with bad settings.
        self.zone_map = self.load_zone_map(gps_profile, gps_zones_file, graph_file)
        self.gps_model = GpsErrorModel(seed=noise_seed)
        self.odom_config = odom_config_from_dict({'profile': odom_profile})
        self.odom_model = OdomErrorModel(self.odom_config, seed=noise_seed + 1)
        self.slam = FakeSlam(start_pose, slam_config_from_dict({'profile': slam_profile}),
                             seed=noise_seed + 2)
        self.imu_rng = random.Random(noise_seed + 3)
        self.prev_true = start_pose
        self.map_pose = self.slam.step(start_pose)
        self.gps_zone = None   # name of the GPS zone the robot is in, for log messages

        # The timers and the action server run in different threads, so every
        # access to self.robot, the error models and self.active_goal goes through
        # this lock.
        self.lock = threading.Lock()

        # The goal currently being driven to. When a new goal arrives it replaces
        # this, and the older goal's loop notices and ends itself.
        self.active_goal = None

        # --- Publishers ---
        # Publishers send messages on a named topic. Any node subscribed to that
        # topic receives them. create_publisher(message_type, topic_name, qos):
        #   - topic names have no leading '/', so they can be namespaced later
        #   - qos sets delivery rules. qos_profile_sensor_data = "best effort, keep
        #     only the latest few": a lost sensor reading isn't resent because a
        #     newer one is coming. Subscribers to these topics must also use a
        #     best-effort profile, or they won't connect.
        #   - a plain number (10) means default reliable delivery, queue of 10.
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # Ground truth, for evaluation and RViz only (never used by the guide nodes):
        # the robot's true pose and where the map frame truly is, both in enu.
        self.truth_pose_pub = self.create_publisher(PoseStamped, 'sim/ground_truth/pose', 10)
        self.truth_map_pub = self.create_publisher(
            PoseStamped, 'sim/ground_truth/map_in_enu', 10)

        # TransformBroadcaster publishes on the shared /tf topic. Here the sim plays
        # the base driver (odom -> base_link) and RTAB-Map (map -> odom).
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timers ---
        # create_timer(period_seconds, function) calls the function repeatedly.
        # Physics + odometry + IMU + TF run fast (20 Hz); GPS runs slower (5 Hz), as a
        # real receiver does. The node keeps the timers alive internally, so we
        # don't need to store them. Note: no brackets after the function names,
        # we pass the function itself for the timer to call later.
        self.create_timer(self.dt, self.physics_tick)
        self.create_timer(1.0 / gps_rate_hz, self.publish_gps)

        # --- Fake Nav2 action server ---
        # An action is a long-running request: a client sends a goal, the server
        # works on it while sending feedback, and finishes with a result. We use
        # the same action type and name as real Nav2, so client code written
        # against this simulator works unchanged with the real robot.
        # ActionServer(node, action_type, action_name, ...):
        #   - execute_callback: function that carries out each accepted goal
        #   - goal_callback: decides whether to accept a new goal (we always accept)
        #   - cancel_callback: decides whether to allow cancelling (we always allow)
        #   - callback_group=ReentrantCallbackGroup(): lets several goals run at
        #     the same time, so a new goal can start while the old one's loop is
        #     still running (needed for one goal to replace another).
        # `lambda x: value` is a one-line function that ignores x and returns value.
        self.action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute_goal,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

        # get_logger() prints timestamped messages tagged with the node name.
        # Prefer it over print(): output can be filtered by severity and recorded.
        self.get_logger().info(
            f'Simulated robot ready, origin ({self.origin_lat}, {self.origin_lon}), '
            f'start ({start_x:.2f}, {start_y:.2f}) m, heading {start_yaw_deg:.1f} deg')
        self.get_logger().info(
            f'Sensor errors: seed {noise_seed}, GPS default {self.gps_default_name!r} '
            f'({len(self.zone_map.zones)} zones), odom {odom_profile!r}, '
            f'SLAM {slam_profile!r}, IMU yaw bias {math.degrees(self.imu_yaw_bias):.1f} deg '
            f'noise {math.degrees(self.imu_yaw_noise):.1f} deg')

    def load_zone_map(self, gps_profile, zones_file, graph_file):
        """Return the GPS zone map from the zones file (if any) and the default profile."""
        values = {}
        if zones_file not in UNSET:
            # yaml.safe_load turns the YAML text into Python dicts and lists.
            with open(zones_file) as f:
                values = yaml.safe_load(f) or {}
        values.setdefault('default', {'profile': gps_profile})
        self.gps_default_name = values['default'].get('profile', 'custom')
        graph = None
        if values.get('tunnels_from_graph', False):
            if graph_file in UNSET:
                raise ValueError('the GPS zones file needs graph_file (tunnels_from_graph)')
            # Plain networkx is enough here (no osmnx): gps_zones handles the text
            # attributes read_graphml produces.
            graph = networkx.read_graphml(graph_file)
        return zone_map_from_dict(values, self.origin_lat, self.origin_lon, graph)

    # ------------------------------------------------------------------
    # Simulation and sensors
    # ------------------------------------------------------------------

    def physics_tick(self):
        """Advance the robot one step, then publish odometry, IMU, TF and ground truth.

        Called by the physics timer 20 times a second.
        """
        # Move the robot one step, feed the true motion to the odometry and SLAM
        # models, and copy out the results. This happens inside the lock so the values
        # are consistent; building messages happens after releasing it, so other
        # threads aren't kept waiting.
        with self.lock:
            self.robot.step(self.dt)
            s = self.robot.state
            true_pose = Pose2D(s.x, s.y, s.yaw)
            v, omega = s.v, s.omega
            forward, dyaw = body_motion(self.prev_true, true_pose)
            self.prev_true = true_pose
            odom_pose = self.odom_model.step(forward, dyaw)
            self.map_pose = self.slam.step(true_pose, abs(forward))
            map_pose = self.map_pose
            map_in_enu = self.slam.map_in_enu

        # Current ROS time, converted to the format message headers use. All
        # messages describe the same instant, so they share one timestamp.
        stamp = self.get_clock().now().to_msg()

        # Odometry: where the robot is and how fast it's moving, as wheel encoders
        # would report it. Message structure:
        #   header.frame_id = 'odom'      -> position is measured in the odom frame
        #                                    (from where the robot was switched on:
        #                                    always (0, 0) at the start, then drifting)
        #   child_frame_id = 'base_link'  -> it's the position of the robot's body
        #   pose.pose                     -> position + orientation (the outer
        #                                    'pose' also holds its uncertainty)
        #   twist.twist                   -> velocity: linear.x = forward speed (m/s),
        #                                    angular.z = turning rate (rad/s), with the
        #                                    same scale errors as the wheels
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = odom_pose.x
        odom.pose.pose.position.y = odom_pose.y
        odom.pose.pose.orientation = quaternion_msg(odom_pose.yaw)
        odom.twist.twist.linear.x = v * (1.0 + self.odom_config.distance_scale_error)
        odom.twist.twist.angular.z = omega * (1.0 + self.odom_config.turn_scale_error)
        self.odom_pub.publish(odom)

        # TF: the chain map -> odom -> base_link. odom -> base_link is the odometry
        # pose; map -> odom is RTAB-Map's correction, chosen so the chain ends at the
        # SLAM pose. enu -> map (the true alignment) is for RViz only (see the module
        # docstring). sendTransform accepts a list and publishes them all at once.
        if self.publish_tf:
            self.tf_broadcaster.sendTransform([
                transform_msg(ENU_FRAME, MAP_FRAME, map_in_enu, stamp),
                transform_msg(MAP_FRAME, ODOM_FRAME, map_to_odom(map_pose, odom_pose), stamp),
                transform_msg(ODOM_FRAME, BASE_FRAME, odom_pose, stamp),
            ])

        # IMU: the robot's orientation and turning rate, as a motion sensor chip
        # would report it. Its frame is base_link because it's mounted on the robot.
        # The heading is absolute (relative to east, like a compass) and gets the
        # configured bias and noise; the turning rate (gyro) stays exact.
        # We don't simulate acceleration; by ROS convention, setting the first
        # covariance entry to -1 tells consumers "this field isn't provided" so
        # they ignore it instead of trusting a zero.
        imu_yaw = wrap_angle(true_pose.yaw + self.imu_yaw_bias
                             + self.imu_rng.gauss(0.0, 1.0) * self.imu_yaw_noise)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = BASE_FRAME
        imu.orientation = quaternion_msg(imu_yaw)
        imu.angular_velocity.z = omega
        imu.linear_acceleration_covariance[0] = -1.0  # -1 means "not provided"
        self.imu_pub.publish(imu)

        # Ground truth (enu frame), for evaluation only.
        if self.publish_truth:
            self.truth_pose_pub.publish(pose_msg(ENU_FRAME, true_pose, stamp))
            self.truth_map_pub.publish(pose_msg(ENU_FRAME, map_in_enu, stamp))

    def publish_gps(self):
        """Publish the robot's position as a GPS fix, with the errors of its zone.

        Called by the GPS timer 5 times a second. Looks up the GPS conditions where
        the robot truly is, lets the error model produce the receiver's output, and
        converts it into latitude/longitude, as a real GPS receiver would output.
        """
        with self.lock:
            x, y = self.robot.state.x, self.robot.state.y
        # Seconds since the clock's epoch, as a float (nanoseconds * 1e-9).
        t = self.get_clock().now().nanoseconds * 1e-9
        conditions, zone = self.zone_map.conditions_at(x, y)
        if zone != self.gps_zone:
            self.get_logger().info(f"GPS zone: {zone or 'default'}")
            self.gps_zone = zone
        reading = self.gps_model.read(x, y, t, conditions)
        if not reading.available and not self.gps_publish_no_fix:
            return

        # NavSatFix is ROS's standard GPS message. Fields used:
        #   header.frame_id = 'gps_link' -> the GPS antenna's frame on the robot
        #   status.status = STATUS_FIX   -> "I have a valid position"
        #                   STATUS_NO_FIX -> no position (lat/lon are then NaN,
        #                                    "not a number", and must be ignored)
        #   status.service = SERVICE_GPS -> which satellite system is used
        #   position_covariance          -> 3x3 matrix (east, north, up) in m^2, row by
        #                                   row; DIAGONAL_KNOWN = only the diagonal is
        #                                   filled. UNKNOWN = no accuracy information.
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.altitude = 0.0
        if reading.available:
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.latitude, fix.longitude = enu_to_latlon(
                reading.east, reading.north, self.origin_lat, self.origin_lon)
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.latitude = fix.longitude = fix.altitude = math.nan
        if self.report_gps_covariance and reading.available:
            horizontal = reading.reported_sigma ** 2
            fix.position_covariance = [horizontal, 0.0, 0.0,
                                       0.0, horizontal, 0.0,
                                       0.0, 0.0, 4.0 * horizontal]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_pub.publish(fix)

    # ------------------------------------------------------------------
    # Fake Nav2
    # ------------------------------------------------------------------

    def execute_goal(self, goal_handle):
        """Drive to one goal, sending feedback, until reached, cancelled or replaced.

        Called by the action server once for each accepted goal, in its own thread.
        `goal_handle` represents that one goal: it holds the request and has
        methods to send feedback and to finish the goal with a status.
        """
        # goal_handle.request is the goal part of the NavigateToPose action:
        # a PoseStamped, i.e. a position + orientation tagged with a frame.
        target = goal_handle.request.pose

        # Goals must be in the map frame, as for real Nav2. A goal in another frame
        # (e.g. base_link = relative to the robot) would mean a different place, so
        # warn rather than silently misinterpret it.
        if target.header.frame_id not in ('', MAP_FRAME):
            self.get_logger().warn(
                f"Goal is in frame '{target.header.frame_id}', treating it as 'map'")

        gx, gy = target.pose.position.x, target.pose.position.y
        self.get_logger().info(f'New goal: x={gx:.1f} m, y={gy:.1f} m (map frame)')

        # Take control of the robot. Pointing active_goal at this goal also tells
        # any older goal still running that it has been replaced. The model drives
        # in the true enu frame, so the map-frame goal is converted with where the
        # (fake) SLAM map frame currently is: like real Nav2, the robot drives to the
        # place SLAM believes the goal is.
        with self.lock:
            self.active_goal = goal_handle  # a newer goal replaces any older one
            self.robot.set_goal(*transform_point(self.slam.map_in_enu, gx, gy))

        # Remember the start time for the feedback's navigation_time field, and
        # create one Feedback message (fields defined by the action) to reuse.
        start = self.get_clock().now()
        feedback = NavigateToPose.Feedback()

        # Check progress 10 times a second until the goal ends.
        # rclpy.ok() stays True until ROS is shut down (e.g. Ctrl+C).
        while rclpy.ok():
            # Decide what's happening, in order of priority: replaced by a newer
            # goal, cancel requested by the client, arrived, or still driving.
            # goal_handle.is_cancel_requested is set by ROS when the client asks
            # to cancel. Cancelled/reached goals release the robot; a replaced goal
            # must not, because the robot now belongs to the newer goal. All of
            # this happens inside one lock so a new goal can't slip in between
            # the check and the release.
            with self.lock:
                if self.active_goal is not goal_handle:
                    outcome = 'replaced'
                elif goal_handle.is_cancel_requested:
                    outcome = 'cancelled'
                else:
                    # Re-convert the goal: the map frame drifts while we drive.
                    self.robot.set_goal(*transform_point(self.slam.map_in_enu, gx, gy))
                    outcome = 'reached' if self.robot.goal_reached() else None
                if outcome in ('cancelled', 'reached'):
                    self.robot.clear_goal()
                    self.active_goal = None
                distance = self.robot.distance_to_goal()
                map_pose = self.map_pose

            # End the goal if needed. The status call tells the client how the goal
            # ended (this is the main answer it receives):
            #   abort()    -> failed / stopped by the server
            #   canceled() -> stopped because the client asked
            #   succeed()  -> completed
            # Returning a Result message is required; its error_code defaults to 0
            # ("no error").
            if outcome == 'replaced':
                self.get_logger().info('Goal replaced by a newer one')
                goal_handle.abort()
                return NavigateToPose.Result()
            if outcome == 'cancelled':
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                return NavigateToPose.Result()
            if outcome == 'reached':
                self.get_logger().info('Goal reached')
                goal_handle.succeed()
                return NavigateToPose.Result()

            # Still driving: send progress to the client. These are fields of the
            # NavigateToPose feedback definition (see `ros2 interface show
            # nav2_msgs/action/NavigateToPose`). current_pose is where SLAM thinks
            # the robot is, in the map frame, as real Nav2 reports it. Subtracting two
            # ROS times gives a Duration, which .to_msg() converts for the message.
            # publish_feedback() sends it to the client that sent this goal.
            now = self.get_clock().now()
            feedback.current_pose = pose_msg(MAP_FRAME, map_pose, now.to_msg())
            feedback.distance_remaining = distance
            feedback.navigation_time = (now - start).to_msg()
            goal_handle.publish_feedback(feedback)

            # Wait before checking again. This only pauses this goal's thread;
            # the timers keep running in other threads (see main()).
            time.sleep(0.1)  # send feedback 10 times a second

        # Only reached if ROS shuts down mid-goal (Ctrl+C): end it as failed. The
        # action server may already be gone by then, so a failure here is ignored.
        try:
            goal_handle.abort()
        except Exception:  # any error during shutdown is harmless
            pass
        return NavigateToPose.Result()


def main(args=None):
    """Entry point: start ROS, run the node until Ctrl+C, then clean up.

    `ros2 run guide_sim sim_robot` calls this function (registered in setup.py).
    """
    # Start the ROS library (also reads any --ros-args, e.g. parameter
    # overrides), then create our node, which runs __init__ above.
    rclpy.init(args=args)
    node = SimRobotNode()

    # The executor waits for events (a timer is due, a goal arrives) and calls the
    # matching function. A MultiThreadedExecutor can run several at once, which
    # we need: execute_goal loops for as long as the robot drives, and the
    # physics timer must keep running at the same time. A single-threaded
    # executor would freeze the simulation during every goal.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # spin() runs until Ctrl+C (KeyboardInterrupt) or ROS is shut down from
    # outside (ExternalShutdownException). Either way, `finally` destroys the
    # node and shuts ROS down cleanly; try_shutdown() doesn't complain if ROS
    # is already shut down.
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# Allows running the file directly with `python3 sim_robot_node.py`.
# Normally it's started with `ros2 run`, which calls main() itself.
if __name__ == '__main__':
    main()
