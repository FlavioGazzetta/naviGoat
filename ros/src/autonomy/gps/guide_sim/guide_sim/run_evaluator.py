"""Run evaluator (dev tool, simulator only): scores a trip against ground truth.

Compares what the guide stack believes with what the simulator knows is true
(sim/ground_truth/pose, in the enu frame) and prints a summary when the trip
ends (ARRIVED or FAILED) or on Ctrl+C:

- localisation: error between guide/pose and the true pose (median, 95 %,
  max) and how often it was within 3 sigma (honesty);
- every crossing: how far before the TRUE near kerb waiting began, how close
  the true robot came to it before the go, and a VIOLATION if the true robot
  passed the near kerb before the user's go (the
  Step 8 done criterion). The near kerb is the follower's own crossing start
  (distance_along + distance_to_next_crossing in guide/route_progress), the
  true position is projected onto guide/route;
- the mission: time spent in each state, reroutes, true arrival error.

With auto_go true it also presses go (guide/user_go) go_delay seconds after the
mission reaches READY, WAITING_AT_CROSSING or PAUSED, so a long test trip runs
unattended. This replaces the user's judgement: for simulation only.

Not part of the production build (it lives in guide_sim).
"""

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from guide_core.route import Route
from guide_interfaces.msg import LocalizationStatus, MissionState, RouteProgress
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

# Latched QoS (reliable + transient_local, depth 1): guide/route and
# guide/mission_state are latched, so a subscriber needs the same profile.
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

# MissionState numbers -> names, built from the message's own constants.
STATE_NAMES = {getattr(MissionState, n): n for n in (
    'IDLE', 'PLANNING', 'READY', 'NAVIGATING', 'WAITING_AT_CROSSING', 'PAUSED', 'ARRIVED',
    'FAILED', 'GPS_DEGRADED', 'POSITION_LOST', 'REROUTING')}
# States in which auto_go presses go.
GO_STATES = (MissionState.READY, MissionState.WAITING_AT_CROSSING, MissionState.PAUSED)


def percentile(values, q):
    """Return the q-quantile (0..1) of a list, or nan if it is empty."""
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


class RunEvaluator(Node):
    """Collects ground-truth comparisons during one trip and prints a summary."""

    def __init__(self):
        super().__init__('run_evaluator')
        # auto_go: press go automatically (test runs); go_delay: seconds to wait first.
        self.auto_go = self.declare_parameter('auto_go', False).value
        self.go_delay = self.declare_parameter('go_delay', 3.0).value

        # Latest messages.
        self.truth = None          # (x, y) true position in enu
        self.route = None          # guide_core Route of the current guide/route
        self.state = None          # current MissionState number
        self.state_since = None    # time (s) the current state began
        self.quality = None

        # Results.
        self.loc_errors = []       # metres, one per second
        self.loc_honest = 0        # samples with error <= 3 sigma
        self.state_time = {}       # state name -> seconds
        self.state_count = {}      # state name -> times entered
        self.crossings = {}        # (route_id, index) -> dict of results
        self.go_presses = 0
        self.arrival_error = None
        self.summary_printed = False
        self.go_timer = None

        # Subscriptions. The ground truth and guide/pose are reliable streams.
        self.create_subscription(PoseStamped, 'sim/ground_truth/pose', self.on_truth, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'guide/pose', self.on_pose, 10)
        self.create_subscription(LocalizationStatus, 'guide/localization_status',
                                 self.on_status, 10)
        self.create_subscription(Path, 'guide/route', self.on_route, LATCHED_QOS)
        self.create_subscription(RouteProgress, 'guide/route_progress', self.on_progress, 10)
        self.create_subscription(MissionState, 'guide/mission_state', self.on_state,
                                 LATCHED_QOS)
        self.go_client = self.create_client(Trigger, 'guide/user_go')
        self.latest_pose = None
        # Compare the estimate with the truth once per second.
        self.create_timer(1.0, self.sample_localization)
        self.get_logger().info(
            'Run evaluator ready' + (f' (auto_go after {self.go_delay:.0f} s)'
                                     if self.auto_go else ''))

    def now(self):
        """Return the node clock's time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    # --- Inputs -------------------------------------------------------------

    def on_truth(self, msg):
        self.truth = (msg.pose.position.x, msg.pose.position.y)

    def on_pose(self, msg):
        self.latest_pose = msg

    def on_status(self, msg):
        self.quality = msg.quality

    def on_route(self, msg):
        # An empty Path means "no route" (cleared).
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.route = Route(points) if len(points) >= 2 else None

    def sample_localization(self):
        """Record the estimate's error against the truth (and whether within 3 sigma)."""
        if self.truth is None or self.latest_pose is None:
            return
        p = self.latest_pose.pose.pose.position
        cov = self.latest_pose.pose.covariance
        sigma = math.sqrt(max(cov[0], cov[7], 0.0))
        error = math.hypot(p.x - self.truth[0], p.y - self.truth[1])
        self.loc_errors.append(error)
        if error <= 3.0 * sigma:
            self.loc_honest += 1

    def on_progress(self, msg):
        """Track every crossing: the true robot must not pass the near kerb before go."""
        if self.route is None or self.truth is None or msg.next_crossing_index < 0:
            return
        key = (msg.route_id, msg.next_crossing_index)
        near_kerb = msg.distance_along + msg.distance_to_next_crossing
        true_s = self.route.project(*self.truth).s
        c = self.crossings.setdefault(key, {'near_kerb': near_kerb, 'max_true_s': -math.inf,
                                            'stop_gap': None, 'violation': False})
        c['near_kerb'] = near_kerb
        c['max_true_s'] = max(c['max_true_s'], true_s)
        if msg.waiting_at_crossing and c['stop_gap'] is None:
            c['stop_gap'] = near_kerb - true_s
        if true_s > near_kerb and not c['violation']:
            c['violation'] = True
            self.get_logger().error(
                f'VIOLATION at crossing {key[1] + 1} (route {key[0]}): the true robot is '
                f'{true_s - near_kerb:.1f} m past the near kerb before the go')

    def on_state(self, msg):
        """Time each mission state; at the end of the trip print the summary."""
        now = self.now()
        if self.state is not None and self.state_since is not None:
            name = STATE_NAMES.get(self.state, str(self.state))
            self.state_time[name] = self.state_time.get(name, 0.0) + now - self.state_since
        if msg.state != self.state:
            name = STATE_NAMES.get(msg.state, str(msg.state))
            self.state_count[name] = self.state_count.get(name, 0) + 1
        self.state, self.state_since = msg.state, now

        if msg.state == MissionState.ARRIVED and self.truth is not None and self.route:
            end = self.route.points[-1]
            self.arrival_error = math.hypot(self.truth[0] - end[0], self.truth[1] - end[1])
        if msg.state in (MissionState.ARRIVED, MissionState.FAILED):
            self.print_summary(STATE_NAMES[msg.state] + ': ' + msg.message)
        if self.auto_go and msg.state in GO_STATES:
            self.schedule_go(msg.state)

    # --- auto_go ------------------------------------------------------------

    def schedule_go(self, state):
        """Press go after go_delay, if the mission is still in the same state then."""
        if self.go_timer is not None:
            self.destroy_timer(self.go_timer)

        def press():
            self.destroy_timer(self.go_timer)
            self.go_timer = None
            if self.state != state or not self.go_client.service_is_ready():
                return
            self.go_presses += 1
            self.get_logger().info(f'auto_go: pressing go ({STATE_NAMES[state]})')
            self.go_client.call_async(Trigger.Request())

        self.go_timer = self.create_timer(self.go_delay, press)

    # --- Summary ------------------------------------------------------------

    def print_summary(self, ending):
        """Log the results of the trip (once)."""
        if self.summary_printed:
            return
        self.summary_printed = True
        log = self.get_logger().info
        n = len(self.loc_errors)
        log('===== Run summary =====')
        log(f'Ended: {ending}')
        if n:
            log(f'Localization error: median {percentile(self.loc_errors, 0.5):.2f} m, '
                f'95% {percentile(self.loc_errors, 0.95):.2f} m, '
                f'max {max(self.loc_errors):.2f} m; within 3 sigma '
                f'{100.0 * self.loc_honest / n:.1f}% of {n} s')
        violations = 0
        for (route_id, index), c in sorted(self.crossings.items()):
            closest = c['near_kerb'] - c['max_true_s']
            gap = ('waiting not reached' if c['stop_gap'] is None
                   else f"waiting began {c['stop_gap']:.1f} m before the true near kerb")
            gap += f', closest {closest:.1f} m before it'
            flag = 'VIOLATION' if c['violation'] else 'ok'
            violations += c['violation']
            log(f'Crossing {index + 1} (route {route_id}): {gap} [{flag}]')
        log(f'Crossing violations: {violations}')
        states = ', '.join(f'{k} {v:.0f} s' for k, v in sorted(self.state_time.items()))
        log(f'Time per state: {states}')
        log(f"Reroutes: {self.state_count.get('REROUTING', 0)}, GPS_DEGRADED entered "
            f"{self.state_count.get('GPS_DEGRADED', 0)}x, POSITION_LOST entered "
            f"{self.state_count.get('POSITION_LOST', 0)}x, go presses (auto) "
            f'{self.go_presses}')
        if self.arrival_error is not None:
            log(f'True arrival error: {self.arrival_error:.2f} m from the route end')
        log('=======================')


def main(args=None):
    rclpy.init(args=args)
    node = RunEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.print_summary('stopped (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
