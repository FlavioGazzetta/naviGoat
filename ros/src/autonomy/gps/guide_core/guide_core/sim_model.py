"""A simple 2D robot that drives towards a goal. Used by the simulator.

Stands in for both the real robot and Nav2: given a goal, it turns towards it
and drives there in a straight line. No obstacles, no noise (yet).
"""
from dataclasses import dataclass
import math

from guide_core.angles import wrap_angle


@dataclass
class RobotState:
    """Where the robot is and how it is moving."""

    x: float = 0.0      # metres east of the origin
    y: float = 0.0      # metres north of the origin
    yaw: float = 0.0    # heading in radians (0 = east, pi/2 = north)
    v: float = 0.0      # forward speed, m/s
    omega: float = 0.0  # turn rate, rad/s (positive = turning left)


class SimRobot:
    """Unicycle-model robot with a single goal at a time."""

    def __init__(self, max_speed=1.0, max_turn_rate=1.0, goal_tolerance=0.3,
                 slowdown_distance=2.0):
        self.max_speed = max_speed                  # m/s (about walking pace)
        self.max_turn_rate = max_turn_rate          # rad/s
        self.goal_tolerance = goal_tolerance        # metres; closer than this = arrived
        self.slowdown_distance = slowdown_distance  # metres; start slowing within this
        self.state = RobotState()
        self.goal = None                            # (x, y) or None

    def set_goal(self, x, y):
        self.goal = (x, y)

    def clear_goal(self):
        self.goal = None

    def distance_to_goal(self):
        if self.goal is None:
            return 0.0
        return math.hypot(self.goal[0] - self.state.x, self.goal[1] - self.state.y)

    def goal_reached(self):
        return self.goal is not None and self.distance_to_goal() < self.goal_tolerance

    def step(self, dt):
        """Advance the simulation by dt seconds."""
        s = self.state

        # No goal, or already there: stand still.
        if self.goal is None or self.goal_reached():
            s.v = 0.0
            s.omega = 0.0
            return

        dx = self.goal[0] - s.x
        dy = self.goal[1] - s.y
        distance = math.hypot(dx, dy)
        heading_error = wrap_angle(math.atan2(dy, dx) - s.yaw)

        # Turn towards the goal: faster when further off, but never above the limit.
        s.omega = max(-self.max_turn_rate,
                      min(self.max_turn_rate, 2.0 * heading_error))

        # Drive forwards only when roughly facing the goal, and slow down near it.
        speed = self.max_speed * min(1.0, distance / self.slowdown_distance)
        s.v = speed * max(0.0, math.cos(heading_error))

        # Move: update heading, then position along the new heading.
        s.yaw = wrap_angle(s.yaw + s.omega * dt)
        s.x += s.v * math.cos(s.yaw) * dt
        s.y += s.v * math.sin(s.yaw) * dt
