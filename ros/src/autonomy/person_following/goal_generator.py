"""Planar following goals and update throttling, independent of ROS/Nav2."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowingGoal:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GoalConfig:
    distance_m: float = 1.0
    tolerance_m: float = 0.15
    update_hz: float = 2.0
    movement_threshold_m: float = 0.15
    heading_threshold_rad: float = 0.2

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError("Goal parameters must be finite and positive")
        if self.tolerance_m >= self.distance_m:
            raise ValueError("Tolerance must be smaller than following distance")


def camera_point_on_map(x, y, transform, max_tilt_rad=0.05):
    """Apply a TF transform to Enid's level-camera point (x forward, y left).

    transform = (tx, ty, qx, qy, qz, qw). The tracker provides no vertical
    coordinate, so reject tilted camera frames rather than inventing height.
    """
    if len(transform) != 6 or not all(math.isfinite(v) for v in (x, y, *transform)):
        raise ValueError("Position and transform must be finite")
    tx, ty, qx, qy, qz, qw = transform
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-9:
        raise ValueError("Invalid rotation quaternion")
    qx, qy, qz, qw = (v / norm for v in (qx, qy, qz, qw))
    if 1 - 2*(qx*qx + qy*qy) < math.cos(max_tilt_rad):
        raise ValueError("Tracker requires a level camera; full 3D target position is unavailable")
    return (tx + (1 - 2*(qy*qy + qz*qz))*x + 2*(qx*qy - qz*qw)*y,
            ty + 2*(qx*qy + qz*qw)*x + (1 - 2*(qx*qx + qz*qz))*y)


class GoalGenerator:
    def __init__(self, config=None):
        self.config = config if config is not None else GoalConfig()
        self.reset()

    def reset(self):
        self._last_goal = None
        self._last_sent = None
        self._last_tick = None

    def update(self, robot, target, now_s):
        """Return ('send', goal), ('hold', None), or ('wait', None).

        robot and target are (x,y) in one fixed navigation frame. Stand off on
        the robot's side of the person, facing them. Already close: hold rather
        than issuing a backwards destination. Distance is measured from base_link.
        """
        if (not all(math.isfinite(v) for v in (*robot, *target, now_s))
                or (self._last_tick is not None and now_s < self._last_tick)):
            raise ValueError("Finite positions and monotonic control time required")
        self._last_tick = now_s
        dx, dy = target[0] - robot[0], target[1] - robot[1]
        gap = math.hypot(dx, dy)
        if gap <= self.config.distance_m + self.config.tolerance_m:
            self._last_goal = None
            return "hold", None
        fraction = (gap - self.config.distance_m) / gap
        goal = FollowingGoal(robot[0] + fraction*dx, robot[1] + fraction*dy, math.atan2(dy, dx))
        if self._last_sent is not None and now_s - self._last_sent < 1 / self.config.update_hz:
            return "wait", None
        if self._last_goal is not None:
            movement = math.hypot(goal.x - self._last_goal.x, goal.y - self._last_goal.y)
            heading = abs(math.atan2(math.sin(goal.yaw - self._last_goal.yaw),
                                     math.cos(goal.yaw - self._last_goal.yaw)))
            if movement < self.config.movement_threshold_m and heading < self.config.heading_threshold_rad:
                return "wait", None
        self._last_sent, self._last_goal = now_s, goal
        return "send", goal
