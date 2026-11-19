"""Simulated wheel odometry and fake SLAM for the simulator (Step 8).

OdomErrorModel turns the robot's true motion into what a skid-steer base's
wheel odometry reports: a pose in the odom frame that starts at (0, 0, 0)
wherever the robot really is, with
* a distance scale error (wheel size, tyre wear),
* a turn scale error (skid-steer wheels slip sideways when turning, so the
  measured turn is systematically too large or too small),
* a heading drift per metre driven,
* random noise growing with the square root of the distance / angle.

FakeSlam stands in for RTAB-Map: its map frame starts at the robot's true
start pose (x forward, as RTAB-Map does) and drifts slowly relative to the
true ENU frame as the robot drives. The drift turns the map frame about the
robot's position, so the robot's map pose stays smooth while its heading
error grows. It reports the robot's pose in the map frame and the map -> odom
transform that makes map -> odom -> base_link equal that pose. With
map_at_start False, the map frame is the ENU frame (the v0 assumption) and
only the drift remains.

All randomness comes from seeded random.Random generators.
"""

from dataclasses import dataclass, fields, replace
import math
import random

from guide_core.se2 import compose, IDENTITY, inverse, Pose2D, relative


@dataclass(frozen=True)
class OdomErrorConfig:
    """Error parameters of the wheel odometry."""

    # Measured distance = true distance * (1 + distance_scale_error).
    distance_scale_error: float = 0.0
    # Measured turn = true turn * (1 + turn_scale_error).
    turn_scale_error: float = 0.0
    # Extra yaw (rad) added per metre driven, sign included.
    heading_drift_per_metre: float = 0.0
    # Noise sigma: metres per sqrt(metre) driven.
    distance_noise: float = 0.0
    # Noise sigma: radians per sqrt(metre) driven.
    heading_noise: float = 0.0
    # Noise sigma: radians per sqrt(radian) turned.
    turn_noise: float = 0.0

    def __post_init__(self):
        for name in ('distance_noise', 'heading_noise', 'turn_noise'):
            if getattr(self, name) < 0.0:
                raise ValueError(f'{name} must be >= 0')
        for name in ('distance_scale_error', 'turn_scale_error'):
            if not -0.5 < getattr(self, name) < 0.5:
                raise ValueError(f'{name} must be between -0.5 and 0.5')


@dataclass(frozen=True)
class SlamDriftConfig:
    """How fast the fake SLAM map frame drifts relative to the true ENU frame."""

    # True: the map frame starts at the robot's start pose (like RTAB-Map).
    map_at_start: bool = True
    # Random-walk sigma of the map origin position: metres per sqrt(metre) driven.
    position_drift: float = 0.0
    # Random-walk sigma of the map rotation: radians per sqrt(metre) driven.
    yaw_drift: float = 0.0

    def __post_init__(self):
        if self.position_drift < 0.0 or self.yaw_drift < 0.0:
            raise ValueError('drift values must be >= 0')


# Named starting values; tune once the team has real odometry logs.
ODOM_PROFILES = {
    'perfect': OdomErrorConfig(),
    'skid_steer': OdomErrorConfig(
        distance_scale_error=0.02, turn_scale_error=0.08,
        heading_drift_per_metre=0.001, distance_noise=0.01,
        heading_noise=0.003, turn_noise=0.02),
}
SLAM_PROFILES = {
    'perfect': SlamDriftConfig(),
    'enu': SlamDriftConfig(map_at_start=False),
    'lidar': SlamDriftConfig(position_drift=0.01, yaw_drift=0.0005),
}


def _from_dict(cls, profiles, values, kind):
    """Return a config of cls from a dict with an optional 'profile' key."""
    values = dict(values or {})
    base = profiles['perfect']
    if 'profile' in values:
        name = values.pop('profile')
        if name not in profiles:
            raise ValueError(f'unknown {kind} profile {name!r}; known: {sorted(profiles)}')
        base = profiles[name]
    unknown = sorted(set(values) - {f.name for f in fields(cls)})
    if unknown:
        raise ValueError(f'unknown {kind} fields: {unknown}')
    return replace(base, **values)


def odom_config_from_dict(values):
    """Return an OdomErrorConfig ('perfect' unless a profile is named)."""
    return _from_dict(OdomErrorConfig, ODOM_PROFILES, values, 'odometry')


def slam_config_from_dict(values):
    """Return a SlamDriftConfig ('perfect' unless a profile is named)."""
    return _from_dict(SlamDriftConfig, SLAM_PROFILES, values, 'SLAM drift')


def body_motion(previous, current):
    """Return (forward distance, yaw change) between two true poses.

    The forward distance is the displacement projected on the mean heading,
    so it is negative when reversing; sideways slip is ignored.
    """
    dx, dy = current.x - previous.x, current.y - previous.y
    dyaw = math.atan2(math.sin(current.yaw - previous.yaw),
                      math.cos(current.yaw - previous.yaw))
    mid = previous.yaw + dyaw / 2.0
    return dx * math.cos(mid) + dy * math.sin(mid), dyaw


class OdomErrorModel:
    """Integrate true body motion into a drifting odometry pose in the odom frame."""

    def __init__(self, config, seed=0):
        self.config = config
        self._rng = random.Random(seed)
        self.pose = IDENTITY
        self.distance = 0.0

    def step(self, forward, dyaw):
        """Add one true motion, given as forward metres and yaw change; return the pose."""
        c = self.config
        dist = abs(forward)
        measured = forward * (1.0 + c.distance_scale_error) \
            + self._rng.gauss(0.0, 1.0) * c.distance_noise * math.sqrt(dist)
        turn = dyaw * (1.0 + c.turn_scale_error) \
            + c.heading_drift_per_metre * dist \
            + self._rng.gauss(0.0, 1.0) * c.heading_noise * math.sqrt(dist) \
            + self._rng.gauss(0.0, 1.0) * c.turn_noise * math.sqrt(abs(dyaw))
        # Midpoint integration: move along the heading halfway through the turn.
        self.pose = compose(self.pose, Pose2D(0.0, 0.0, turn / 2.0))
        self.pose = compose(self.pose, Pose2D(measured, 0.0, turn / 2.0))
        self.distance += dist
        return self.pose


class FakeSlam:
    """Stand-in for RTAB-Map: robot pose in a drifting map frame."""

    def __init__(self, start_pose, config, seed=0):
        self.config = config
        self._rng = random.Random(seed)
        # The map frame expressed in ENU (ground truth, for evaluation only).
        self.map_in_enu = Pose2D(*start_pose) if config.map_at_start else IDENTITY

    def step(self, true_pose, distance=0.0):
        """Drift the map frame by the distance driven and return the robot's map pose."""
        true_pose = Pose2D(*true_pose)
        if distance > 0.0:
            # Drift happens where the robot is, as with real SLAM: the map frame turns
            # about the robot's position (so the robot's map pose stays continuous and
            # only its heading error grows) and shifts a little.
            root = math.sqrt(distance)
            dx = self._rng.gauss(0.0, 1.0) * self.config.position_drift * root
            dy = self._rng.gauss(0.0, 1.0) * self.config.position_drift * root
            dyaw = self._rng.gauss(0.0, 1.0) * self.config.yaw_drift * root
            c, s = math.cos(dyaw), math.sin(dyaw)
            x, y = true_pose.x, true_pose.y
            about_robot = Pose2D(x - c * x + s * y + dx, y - s * x - c * y + dy, dyaw)
            self.map_in_enu = compose(about_robot, self.map_in_enu)
        return relative(self.map_in_enu, true_pose)


def map_to_odom(map_pose, odom_pose):
    """Return the map -> odom transform so that map -> odom -> base_link = map_pose."""
    return compose(map_pose, inverse(odom_pose))
