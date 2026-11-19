"""2D rigid transforms (x, y, yaw) for frames such as enu, map and odom.

A Pose2D "a" is read as "frame B expressed in frame A" (or a robot pose in
frame A). compose(a, b) chains them: if a is B in A and b is C in B, the
result is C in A. inverse(a) is A in B. Yaw follows REP 103 (counter-
clockwise positive, radians) and is always wrapped to [-pi, pi).
"""

import math
from typing import NamedTuple


def wrap_angle(angle):
    """Return the angle wrapped to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Pose2D(NamedTuple):
    """A 2D pose or transform: position in metres, yaw in radians."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


IDENTITY = Pose2D()


def compose(a, b):
    """Return a followed by b (b expressed in a's frame)."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y,
                  wrap_angle(a.yaw + b.yaw))


def inverse(a):
    """Return the inverse transform of a."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(-c * a.x - s * a.y, s * a.x - c * a.y, wrap_angle(-a.yaw))


def transform_point(a, x, y):
    """Return the point (x, y), given in a's child frame, in a's parent frame."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return a.x + c * x - s * y, a.y + s * x + c * y


def relative(a, b):
    """Return b expressed in a's frame: inverse(a) composed with b."""
    return compose(inverse(a), b)
