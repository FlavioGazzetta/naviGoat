"""Angle helpers for a robot moving on flat ground.

Headings ("yaw") are in radians: 0 = facing east (+x), positive = counter-clockwise,
so pi/2 = facing north (+y). This is the ROS convention (REP 103).
"""
import math


def wrap_angle(angle):
    """Return the same angle expressed between -pi and pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw):
    """Return the quaternion (x, y, z, w) for a rotation of `yaw` about the vertical axis."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(x, y, z, w):
    """Return the heading (yaw) in radians from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
