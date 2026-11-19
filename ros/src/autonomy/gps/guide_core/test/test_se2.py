"""Tests for 2D rigid transforms."""

import math

from guide_core.se2 import (
    compose, IDENTITY, inverse, Pose2D, relative, transform_point, wrap_angle)
import pytest


def assert_pose(actual, expected):
    """Assert two poses are equal to within 1e-9 (yaw compared on the circle)."""
    assert actual.x == pytest.approx(expected.x, abs=1e-9)
    assert actual.y == pytest.approx(expected.y, abs=1e-9)
    assert wrap_angle(actual.yaw - expected.yaw) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0), (math.pi, -math.pi), (-math.pi, -math.pi), (3 * math.pi / 2, -math.pi / 2),
    (-5 * math.pi / 2, -math.pi / 2), (0.1, 0.1),
])
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected)


def test_compose_rotates_the_second_pose_into_the_first_frame():
    a = Pose2D(10.0, 5.0, math.pi / 2)
    assert_pose(compose(a, Pose2D(2.0, 0.0, 0.0)), Pose2D(10.0, 7.0, math.pi / 2))
    assert_pose(compose(a, Pose2D(0.0, 1.0, math.pi / 2)), Pose2D(9.0, 5.0, math.pi))


def test_inverse_undoes_compose():
    a = Pose2D(3.0, -4.0, 0.7)
    assert_pose(compose(a, inverse(a)), IDENTITY)
    assert_pose(compose(inverse(a), a), IDENTITY)


def test_transform_point_matches_compose():
    a = Pose2D(1.0, 2.0, -2.0)
    x, y = transform_point(a, 3.0, 4.0)
    p = compose(a, Pose2D(3.0, 4.0, 0.0))
    assert (x, y) == pytest.approx((p.x, p.y))


def test_relative_expresses_b_in_a():
    start = Pose2D(0.0, 0.0, math.pi / 2)
    robot = Pose2D(10.0, 0.0, math.pi / 2)
    # Facing north, 10 m east is 10 m to the right: y = -10.
    assert_pose(relative(start, robot), Pose2D(0.0, -10.0, 0.0))
