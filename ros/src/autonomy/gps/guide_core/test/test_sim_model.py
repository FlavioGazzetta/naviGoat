"""Tests for guide_core.angles and guide_core.sim_model."""
import math

from guide_core.angles import quaternion_to_yaw, wrap_angle, yaw_to_quaternion
from guide_core.sim_model import SimRobot

DT = 0.05  # seconds per step (20 Hz)


def run_until_reached(robot, max_time=60.0):
    """Step the robot until it reaches its goal; return the time taken, or None."""
    t = 0.0
    while t < max_time:
        if robot.goal_reached():
            return t
        robot.step(DT)
        t += DT
    return None


def test_wrap_angle_takes_the_short_way_round():
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2, abs_tol=1e-9)
    assert math.isclose(wrap_angle(-3 * math.pi / 2), math.pi / 2, abs_tol=1e-9)


def test_yaw_quaternion_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 2.0, 3.1):
        assert math.isclose(quaternion_to_yaw(*yaw_to_quaternion(yaw)), yaw, abs_tol=1e-9)


def test_stands_still_without_a_goal():
    robot = SimRobot()
    for _ in range(100):
        robot.step(DT)
    assert robot.state.x == 0.0 and robot.state.y == 0.0


def test_reaches_a_goal_straight_ahead():
    robot = SimRobot()
    robot.set_goal(10.0, 0.0)
    assert run_until_reached(robot) is not None
    assert robot.distance_to_goal() < robot.goal_tolerance


def test_reaches_a_goal_behind_it():
    robot = SimRobot()
    robot.set_goal(-5.0, 0.0)
    assert run_until_reached(robot) is not None


def test_reaches_a_goal_off_to_the_side():
    robot = SimRobot()
    robot.set_goal(3.0, 8.0)
    assert run_until_reached(robot) is not None


def test_never_exceeds_max_speed():
    robot = SimRobot(max_speed=1.0)
    robot.set_goal(20.0, 5.0)
    for _ in range(400):
        robot.step(DT)
        assert robot.state.v <= robot.max_speed + 1e-9
