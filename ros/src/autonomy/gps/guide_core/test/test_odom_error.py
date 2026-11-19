"""Tests for the simulated odometry drift and the fake SLAM."""

from dataclasses import replace
import math
import statistics

from guide_core.odom_error import (
    body_motion, FakeSlam, map_to_odom, odom_config_from_dict, ODOM_PROFILES,
    OdomErrorConfig, OdomErrorModel, slam_config_from_dict, SLAM_PROFILES, SlamDriftConfig)
from guide_core.se2 import compose, Pose2D, wrap_angle
import pytest

PERFECT = OdomErrorConfig()


def straight(model, metres, step=0.1):
    """Drive the model straight ahead and return the final pose."""
    n = max(1, int(round(abs(metres) / step)))
    for _ in range(n):
        pose = model.step(metres / n, 0.0)
    return pose


def turn(model, radians, step=0.05):
    """Turn the model on the spot and return the final pose."""
    n = max(1, int(round(abs(radians) / step)))
    for _ in range(n):
        pose = model.step(0.0, radians / n)
    return pose


def square(model, side=20.0):
    """Drive a counter-clockwise square and return the final pose."""
    for _ in range(4):
        straight(model, side)
        pose = turn(model, math.pi / 2)
    return pose


def true_path(start, legs):
    """Return true poses for legs of (forward metres, yaw change) applied in small steps."""
    poses = [Pose2D(*start)]
    for forward, dyaw in legs:
        n = max(1, int(round(max(abs(forward) / 0.1, abs(dyaw) / 0.05))))
        for _ in range(n):
            poses.append(compose(poses[-1], Pose2D(forward / n, 0.0, dyaw / n)))
    return poses


def test_perfect_odometry_starts_at_zero_and_closes_a_square():
    model = OdomErrorModel(PERFECT)
    assert model.pose == Pose2D(0.0, 0.0, 0.0)
    pose = square(model)
    assert (pose.x, pose.y) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert wrap_angle(pose.yaw) == pytest.approx(0.0, abs=1e-9)
    assert model.distance == pytest.approx(80.0)


def test_distance_scale_error_stretches_straight_lines():
    model = OdomErrorModel(replace(PERFECT, distance_scale_error=0.02))
    pose = straight(model, 100.0)
    assert (pose.x, pose.y) == pytest.approx((102.0, 0.0))


def test_turn_scale_error_over_turns():
    model = OdomErrorModel(replace(PERFECT, turn_scale_error=0.1))
    assert turn(model, math.pi / 2).yaw == pytest.approx(1.1 * math.pi / 2)


def test_heading_drift_bends_a_straight_line():
    model = OdomErrorModel(replace(PERFECT, heading_drift_per_metre=0.001))
    pose = straight(model, 100.0)
    assert pose.yaw == pytest.approx(0.1)
    # Constant curvature: an arc of radius 1000 m, 100 m long, ends 5.0 m to the left.
    assert pose.y == pytest.approx(1000.0 * (1.0 - math.cos(0.1)), rel=1e-3)


def test_reversing_moves_odometry_backwards():
    model = OdomErrorModel(PERFECT)
    for _ in range(50):
        pose = model.step(-0.1, 0.0)
    assert (pose.x, pose.y) == pytest.approx((-5.0, 0.0))
    assert model.distance == pytest.approx(5.0)


def test_same_seed_gives_same_odometry():
    a = square(OdomErrorModel(ODOM_PROFILES['skid_steer'], seed=3))
    b = square(OdomErrorModel(ODOM_PROFILES['skid_steer'], seed=3))
    c = square(OdomErrorModel(ODOM_PROFILES['skid_steer'], seed=4))
    assert a == b
    assert a != c


def test_distance_noise_grows_with_square_root_of_distance():
    config = replace(PERFECT, distance_noise=0.05)
    finals = [straight(OdomErrorModel(config, seed=s), 100.0).x for s in range(400)]
    assert statistics.mean(finals) == pytest.approx(100.0, abs=0.1)
    assert statistics.pstdev(finals) == pytest.approx(0.05 * 10.0, rel=0.15)


def test_skid_steer_profile_drifts_but_stays_plausible():
    pose = square(OdomErrorModel(ODOM_PROFILES['skid_steer'], seed=1), side=50.0)
    error = math.hypot(pose.x, pose.y)
    # 200 m driven: a few metres off, not zero and not absurd.
    assert 0.5 < error < 40.0


def test_body_motion_recovers_forward_and_turn():
    poses = true_path((5.0, 5.0, 1.0), [(10.0, 0.0), (0.0, math.pi / 2), (-3.0, 0.0)])
    forward = sum(body_motion(a, b)[0] for a, b in zip(poses, poses[1:]))
    turned = sum(body_motion(a, b)[1] for a, b in zip(poses, poses[1:]))
    assert forward == pytest.approx(7.0)
    assert turned == pytest.approx(math.pi / 2)


def test_odometry_from_true_path_ignores_where_the_robot_starts():
    poses = true_path((-26.86, -19.04, math.radians(-82.0)), [(10.0, 0.0)])
    model = OdomErrorModel(PERFECT)
    for a, b in zip(poses, poses[1:]):
        pose = model.step(*body_motion(a, b))
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((10.0, 0.0, 0.0), abs=1e-9)


def test_fake_slam_map_frame_starts_at_the_start_pose():
    start = Pose2D(10.0, 20.0, math.pi / 2)
    slam = FakeSlam(start, SLAM_PROFILES['perfect'])
    assert slam.step(start) == pytest.approx(Pose2D(0.0, 0.0, 0.0))
    # Facing north, walking 5 m north is 5 m forward in the map frame.
    assert slam.step(Pose2D(10.0, 25.0, math.pi / 2)) == pytest.approx(Pose2D(5.0, 0.0, 0.0))


def test_fake_slam_in_enu_mode_reports_the_true_pose():
    slam = FakeSlam(Pose2D(10.0, 20.0, 1.0), SLAM_PROFILES['enu'])
    assert slam.step(Pose2D(3.0, 4.0, 0.5)) == pytest.approx(Pose2D(3.0, 4.0, 0.5))


def test_map_to_odom_closes_the_tf_chain():
    map_pose = Pose2D(12.0, -3.0, 0.4)
    odom_pose = Pose2D(11.5, -1.0, 0.25)
    t = map_to_odom(map_pose, odom_pose)
    assert compose(t, odom_pose) == pytest.approx(map_pose)


def test_fake_slam_drift_grows_with_distance():
    config = SlamDriftConfig(map_at_start=False, position_drift=0.02, yaw_drift=0.0)
    xs = []
    for seed in range(300):
        slam = FakeSlam(Pose2D(), config, seed=seed)
        for _ in range(100):
            pose = slam.step(Pose2D(), distance=1.0)
        xs.append(pose.x)
    assert statistics.pstdev(xs) == pytest.approx(0.02 * 10.0, rel=0.15)


def test_fake_slam_yaw_drift_turns_the_map_about_the_robot():
    config = SlamDriftConfig(map_at_start=False, position_drift=0.0, yaw_drift=0.01)
    slam = FakeSlam(Pose2D(), config, seed=5)
    robot = Pose2D(300.0, 0.0, 0.0)
    for _ in range(100):
        pose = slam.step(robot, distance=1.0)
    # Far from the map origin, yet the robot's map position does not jump.
    assert (pose.x, pose.y) == pytest.approx((300.0, 0.0), abs=1e-6)
    assert abs(pose.yaw) > 0.01


def test_fake_slam_without_distance_does_not_drift():
    slam = FakeSlam(Pose2D(), SLAM_PROFILES['lidar'], seed=1)
    for _ in range(100):
        pose = slam.step(Pose2D(1.0, 2.0, 0.3))
    assert pose == pytest.approx(Pose2D(1.0, 2.0, 0.3))


def test_config_from_dict_uses_profiles_and_overrides():
    odom = odom_config_from_dict({'profile': 'skid_steer', 'turn_noise': 0.0})
    assert odom.turn_scale_error == ODOM_PROFILES['skid_steer'].turn_scale_error
    assert odom.turn_noise == 0.0
    assert odom_config_from_dict({}) == PERFECT
    assert slam_config_from_dict({'profile': 'enu'}).map_at_start is False


@pytest.mark.parametrize('call, values', [
    (odom_config_from_dict, {'profile': 'tracked'}),
    (odom_config_from_dict, {'distance_scale': 0.1}),
    (slam_config_from_dict, {'yaw_drfit': 0.1}),
])
def test_config_from_dict_rejects_unknown_names(call, values):
    with pytest.raises(ValueError):
        call(values)


@pytest.mark.parametrize('cls, kwargs', [
    (OdomErrorConfig, {'distance_noise': -0.1}),
    (OdomErrorConfig, {'turn_scale_error': 0.6}),
    (SlamDriftConfig, {'yaw_drift': -1.0}),
])
def test_invalid_configs_raise(cls, kwargs):
    with pytest.raises(ValueError):
        cls(**kwargs)
