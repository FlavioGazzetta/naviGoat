"""Tests for the alignment filter (map frame in ENU) against the sim's error models."""

from dataclasses import dataclass, replace
import math

from guide_core.alignment import (
    AlignmentConfig, AlignmentFilter, FixResult, largest_eigenvalue_2x2)
from guide_core.gps_error import GpsConditions, GpsErrorModel, PROFILES
from guide_core.odom_error import body_motion, FakeSlam, SLAM_PROFILES
from guide_core.se2 import compose, Pose2D, relative, wrap_angle
import pytest

START = Pose2D(-26.86, -19.04, math.radians(-82.0))
# A 680 m walk with four turns, like a trip across South Kensington.
LEGS = [(60, 0.0), (0, 1.2), (120, 0.0), (0, -1.5), (150, 0.0), (0, 0.8), (100, 0.0),
        (0, 1.4), (150, 0.0)]
PERFECT_GPS = GpsConditions(reported_sigma=0.5)


def true_path(start=START, legs=LEGS, step=0.2):
    """Return true poses every `step` metres (5 Hz at 1 m/s); turns on the spot."""
    poses = [start]
    for forward, turn in legs:
        if turn:
            n = max(1, int(abs(turn) / 0.1))
            for _ in range(n):
                poses.append(compose(poses[-1], Pose2D(0.0, 0.0, turn / n)))
        for _ in range(int(round(forward / step))):
            poses.append(compose(poses[-1], Pose2D(step, 0.0, 0.0)))
    return poses


@dataclass
class Run:
    """Summary of one simulated drive."""

    errors: list
    sigmas: list
    results: dict
    filt: AlignmentFilter
    truth_map_in_enu: Pose2D
    map_pose: Pose2D

    def honest(self):
        """Return the fraction of samples whose error is within 3 sigma."""
        return sum(e <= 3.0 * s for e, s in zip(self.errors, self.sigmas)) / len(self.errors)

    def percentile(self, q):
        """Return the q-quantile of the position errors."""
        ordered = sorted(self.errors)
        return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def drive(seed=0, conditions=PROFILES['urban_canyon'], slam='lidar', imu_bias_deg=5.0,
          imu=True, no_fix=None, config=None, gps_offset=None, legs=LEGS, skip=50.0):
    """Drive the path with seeded GPS and SLAM errors and score the filter.

    no_fix: (from, to) distances (m) without GPS. gps_offset(distance) -> (de, dn)
    adds a deliberate offset to fixes. Errors are scored after `skip` metres.
    """
    poses = true_path(legs=legs)
    slam_model = FakeSlam(START, SLAM_PROFILES[slam], seed=seed + 2)
    gps_model = GpsErrorModel(seed=seed)
    filt = AlignmentFilter(config)
    errors, sigmas, results = [], [], {}
    previous, travelled = poses[0], 0.0
    for i, true_pose in enumerate(poses):
        forward, _ = body_motion(previous, true_pose)
        previous = true_pose
        travelled += abs(forward)
        map_pose = slam_model.step(true_pose, abs(forward))
        t = i * 0.2
        if imu:
            filt.heading(wrap_angle(true_pose.yaw + math.radians(imu_bias_deg)), map_pose)
        here = conditions
        if no_fix and no_fix[0] <= travelled <= no_fix[1]:
            here = PROFILES['no_fix']
        reading = gps_model.read(true_pose.x, true_pose.y, t, here)
        if reading.available:
            de, dn = gps_offset(travelled) if gps_offset else (0.0, 0.0)
            update = filt.gps(t, reading.east + de, reading.north + dn,
                              reading.reported_sigma, map_pose)
            results[update.result] = results.get(update.result, 0) + 1
        else:
            filt.predict(map_pose, t)
        if filt.initialised and travelled > skip:
            estimate = filt.robot_pose(map_pose)
            errors.append(math.hypot(estimate.x - true_pose.x, estimate.y - true_pose.y))
            sigmas.append(filt.position_sigma(map_pose))
    return Run(errors, sigmas, results, filt, slam_model.map_in_enu, map_pose)


def test_not_initialised_before_a_fix():
    filt = AlignmentFilter()
    assert not filt.initialised
    assert filt.robot_pose(Pose2D()) is None
    assert filt.position_sigma(Pose2D()) == math.inf
    assert filt.yaw_sigma() == math.inf


def test_initialises_from_first_fix_and_imu_heading():
    filt = AlignmentFilter()
    map_pose = Pose2D(0.0, 0.0, 0.0)
    assert filt.heading(START.yaw, map_pose) is False
    update = filt.gps(0.0, START.x, START.y, 5.0, map_pose)
    assert update.result is FixResult.INITIALISED
    assert filt.transform == pytest.approx(START)
    # White part max(5 * 0.3, 0.5) = 1.5 m and bias part 5 * 0.6 = 3 m.
    assert filt.position_sigma(map_pose) == pytest.approx(math.hypot(1.5, 3.0))
    assert filt.yaw_sigma() == pytest.approx(math.radians(15.0))


def test_without_heading_waits_for_baseline_then_finds_heading_from_travel():
    poses = true_path(legs=[(30, 0.0)])
    filt = AlignmentFilter()
    results = []
    for i, true_pose in enumerate(poses):
        map_pose = relative(START, true_pose)
        results.append(filt.gps(i * 0.2, true_pose.x, true_pose.y, 0.5, map_pose).result)
    first = results.index(FixResult.INITIALISED)
    # 15 m baseline at 0.2 m per fix: 75 fixes.
    assert first == 75
    assert set(results[:first]) == {FixResult.WAITING}
    assert filt.transform.yaw == pytest.approx(START.yaw, abs=1e-6)


def test_perfect_data_gives_the_true_alignment():
    run = drive(conditions=PERFECT_GPS, slam='perfect', imu_bias_deg=0.0)
    assert run.filt.transform == pytest.approx(run.truth_map_in_enu, abs=1e-3)
    assert max(run.errors) < 0.01


def test_fixes_closer_than_min_interval_are_skipped():
    filt = AlignmentFilter()
    map_pose = Pose2D()
    filt.heading(0.0, map_pose)
    assert filt.gps(0.0, 0.0, 0.0, 2.0, map_pose).result is FixResult.INITIALISED
    assert filt.gps(0.4, 0.0, 0.0, 2.0, map_pose).result is FixResult.SKIPPED
    assert filt.gps(1.0, 0.0, 0.0, 2.0, map_pose).result is FixResult.ACCEPTED


def test_a_multipath_jump_is_rejected():
    run = drive(conditions=PROFILES['open'], legs=[(100, 0.0)])
    filt, map_pose = run.filt, run.map_pose
    before = filt.transform
    true_pose = compose(before, map_pose)
    update = filt.gps(1000.0, true_pose.x + 30.0, true_pose.y, 2.5, map_pose)
    assert update.result is FixResult.REJECTED
    assert update.distance > 3.7
    assert filt.transform == before


def test_persistent_disagreement_is_reacquired():
    # After 120 m every fix is 20 m further east (e.g. SLAM relocalised): the
    # filter rejects them for reacquire_time seconds, then follows them.
    def shift(travelled):
        return (20.0, 0.0) if travelled > 120.0 else (0.0, 0.0)

    run = drive(conditions=PROFILES['open'], gps_offset=shift, skip=0.0,
                legs=[(300, 0.0)])
    assert run.results[FixResult.REJECTED] >= 25
    assert run.results[FixResult.REACQUIRED] == 1
    # By the end the estimate follows the shifted fixes (true error ~20 m east).
    assert run.errors[-1] == pytest.approx(20.0, abs=3.0)


def test_uncertainty_grows_without_gps_and_recovers():
    run = drive(conditions=PROFILES['open'], no_fix=(200.0, 400.0), seed=1)
    poses = true_path()
    i_before, i_tunnel_end = int(195 / 0.2), int(398 / 0.2)
    sigma_before = run.sigmas[i_before - int(50 / 0.2)]
    sigma_end_of_tunnel = run.sigmas[i_tunnel_end - int(50 / 0.2)]
    assert sigma_end_of_tunnel > 2.0 * sigma_before
    assert run.sigmas[-1] < 1.5 * sigma_before
    assert len(poses) > i_tunnel_end
    assert run.honest() >= 0.98


def test_standing_still_cannot_average_the_bias_away():
    filt = AlignmentFilter()
    gps_model = GpsErrorModel(seed=3)
    map_pose = Pose2D()
    filt.heading(0.0, map_pose)
    for i in range(3000):   # 10 minutes at 5 Hz
        r = gps_model.read(0.0, 0.0, i * 0.2, PROFILES['urban_canyon'])
        if r.available:
            filt.gps(i * 0.2, r.east, r.north, r.reported_sigma, map_pose)
    # 600 fused fixes; treated as independent they would give 1.5 / sqrt(600) = 0.06 m.
    # The 3 m bias changes only about once a minute: about 10 independent samples.
    sigma = filt.position_sigma(map_pose)
    assert sigma > 0.8
    estimate = filt.robot_pose(map_pose)
    assert math.hypot(estimate.x, estimate.y) < 3.0 * sigma


def test_a_biased_compass_cannot_drag_the_heading():
    run = drive(conditions=PROFILES['open'], imu_bias_deg=10.0, slam='perfect', seed=2)
    error = wrap_angle(run.filt.transform.yaw - run.truth_map_in_enu.yaw)
    assert abs(math.degrees(error)) < 2.0


def test_compass_is_used_only_while_the_heading_is_uncertain():
    config = AlignmentConfig(imu_yaw_sigma=math.radians(2.0))
    filt = AlignmentFilter(config)
    poses = true_path(legs=[(20, 0.0)])
    for i, true_pose in enumerate(poses):
        filt.gps(i * 0.2, true_pose.x, true_pose.y, 0.5, relative(START, true_pose))
    assert filt.initialised
    map_pose = relative(START, poses[-1])
    before = filt.yaw_sigma()
    assert before > math.radians(2.0)
    assert filt.heading(START.yaw, map_pose) is True
    assert filt.yaw_sigma() < before
    assert filt.yaw_sigma() <= math.radians(2.0)
    assert filt.heading(START.yaw, map_pose) is False


@pytest.mark.parametrize('seed', [0, 1, 2])
def test_urban_canyon_drive_stays_honest(seed):
    run = drive(seed=seed)
    assert run.honest() >= 0.98
    assert run.percentile(0.95) < 8.0
    assert run.results.get(FixResult.ACCEPTED, 0) > 400


def test_urban_canyon_tunnel_stays_honest():
    run = drive(seed=1, no_fix=(250.0, 450.0))
    assert run.honest() >= 0.98
    assert run.percentile(0.5) < 5.0


def test_distance_and_time_since_fix_are_tracked():
    filt = AlignmentFilter()
    filt.heading(0.0, Pose2D())
    filt.gps(10.0, 0.0, 0.0, 2.0, Pose2D())
    filt.predict(Pose2D(3.0, 4.0, 0.0), 12.0)
    assert filt.distance_since_fix == pytest.approx(5.0)
    assert filt.seconds_since_fix(15.0) == pytest.approx(5.0)
    filt.gps(15.0, 3.0, 4.0, 2.0, Pose2D(3.0, 4.0, 0.0))
    assert filt.distance_since_fix == 0.0


def test_non_finite_fix_raises():
    with pytest.raises(ValueError):
        AlignmentFilter().gps(0.0, math.nan, 0.0, 2.0, Pose2D())


@pytest.mark.parametrize('kwargs', [{'gps_gate': 0.0}, {'position_drift': -1.0}])
def test_invalid_config_raises(kwargs):
    with pytest.raises(ValueError):
        AlignmentConfig(**kwargs)


def test_largest_eigenvalue_2x2():
    assert largest_eigenvalue_2x2([[4.0, 0.0], [0.0, 1.0]]) == pytest.approx(4.0)
    assert largest_eigenvalue_2x2([[2.0, 1.0], [1.0, 2.0]]) == pytest.approx(3.0)


def test_config_is_frozen():
    config = AlignmentConfig()
    assert replace(config, gps_gate=9.0).gps_gate == 9.0
    with pytest.raises(AttributeError):
        config.gps_gate = 1.0
