"""Tests for the simulator's GPS error model."""

from dataclasses import replace
import math
import statistics

from guide_core.gps_error import conditions_from_dict, GpsConditions, GpsErrorModel, PROFILES
import pytest


def run(model, conditions, duration, dt=0.2, east=0.0, north=0.0):
    """Return the readings for a robot standing still for ``duration`` seconds."""
    steps = int(round(duration / dt))
    return [model.read(east, north, i * dt, conditions) for i in range(steps + 1)]


def errors(readings):
    """Return (east error, north error) of every available reading."""
    return [(r.east, r.north) for r in readings if r.available]


def test_perfect_profile_gives_exact_position():
    model = GpsErrorModel(seed=1)
    readings = run(model, PROFILES['perfect'], 60.0, east=12.5, north=-3.0)
    assert all(r.available for r in readings)
    assert all(r.east == 12.5 and r.north == -3.0 for r in readings)


def test_same_seed_gives_same_readings():
    a = run(GpsErrorModel(seed=7), PROFILES['urban_canyon'], 300.0)
    b = run(GpsErrorModel(seed=7), PROFILES['urban_canyon'], 300.0)
    assert a == b


def test_different_seeds_give_different_readings():
    a = run(GpsErrorModel(seed=7), PROFILES['urban_canyon'], 60.0)
    b = run(GpsErrorModel(seed=8), PROFILES['urban_canyon'], 60.0)
    assert a != b


def test_jitter_has_configured_sigma():
    conditions = GpsConditions(jitter_sigma=1.0)
    errs = errors(run(GpsErrorModel(seed=3), conditions, 2000.0))
    assert len(errs) > 9000
    assert statistics.pstdev(e for e, _ in errs) == pytest.approx(1.0, rel=0.05)
    assert statistics.pstdev(n for _, n in errs) == pytest.approx(1.0, rel=0.05)


def test_bias_has_configured_sigma_across_seeds():
    conditions = GpsConditions(bias_sigma=3.0, bias_time_constant=60.0)
    finals = []
    for seed in range(300):
        model = GpsErrorModel(seed=seed)
        run(model, conditions, 300.0, dt=1.0)
        finals.append(model.bias[0])
    assert statistics.pstdev(finals) == pytest.approx(3.0, rel=0.15)


def test_bias_changes_slowly():
    conditions = GpsConditions(bias_sigma=3.0, bias_time_constant=60.0)
    errs = errors(run(GpsErrorModel(seed=4), conditions, 600.0, dt=1.0))
    steps = [abs(b[0] - a[0]) for a, b in zip(errs, errs[1:])]
    # One-second change has sigma 3 * sqrt(1 - exp(-2/60)) = 0.55 m, far below 3 m.
    assert statistics.mean(steps) < 0.25 * 3.0


def test_no_fix_conditions_never_give_a_fix():
    readings = run(GpsErrorModel(seed=5), PROFILES['no_fix'], 120.0)
    assert not any(r.available for r in readings)
    assert not any(r.dropout or r.jumping for r in readings)


def test_dropouts_cover_expected_fraction_of_time():
    # Starts at 6/min (0.1/s), 2 s each: unavailable fraction = 0.2 / 1.2.
    conditions = GpsConditions(dropouts_per_minute=6.0, dropout_duration_min=2.0,
                               dropout_duration_max=2.0)
    readings = run(GpsErrorModel(seed=6), conditions, 20000.0)
    fraction = sum(r.dropout for r in readings) / len(readings)
    assert fraction == pytest.approx(0.2 / 1.2, abs=0.02)
    assert all(not r.available for r in readings if r.dropout)


def test_jumps_have_configured_size():
    conditions = GpsConditions(jumps_per_minute=10.0, jump_size_min=10.0,
                               jump_size_max=30.0)
    readings = run(GpsErrorModel(seed=8), conditions, 3000.0)
    jumped = [math.hypot(r.east, r.north) for r in readings if r.jumping]
    clean = [math.hypot(r.east, r.north) for r in readings if not r.jumping]
    assert len(jumped) > 1000
    assert all(10.0 - 1e-9 <= size <= 30.0 + 1e-9 for size in jumped)
    assert all(size == 0.0 for size in clean)


def test_reported_sigma_does_not_change_during_jumps():
    conditions = replace(PROFILES['urban_canyon'], jumps_per_minute=10.0)
    readings = run(GpsErrorModel(seed=9), conditions, 600.0)
    assert any(r.jumping for r in readings)
    assert {r.reported_sigma for r in readings} == {conditions.reported_sigma}


def test_jump_lasts_configured_duration():
    conditions = GpsConditions(jumps_per_minute=2.0, jump_duration_min=3.0,
                               jump_duration_max=3.0)
    readings = run(GpsErrorModel(seed=10), conditions, 3000.0, dt=0.1)
    runs, length = [], 0
    for r in readings:
        if r.jumping:
            length += 1
        elif length:
            runs.append(length)
            length = 0
    assert len(runs) > 20
    assert all(29 <= n <= 31 for n in runs)


def test_tunnel_ends_a_jump_in_progress():
    jumpy = GpsConditions(jumps_per_minute=600.0, jump_duration_min=100.0,
                          jump_duration_max=100.0)
    model = GpsErrorModel(seed=11)
    assert model.read(0.0, 0.0, 0.0, jumpy).jumping is False
    assert model.read(0.0, 0.0, 1.0, jumpy).jumping is True
    model.read(0.0, 0.0, 2.0, PROFILES['no_fix'])
    quiet = replace(jumpy, jumps_per_minute=0.0)
    assert model.read(0.0, 0.0, 3.0, quiet).jumping is False


def test_time_going_backwards_raises():
    model = GpsErrorModel()
    model.read(0.0, 0.0, 5.0, PROFILES['open'])
    with pytest.raises(ValueError):
        model.read(0.0, 0.0, 4.0, PROFILES['open'])


@pytest.mark.parametrize('kwargs', [
    {'jitter_sigma': -1.0},
    {'bias_time_constant': 0.0},
    {'reported_sigma': 0.0},
    {'jump_size_min': 40.0},
    {'dropout_duration_min': 9.0},
])
def test_invalid_conditions_raise(kwargs):
    with pytest.raises(ValueError):
        GpsConditions(**kwargs)


def test_named_profiles_exist():
    assert set(PROFILES) == {'perfect', 'open', 'urban_canyon', 'no_fix'}
    assert PROFILES['no_fix'].fix_available is False


def test_conditions_from_dict_starts_from_named_profile():
    c = conditions_from_dict({'profile': 'urban_canyon', 'jitter_sigma': 2.0})
    assert c.jitter_sigma == 2.0
    assert c.bias_sigma == PROFILES['urban_canyon'].bias_sigma


def test_conditions_from_dict_defaults_to_perfect():
    assert conditions_from_dict({}) == PROFILES['perfect']


@pytest.mark.parametrize('values', [
    {'profile': 'desert'},
    {'jiter_sigma': 1.0},
])
def test_conditions_from_dict_rejects_unknown_names(values):
    with pytest.raises(ValueError):
        conditions_from_dict(values)
