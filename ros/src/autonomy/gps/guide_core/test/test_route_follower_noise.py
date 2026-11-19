"""Tests for the route follower under position uncertainty (Step 8).

Stop points shifted by sigma, sigma-aware arrival, and off-route detection.
Expected stop points are read from RouteFollower.stop_points, so these tests
do not depend on how guide_core.crossings places them.
"""

import math

from guide_core.crossings import RouteCrossing
from guide_core.route import Route
from guide_core.route_follower import FollowerConfig, RouteFollower
from guide_core.walk_costs import CrossingType
import pytest

# 200 m straight east with two crossings: 60..70 m and 120..130 m.
STRAIGHT = Route([[0.0, 0.0], [200.0, 0.0]])


def crossing(start, end):
    """Return an unknown minor-road crossing from start to end (m along the route)."""
    return RouteCrossing(start_s=start, end_s=end, crossing_type=CrossingType.UNKNOWN,
                         crossed_road='minor', tactile_paving=False, has_island=False)


# The tests below do their arithmetic with a 2-sigma shift (2 * (sigma - 0.5));
# the default (3 sigma) has its own test.
TWO_SIGMA = FollowerConfig(stop_sigma_factor=2.0)


def follower(config=None, crossings=(crossing(60.0, 70.0), crossing(120.0, 130.0))):
    """Return a follower on the straight route (2-sigma shift unless config is given)."""
    return RouteFollower(STRAIGHT, config or TWO_SIGMA, crossings)


# --- Stop point moved earlier for uncertainty -----------------------------------

def test_default_shift_is_three_sigma():
    f = follower(FollowerConfig())
    f.update(10.0, 0.0, sigma=2.5)             # 3 * (2.5 - 0.5) = 6.0 m
    assert f.stop_offset == pytest.approx(6.0)


def test_small_sigma_does_not_move_the_stop():
    f = follower()
    f.update(10.0, 0.0, sigma=0.5)
    assert f.stop_offset == 0.0
    assert f.active_stop_s == pytest.approx(f.stop_points[0])


def test_stop_moves_two_sigma_earlier():
    f = follower()
    out = f.update(30.0, 0.0, sigma=2.5)       # 2 * (2.5 - 0.5) = 4.0 m
    assert f.stop_offset == pytest.approx(4.0)
    assert out.new_goal.is_crossing_stop
    assert out.new_goal.s == pytest.approx(f.stop_points[0] - 4.0)


def test_shift_is_rounded_up_to_half_metres():
    f = follower()
    f.update(10.0, 0.0, sigma=1.6)             # 2 * 1.1 = 2.2 -> 2.5 m
    assert f.stop_offset == pytest.approx(2.5)


def test_far_from_the_stop_the_shift_follows_a_clear_drop_in_sigma():
    f = follower()
    f.update(10.0, 0.0, sigma=3.0)
    f.update(11.0, 0.0, sigma=1.0)
    assert f.stop_offset == pytest.approx(1.0)


def test_sigma_wobbling_at_a_rounding_boundary_does_not_resend_the_stop():
    # sigma around 3.0 m: 2 * (sigma - 0.5) is just under or just over 5.0 m.
    f = follower()
    goals = []
    for i in range(40):
        sigma = 3.0 + (0.01 if i % 2 else -0.01)
        out = f.update(20.0 + 0.4 * i, 0.0, sigma=sigma)
        if out.new_goal is not None and out.new_goal.is_crossing_stop:
            goals.append(out.new_goal.s)
    assert len(goals) == 1       # sent once, never re-sent back and forth
    assert f.stop_offset == pytest.approx(5.5)
    assert goals[0] == pytest.approx(f.stop_points[0] - 5.5)


def test_a_small_drop_in_sigma_keeps_the_shift():
    f = follower()
    f.update(10.0, 0.0, sigma=3.0)               # 5.0 m
    f.update(11.0, 0.0, sigma=2.6)               # wants 4.5 m: less than two steps lower
    assert f.stop_offset == pytest.approx(5.0)


def test_near_the_stop_the_shift_only_grows():
    f = follower()
    static = f.stop_points[0]
    f.update(static - 10.0, 0.0, sigma=2.5)
    f.update(static - 9.0, 0.0, sigma=0.5)
    assert f.stop_offset == pytest.approx(4.0)
    assert f.update(static - 8.5, 0.0, sigma=0.5).new_goal is None   # nothing re-sent


def test_robot_already_past_the_shifted_stop_stops_where_it_is():
    f = follower()
    static = f.stop_points[0]
    f.update(static - 1.5, 0.0, sigma=0.4)
    out = f.update(static - 1.0, 0.0, sigma=3.0)   # shift 5 m: the stop is now behind it
    assert out.waiting_at_crossing
    assert f.active_stop_s == pytest.approx(static - 1.0)
    assert out.new_goal.is_crossing_stop
    assert out.new_goal.s == pytest.approx(static - 1.0)


def test_shift_never_goes_before_the_previous_far_kerb():
    f = follower(crossings=(crossing(60.0, 70.0), crossing(74.0, 80.0)))
    f.update(58.0, 0.0)
    assert f.confirm_crossing() is True
    f.update(71.0, 0.0, sigma=4.0)              # shift 7 m would reach 65.5 m
    assert f.active_stop_s == pytest.approx(71.0)


def test_a_static_stop_before_the_previous_far_kerb_is_not_moved(monkeypatch):
    f = follower(crossings=(crossing(60.0, 70.0), crossing(74.0, 80.0)))
    # Pretend stop_points put the second stop at 69 m, inside the first crossing.
    monkeypatch.setattr(f, '_stop_s', (f.stop_points[0], 69.0))
    f.update(58.0, 0.0)
    assert f.confirm_crossing() is True
    f.update(66.0, 0.0)
    assert f.active_stop_s == pytest.approx(69.0)


def test_shift_resets_after_the_users_go():
    f = follower()
    static = f.stop_points[0]
    f.update(static - 4.0, 0.0, sigma=2.5)
    assert f.update(static - 4.2, 0.0, sigma=2.5).waiting_at_crossing
    assert f.confirm_crossing() is True
    assert f.stop_offset == 0.0
    f.update(72.0, 0.0, sigma=0.4)
    assert f.active_stop_s == pytest.approx(f.stop_points[1])


def test_only_the_user_releases_a_shifted_stop():
    f = follower()
    static = f.stop_points[0]
    for i in range(30):                         # robot creeps forward, sigma large
        out = f.update(static - 6.0 + 0.1 * i, 0.0, sigma=2.5)
        assert out.new_goal is None or out.new_goal.is_crossing_stop
    assert f.next_crossing_index == 0
    assert out.waiting_at_crossing


# --- Arrival --------------------------------------------------------------------

def test_arrival_tolerance_grows_with_sigma():
    route = Route([[0.0, 0.0], [50.0, 0.0]])
    assert not RouteFollower(route).update(48.5, 0.0).arrived
    assert not RouteFollower(route).update(48.5, 0.0, sigma=2.0).arrived   # 1.0 m
    assert RouteFollower(route).update(48.5, 0.0, sigma=4.0).arrived       # 2.0 m


# --- Off-route detection --------------------------------------------------------

def run_offset(f, offsets, start_x=20.0, sigma=1.0):
    """Feed one pose per second at x = start_x + t, y = offset; return off_route flags."""
    flags = []
    for t, y in enumerate(offsets):
        flags.append(f.update(start_x + t * 0.2, y, sigma=sigma, now=float(t)).off_route)
    return flags


def test_off_route_needs_time_beyond_the_limit():
    flags = run_offset(follower(), [12.0] * 7)
    # Beyond 10 m from t=0: off-route from t=5 (off_route_time), not before.
    assert flags == [False] * 5 + [True] * 2


def test_off_route_has_hysteresis():
    flags = run_offset(follower(), [12.0] * 6 + [7.0] * 3 + [4.0])
    assert flags == [False] * 5 + [True] + [True] * 3 + [False]


def test_a_brief_excursion_is_not_off_route():
    flags = run_offset(follower(), [12.0] * 4 + [8.0] + [12.0] * 4)
    assert not any(flags)


def test_off_route_is_not_judged_without_time():
    f = follower()
    assert not any(f.update(20.0, 15.0).off_route for _ in range(20))


def test_off_route_is_not_judged_with_large_sigma():
    f = follower()
    flags = [f.update(20.0, 15.0, sigma=6.0, now=float(t)).off_route for t in range(10)]
    assert not any(flags)


def test_off_route_state_is_kept_while_sigma_is_large():
    f = follower()
    run_offset(f, [12.0] * 6)
    assert f.off_route
    assert f.update(21.0, 2.0, sigma=6.0, now=10.0).off_route   # unsure: unchanged
    assert not f.update(21.0, 2.0, sigma=1.0, now=11.0).off_route


def test_off_route_is_not_judged_while_waiting_at_a_crossing():
    f = follower()
    static = f.stop_points[0]
    flags = [f.update(static, 12.0, sigma=1.0, now=float(t)).off_route for t in range(10)]
    assert f.waiting_at_crossing
    assert not any(flags)


def test_off_route_is_not_judged_on_a_crossing():
    f = follower()
    f.update(f.stop_points[0], 0.0)
    assert f.confirm_crossing() is True
    flags = [f.update(64.0, 12.0, sigma=1.0, now=float(t)).off_route for t in range(10)]
    assert not any(flags)


# --- Compatibility and validation -----------------------------------------------

def test_without_sigma_and_time_nothing_changes():
    a, b = follower(), follower()
    for i in range(0, 120):
        x = i * 0.5
        assert a.update(x, 0.3) == b.update(x, 0.3, sigma=0.0)


def test_negative_sigma_is_rejected():
    with pytest.raises(ValueError):
        follower().update(0.0, 0.0, sigma=-1.0)


@pytest.mark.parametrize('kwargs', [
    {'stop_sigma_step': 0.0},
    {'stop_sigma_factor': -1.0},
    {'off_route_time': math.nan},
    {'back_on_route_distance': 12.0},
])
def test_invalid_noise_config_rejected(kwargs):
    with pytest.raises(ValueError):
        FollowerConfig(**kwargs)
