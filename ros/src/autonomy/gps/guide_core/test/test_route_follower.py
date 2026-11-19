"""Tests for guide_core.route_follower."""

import math

from guide_core.route import Route
from guide_core.route_follower import FollowerConfig, RouteFollower
import pytest

# L route: 40 m east, then 30 m north. Total 70 m, one corner at s = 40.
L_ROUTE = Route([[0.0, 0.0], [40.0, 0.0], [40.0, 30.0]])


def follower(route=L_ROUTE):
    """Return a follower with the default config (lookahead 25, release 1.5, update 5)."""
    return RouteFollower(route)


# --- Lookahead and goal updates ----------------------------------------------

def test_first_update_sends_goal_lookahead_ahead():
    out = follower().update(0.0, 0.0)
    g = out.new_goal
    assert (g.x, g.y, g.s) == pytest.approx((25.0, 0.0, 25.0))
    assert g.yaw == pytest.approx(0.0)
    assert not g.is_corner and not g.is_final


def test_small_move_sends_no_new_goal():
    f = follower()
    f.update(0.0, 0.0)
    assert f.update(1.0, 0.0).new_goal is None
    assert f.goal.s == pytest.approx(25.0)  # the old goal still stands


def test_move_of_update_distance_sends_new_goal():
    f = follower()
    f.update(0.0, 0.0)
    g = f.update(5.0, 0.0).new_goal
    assert (g.x, g.y) == pytest.approx((30.0, 0.0))


# --- Corners -----------------------------------------------------------------

def test_goal_held_at_corner():
    # s = 16, lookahead would reach s = 41: held at the corner (s = 40).
    g = follower().update(16.0, 0.0).new_goal
    assert (g.x, g.y) == pytest.approx((40.0, 0.0))
    assert g.is_corner
    assert g.yaw == pytest.approx(math.pi / 2)  # faces the outgoing leg (north)


def test_corner_not_released_before_release_distance():
    f = follower()
    f.update(16.0, 0.0)
    assert f.update(38.4, 0.0).new_goal is None  # 1.6 m short: still held
    assert f.goal.is_corner


def test_corner_released_within_release_distance():
    f = follower()
    f.update(16.0, 0.0)
    g = f.update(38.6, 0.0).new_goal  # 1.4 m short: released
    assert (g.x, g.y) == pytest.approx((40.0, 23.6))  # 38.6 + 25 = 63.6
    assert not g.is_corner


def test_corner_release_is_latched():
    f = follower()
    f.update(16.0, 0.0)
    f.update(38.6, 0.0)
    # Robot drops back to 3 m before the corner: it must not be re-held.
    assert f.update(37.0, 0.0).new_goal is None
    assert not f.goal.is_corner


# --- Final goal and arrival --------------------------------------------------

def test_final_goal_sent_even_for_small_change():
    f = follower()
    f.update(40.0, 4.0)                   # s = 44, goal at s = 69
    g = f.update(40.0, 5.5).new_goal      # target reaches the end: only 1 m change
    assert g.is_final
    assert (g.x, g.y) == pytest.approx((40.0, 30.0))


def test_short_route_first_goal_is_final():
    g = follower(Route([[0, 0], [10, 0]])).update(0.0, 0.0).new_goal
    assert g.is_final
    assert g.s == pytest.approx(10.0)


def test_arrival():
    out = follower().update(40.0, 29.5)
    assert out.arrived
    assert out.new_goal is None


def test_no_arrival_at_start_of_loop_route():
    # The route ends where it starts; at the start, 80 m remain.
    loop = Route([[0, 0], [20, 0], [20, 20], [0, 20], [0, 0]])
    out = follower(loop).update(0.0, 0.0)
    assert not out.arrived
    assert out.distance_remaining == pytest.approx(80.0)


def test_no_arrival_when_beside_end():
    # Level with the end (0 m remaining) but 3 m to the side.
    assert not follower().update(43.0, 30.0).arrived


def test_arrival_is_latched():
    f = follower()
    f.update(40.0, 29.5)
    out = f.update(40.0, 20.0)  # moves away from the end
    assert out.arrived
    assert out.new_goal is None


# --- Progress and the search window ------------------------------------------

def test_progress_fields():
    out = follower().update(10.0, 2.0)
    assert out.s == pytest.approx(10.0)
    assert out.segment_index == 0
    assert out.cross_track_error == pytest.approx(2.0)
    assert out.distance_remaining == pytest.approx(60.0)
    assert out.total_length == pytest.approx(70.0)
    assert not out.arrived


def test_window_keeps_progress_on_outward_leg():
    # Out-and-back route: drifting towards the return leg must not jump s to 94.
    f = follower(Route([[0, 0], [50, 0], [50, 4], [0, 4]]))
    f.update(10.0, 0.0)
    out = f.update(10.0, 2.5)
    assert out.segment_index == 0
    assert out.s == pytest.approx(10.0)


# --- Validation --------------------------------------------------------------

@pytest.mark.parametrize('x, y', [(math.nan, 0.0), (0.0, math.inf)])
def test_non_finite_pose_rejected(x, y):
    with pytest.raises(ValueError):
        follower().update(x, y)


@pytest.mark.parametrize('kwargs', [
    {'lookahead': 0.0},
    {'goal_update_distance': -1.0},
    {'arrival_tolerance': math.nan},
    {'corner_angle': -0.1},
    {'corner_angle': 4.0},
])
def test_invalid_config_rejected(kwargs):
    with pytest.raises(ValueError):
        FollowerConfig(**kwargs)


# --- Whole route -------------------------------------------------------------

def test_full_pass_along_l_route():
    # Drive the robot exactly along the route in 0.5 m steps.
    f = follower()
    goals = []
    arrived_at = None
    for i in range(141):                      # s = 0.0, 0.5, ..., 70.0
        s = i * 0.5
        out = f.update(*L_ROUTE.point_at(s))
        if out.new_goal is not None:
            assert arrived_at is None, 'no goals after arrival'
            goals.append(out.new_goal)
        if out.arrived and arrived_at is None:
            arrived_at = s

    # 25 m ahead, every 5 m, held at the corner until s = 38.5,
    # then 63.5, 68.5 and the final point.
    assert [g.s for g in goals] == pytest.approx([25, 30, 35, 40, 63.5, 68.5, 70])
    assert [g.is_corner for g in goals] == [False, False, False, True, False, False, False]
    assert [g.is_final for g in goals] == [False] * 6 + [True]
    assert goals[3].yaw == pytest.approx(math.pi / 2)
    assert arrived_at == pytest.approx(69.0)  # 1.0 m tolerance


# --- Corner safety net (confirm_goal_reached) --------------------------------

def parked_at_corner():
    """Robot parked 0.3 m short of the corner (as the sim does), release distance 0.1 m.

    0.3 m is outside the 0.1 m release distance, so the follower alone never
    releases the corner: the deadlock the safety net exists for.
    """
    f = RouteFollower(L_ROUTE, FollowerConfig(corner_release_distance=0.1))
    f.update(16.0, 0.0)   # goal held at the corner (40, 0)
    f.update(39.7, 0.0)   # parked 0.3 m short
    return f


def test_confirm_before_any_goal_returns_false():
    assert not follower().confirm_goal_reached(25.0, 0.0)


def test_confirm_lookahead_goal_returns_false():
    f = follower()
    f.update(0.0, 0.0)                                # goal at s = 25
    assert not f.confirm_goal_reached(25.0, 0.0)
    assert f.update(0.0, 0.0).new_goal is None        # nothing changed
    assert f.goal.s == pytest.approx(25.0)


def test_corner_deadlock_without_confirm():
    f = parked_at_corner()
    for _ in range(10):
        assert f.update(39.7, 0.0).new_goal is None   # stuck: goal stays at the corner
    assert f.goal.is_corner


def test_confirm_corner_releases_and_next_goal_passes_corner():
    f = parked_at_corner()
    assert f.confirm_goal_reached(40.0, 0.0)
    g = f.update(39.7, 0.0).new_goal                  # 39.7 + 25 = 64.7
    assert (g.x, g.y, g.s) == pytest.approx((40.0, 24.7, 64.7))
    assert not g.is_corner


def test_confirm_wrong_position_returns_false():
    f = parked_at_corner()
    assert not f.confirm_goal_reached(30.0, 0.0)      # stale success for an older goal
    assert f.update(39.7, 0.0).new_goal is None       # corner still held
    assert f.goal.is_corner


def test_confirm_same_corner_twice_second_returns_false():
    f = parked_at_corner()
    assert f.confirm_goal_reached(40.0, 0.0)
    assert not f.confirm_goal_reached(40.0, 0.0)


def test_confirm_final_goal_returns_false():
    f = follower()
    f.update(40.0, 5.5)                               # s = 45.5: goal is the final point
    assert f.goal.is_final
    assert not f.confirm_goal_reached(40.0, 30.0)
    assert not f.update(40.0, 5.5).arrived            # arrival is still update()'s decision


def test_leaving_corner_goal_always_sent():
    # Lookahead 3 m < goal_update_distance 5 m: on release the goal moves only
    # 1.6 m, but it must still be sent or the robot parks at the corner.
    f = RouteFollower(L_ROUTE, FollowerConfig(lookahead=3.0))
    f.update(37.5, 0.0)                               # 37.5 + 3 = 40.5: held at the corner
    assert f.goal.is_corner
    g = f.update(38.6, 0.0).new_goal                  # released; 38.6 + 3 = 41.6
    assert g is not None
    assert (g.x, g.y, g.s) == pytest.approx((40.0, 1.6, 41.6))
