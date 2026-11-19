"""Tests for crossing holds in guide_core.route_follower.

The tests without crossings live in test_route_follower.py and are unchanged.
"""

import math

from guide_core.crossings import CrossingError, RouteCrossing
from guide_core.route import InvalidRouteError, Route
from guide_core.route_follower import FollowerConfig, RouteFollower
from guide_core.walk_costs import CrossingType
import pytest

# Straight route, 100 m east. One crossing from s = 50 to 58: stop at 48.5.
STRAIGHT = Route([[0.0, 0.0], [100.0, 0.0]])
CROSSING = RouteCrossing(50.0, 58.0, CrossingType.SIGNALLED, 'major', True)

# Pavement 50 m east, then a 90 degree turn north at the kerb (s = 50) and
# the crossing straight ahead from s = 50 to 58. Stop at 48.5, on the pavement.
KERB_TURN = Route([[0.0, 0.0], [50.0, 0.0], [50.0, 40.0]])

# L route of test_route_follower.py: corner at s = 40, total 70 m.
L_ROUTE = Route([[0.0, 0.0], [40.0, 0.0], [40.0, 30.0]])


def _crossing(start, end):
    """Return a signalled major-road crossing from start to end (m along the route)."""
    return RouteCrossing(start, end, CrossingType.SIGNALLED, 'major', True)


def straight_follower(**config):
    """Follower on STRAIGHT with CROSSING (default config unless changed)."""
    return RouteFollower(STRAIGHT, FollowerConfig(**config), [CROSSING])


def at_stop_parked_short(distance_short):
    """Follower whose robot has stopped distance_short metres before the stop (48.5)."""
    f = straight_follower()
    f.update(30.0, 0.0)                       # goal held at the stop
    f.update(48.5 - distance_short, 0.0)
    return f


def full_pass(f, route):
    """Drive exactly along the route in 0.5 m steps; return (goals, waiting flags)."""
    goals, waiting = [], []
    for i in range(int(route.total_length / 0.5) + 1):
        out = f.update(*route.point_at(i * 0.5))
        if out.new_goal is not None:
            goals.append(out.new_goal)
        waiting.append(out.waiting_at_crossing)
    return goals, waiting


# --- No crossings: unchanged behaviour ------------------------------------------

def test_no_crossings_full_pass_unchanged():
    goals, waiting = full_pass(RouteFollower(L_ROUTE, crossings=()), L_ROUTE)
    assert [g.s for g in goals] == pytest.approx([25, 30, 35, 40, 63.5, 68.5, 70])
    assert not any(g.is_crossing_stop for g in goals)
    assert not any(waiting)


# --- Where the goal goes ----------------------------------------------------------

def test_goal_short_of_stop_is_lookahead():
    g = straight_follower().update(10.0, 0.0).new_goal   # 10 + 25 = 35 < 48.5
    assert g.s == pytest.approx(35.0)
    assert not g.is_crossing_stop


def test_goal_held_at_crossing_stop():
    g = straight_follower().update(30.0, 0.0).new_goal   # 30 + 25 = 55 > 48.5
    assert (g.x, g.y, g.s) == pytest.approx((48.5, 0.0, 48.5))
    assert g.is_crossing_stop
    assert not g.is_corner and not g.is_final


def test_stop_goal_faces_across_road():
    # The route turns north at the kerb; the robot stops on the pavement
    # facing north (across the road), and the kerb corner does not take over.
    f = RouteFollower(KERB_TURN, crossings=[CROSSING])
    g = f.update(30.0, 0.0).new_goal
    assert (g.x, g.y) == pytest.approx((48.5, 0.0))
    assert g.yaw == pytest.approx(math.pi / 2)
    assert g.is_crossing_stop and not g.is_corner


def test_corner_before_stop_is_held_first():
    # Corner at s = 20 (turn north), crossing 40-48 on the north leg: stop 38.5.
    route = Route([[0.0, 0.0], [20.0, 0.0], [20.0, 60.0]])
    f = RouteFollower(route, crossings=[_crossing(40.0, 48.0)])
    g = f.update(0.0, 0.0).new_goal
    assert g.is_corner and g.s == pytest.approx(20.0)
    g = f.update(18.6, 0.0).new_goal                     # corner released
    assert g.is_crossing_stop and g.s == pytest.approx(38.5)


# --- The hold is never released by the robot itself -------------------------------

def test_hold_not_released_by_reaching_stop():
    f = straight_follower()
    f.update(30.0, 0.0)
    for x in (47.0, 48.2, 48.5, 49.5):
        assert f.update(x, 0.0).new_goal is None
    assert f.goal.is_crossing_stop
    assert f.goal.s == pytest.approx(48.5)


def test_position_jump_past_crossing_keeps_hold():
    # A GPS jump puts the robot beyond the far kerb: the crossing is not skipped.
    f = at_stop_parked_short(0.3)
    out = f.update(60.0, 0.0)
    assert out.s == pytest.approx(60.0)
    assert out.new_goal is None
    assert f.goal.is_crossing_stop


# --- Waiting -------------------------------------------------------------------------

def test_not_waiting_far_from_stop():
    assert not straight_follower().update(30.0, 0.0).waiting_at_crossing


@pytest.mark.parametrize('x, expected', [
    (46.9, False),   # 1.6 m short: outside the 1.5 m wait distance
    (47.1, True),    # 1.4 m short
    (48.2, True),    # 0.3 m short, where the sim stops
    (50.0, True),    # past the stop point
])
def test_waiting_by_distance(x, expected):
    f = straight_follower()
    f.update(30.0, 0.0)
    assert f.update(x, 0.0).waiting_at_crossing is expected


def test_nav2_success_at_stop_means_waiting_not_release():
    f = at_stop_parked_short(2.5)                       # GPS says 2.5 m short
    assert not f.update(46.0, 0.0).waiting_at_crossing
    assert not f.confirm_goal_reached(48.5, 0.0)        # nothing released ...
    out = f.update(46.0, 0.0)
    assert out.waiting_at_crossing                       # ... but now waiting
    assert out.new_goal is None
    assert f.goal.is_crossing_stop


def test_success_for_other_position_ignored():
    f = at_stop_parked_short(2.5)
    assert not f.confirm_goal_reached(30.0, 0.0)
    assert not f.update(46.0, 0.0).waiting_at_crossing


# --- Crossings and config given to the follower --------------------------------------

def test_island_stop_points():
    # Two halves 1.8 m apart: second stop in the middle of the island.
    f = RouteFollower(STRAIGHT, crossings=[_crossing(50.0, 58.0), _crossing(59.8, 67.0)])
    assert f.stop_points == pytest.approx((48.5, 58.9))


def test_overlapping_crossings_merged():
    f = RouteFollower(STRAIGHT, crossings=[_crossing(50.0, 56.0), _crossing(55.0, 60.0)])
    assert len(f.crossings) == 1
    assert (f.crossings[0].start_s, f.crossings[0].end_s) == (50.0, 60.0)


def test_crossing_outside_route_rejected():
    with pytest.raises(CrossingError):
        RouteFollower(STRAIGHT, crossings=[_crossing(95.0, 120.0)])
    assert issubclass(CrossingError, InvalidRouteError)


@pytest.mark.parametrize('kwargs', [
    {'crossing_stop_margin': -1.0},
    {'crossing_stop_margin': math.nan},
    {'crossing_wait_distance': 0.0},
    {'crossing_wait_distance': math.inf},
])
def test_invalid_crossing_config_rejected(kwargs):
    with pytest.raises(ValueError):
        FollowerConfig(**kwargs)


def test_zero_stop_margin_allowed():
    f = straight_follower(crossing_stop_margin=0.0)
    assert f.stop_points == (50.0,)


def test_crossing_at_route_start():
    # Crossing starts 1 m into the route: the stop is clamped to s = 0, where
    # the robot already is, so it waits straight away.
    f = RouteFollower(STRAIGHT, crossings=[_crossing(1.0, 5.0)])
    out = f.update(0.0, 0.0)
    assert out.new_goal.is_crossing_stop and out.new_goal.s == pytest.approx(0.0)
    assert out.waiting_at_crossing


# --- Driving up to a crossing ----------------------------------------------------------

def test_drive_until_crossing_stop():
    # The robot moves 0.5 m per update but, like the sim, stops 0.3 m short of
    # its goal. It must end up waiting just before the stop, with no arrival.
    f = straight_follower()
    s = 0.0
    goals, first_waiting = [], None
    for _ in range(400):
        out = f.update(s, 0.0)
        if out.new_goal is not None:
            goals.append(out.new_goal)
        if out.waiting_at_crossing and first_waiting is None:
            first_waiting = s
        s = min(s + 0.5, f.goal.s - 0.3)
    assert [g.s for g in goals] == pytest.approx([25, 30, 35, 40, 45, 48.5])
    assert [g.is_crossing_stop for g in goals] == [False] * 5 + [True]
    assert first_waiting == pytest.approx(47.0)
    assert s == pytest.approx(48.2)
    assert not out.arrived


# --- The user's go (confirm_crossing) and the far kerb ---------------------------------

def waiting_at_stop(f=None):
    """Follower on STRAIGHT (by default) whose robot waits 0.3 m short of the stop."""
    f = f or straight_follower()
    f.update(30.0, 0.0)
    f.update(48.2, 0.0)
    return f


def test_confirm_before_any_update_rejected():
    assert not straight_follower().confirm_crossing()


def test_early_confirm_rejected_and_not_remembered():
    f = straight_follower()
    f.update(30.0, 0.0)
    assert not f.confirm_crossing()                  # 18.5 m before the stop
    out = f.update(48.2, 0.0)
    assert out.waiting_at_crossing                   # still has to wait ...
    assert out.new_goal is None and f.goal.is_crossing_stop   # ... and the hold stands


def test_confirm_while_waiting_sends_far_kerb_goal():
    f = waiting_at_stop()
    assert f.waiting_at_crossing
    assert f.confirm_crossing()
    g = f.update(48.2, 0.0).new_goal
    assert (g.x, g.y, g.s) == pytest.approx((58.0, 0.0, 58.0))
    assert g.is_crossing_exit
    assert not g.is_crossing_stop and not g.is_corner


def test_second_confirm_rejected():
    f = waiting_at_stop()
    assert f.confirm_crossing()
    assert not f.confirm_crossing()                  # no crossing left to wait at


def test_lookahead_resumes_after_far_kerb():
    f = waiting_at_stop()
    f.confirm_crossing()
    f.update(48.2, 0.0)
    g = f.update(56.6, 0.0).new_goal                 # within 1.5 m of the far kerb
    assert g.s == pytest.approx(81.6)
    assert not g.is_crossing_exit


def test_far_kerb_released_by_nav2_success():
    f = waiting_at_stop(straight_follower(corner_release_distance=0.1))
    f.confirm_crossing()
    f.update(48.2, 0.0)
    assert f.update(57.7, 0.0).new_goal is None      # parked 0.3 m short: held
    assert f.confirm_goal_reached(58.0, 0.0)
    assert f.update(57.7, 0.0).new_goal.s == pytest.approx(82.7)


def test_kerb_corner_held_after_confirm():
    # The route turns north at the kerb (s = 50). After the go, the robot
    # first goes to the kerb corner, and only then across to the far kerb.
    f = RouteFollower(KERB_TURN, crossings=[CROSSING])
    f.update(30.0, 0.0)
    f.update(48.2, 0.0)                              # waiting, 1.8 m from the corner
    assert f.confirm_crossing()
    g = f.update(48.2, 0.0).new_goal
    assert g.is_corner and (g.x, g.y) == pytest.approx((50.0, 0.0))
    g = f.update(48.6, 0.0).new_goal                 # within 1.5 m: corner released
    assert g.is_crossing_exit and (g.x, g.y) == pytest.approx((50.0, 8.0))


def test_waiting_at_next_crossing_after_release():
    f = RouteFollower(STRAIGHT, crossings=[_crossing(30.0, 36.0), _crossing(70.0, 76.0)])
    f.update(28.2, 0.0)
    assert f.confirm_crossing()
    assert not f.update(40.0, 0.0).waiting_at_crossing   # Nav2/success state not carried over
    out = f.update(68.2, 0.0)
    assert out.waiting_at_crossing and out.next_crossing_index == 1


def test_nav2_success_flag_cleared_on_release():
    f = RouteFollower(STRAIGHT, crossings=[_crossing(30.0, 36.0), _crossing(70.0, 76.0)])
    f.update(26.0, 0.0)                              # goal at the first stop (28.5)
    f.confirm_goal_reached(28.5, 0.0)                # Nav2: reached
    assert f.confirm_crossing()
    assert not f.update(50.0, 0.0).waiting_at_crossing


def test_not_waiting_at_island_stop_while_on_first_half():
    # Island stop at 58.9; 57.5 is within 1.5 m of it but still on the road.
    f = RouteFollower(STRAIGHT, crossings=[_crossing(50.0, 58.0), _crossing(59.8, 67.0)])
    waiting_at_stop(f)
    assert f.confirm_crossing()
    assert not f.update(57.5, 0.0).waiting_at_crossing
    assert not f.confirm_crossing()                  # a second press is rejected
    assert f.update(58.2, 0.0).waiting_at_crossing  # on the island now


def test_progress_crossing_fields():
    f = straight_follower()
    out = f.update(30.0, 0.0)
    assert (out.next_crossing_index, out.distance_to_next_crossing) == (0, pytest.approx(20.0))
    assert not out.on_crossing
    f.update(48.2, 0.0)
    f.confirm_crossing()
    out = f.update(54.0, 0.0)
    assert out.on_crossing
    assert (out.next_crossing_index, out.distance_to_next_crossing) == (-1, 0.0)
    assert not f.update(60.0, 0.0).on_crossing


def test_progress_crossing_fields_without_crossings():
    out = RouteFollower(L_ROUTE).update(10.0, 0.0)
    assert (out.waiting_at_crossing, out.on_crossing) == (False, False)
    assert (out.next_crossing_index, out.distance_to_next_crossing) == (-1, 0.0)


def test_crossing_at_route_end():
    f = RouteFollower(STRAIGHT, crossings=[_crossing(95.0, 100.0)])
    f.update(80.0, 0.0)
    f.update(93.2, 0.0)
    assert f.confirm_crossing()
    g = f.update(93.2, 0.0).new_goal
    assert g.is_crossing_exit and g.is_final
    assert f.update(99.5, 0.0).arrived


def test_waiting_property_matches_last_update():
    f = straight_follower()
    assert not f.waiting_at_crossing                 # no update yet
    f.update(30.0, 0.0)
    assert not f.waiting_at_crossing
    f.update(48.2, 0.0)
    assert f.waiting_at_crossing


def test_drive_island_crossing_with_two_confirms():
    # Like the sim: 0.5 m per update, stopping 0.3 m short of each goal. The
    # user confirms whenever the robot has stopped while waiting.
    f = RouteFollower(STRAIGHT, crossings=[_crossing(50.0, 58.0), _crossing(59.8, 67.0)])
    s, previous, confirmed_at, goals = 0.0, None, [], []
    for _ in range(1000):
        out = f.update(s, 0.0)
        if out.new_goal is not None:
            goals.append(out.new_goal)
        if out.arrived:
            break
        if out.waiting_at_crossing and s == previous:
            assert f.confirm_crossing()
            confirmed_at.append(s)
        previous = s
        s = min(s + 0.5, f.goal.s - 0.3) if not f.goal.is_final else min(s + 0.5, 100.0)
    assert out.arrived
    assert confirmed_at == pytest.approx([48.2, 58.6])
    holds = [(g.s, g.is_crossing_stop, g.is_crossing_exit) for g in goals
             if g.is_crossing_stop or g.is_crossing_exit]
    assert holds == [(pytest.approx(48.5), True, False), (pytest.approx(58.0), False, True),
                     (pytest.approx(58.9), True, False), (pytest.approx(67.0), False, True)]


def test_touching_crossings_stop_at_meeting_point():
    # Two halves meeting at an island node (s = 58): the second stop is that
    # node, and the robot halting 0.3 m short of it counts as waiting.
    f = RouteFollower(STRAIGHT, crossings=[_crossing(50.0, 58.0), _crossing(58.0, 66.0)])
    assert f.stop_points == pytest.approx((48.5, 58.0))
    waiting_at_stop(f)
    assert f.confirm_crossing()
    g = f.update(48.2, 0.0).new_goal
    assert g.is_crossing_stop and g.s == pytest.approx(58.0)
    assert not f.update(57.0, 0.0).waiting_at_crossing   # still on the first half
    assert f.update(57.7, 0.0).waiting_at_crossing
