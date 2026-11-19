"""Tests for the mission under localisation problems and rerouting (Step 8)."""

import itertools

from guide_core.localization_quality import Quality
from guide_core.mission import (
    CallPlanner, ClearRoute, ConfirmCrossing, CrossingInfo, Finish, Mission, MissionConfig,
    MissionError, MissionGoal, MissionState, Pause, PlanSummary, Progress, Resume, SendRoute,
    StopRobot)
import pytest

ROBOT = (51.4986, -0.1753)
GOAL = MissionGoal(latitude=51.4941, longitude=-0.1738)
PLAN = PlanSummary(length=650.0, goal_snap_distance=0.0,
                   crossings=(CrossingInfo('unknown', 'minor'),))


# Times for the user's go presses: always far enough apart (min_go_interval).
GO_TIMES = itertools.count(100.0, 10.0)


def of_type(commands, kind):
    """Return the commands of one type."""
    return [c for c in commands if isinstance(c, kind)]


def navigating(config=None):
    """Return (mission, route_id) with a route being followed (state NAVIGATING)."""
    m = Mission(config=config)
    (plan,) = m.start(GOAL, ROBOT)
    m.on_plan_result(plan.plan_id, True, 'ok', PLAN)
    (send,) = m.on_go(0.0).commands
    m.on_route_set(send.route_id, True, 'ok')
    assert m.state is MissionState.NAVIGATING
    return m, send.route_id


def progress(route_id, remaining=600.0, **kwargs):
    """Return a Progress for route_id."""
    return Progress(route_id=route_id, distance_remaining=remaining, **kwargs)


# --- GPS_DEGRADED -------------------------------------------------------------

def test_degraded_while_navigating_is_reported_and_cleared():
    m, _ = navigating()
    assert m.on_localization(Quality.DEGRADED) == ()
    assert m.state is MissionState.GPS_DEGRADED
    assert m.snapshot().message == 'GPS signal is weak. Continuing carefully.'
    assert 'GPS signal weak' in m.snapshot().warnings
    m.on_localization(Quality.GOOD)
    assert m.state is MissionState.NAVIGATING
    assert m.snapshot().message == 'GPS signal is back.'
    assert 'GPS signal weak' not in m.snapshot().warnings


def test_waiting_at_a_crossing_wins_over_degraded():
    m, route = navigating()
    m.on_localization(Quality.DEGRADED)
    m.on_progress(progress(route, waiting_at_crossing=True, next_crossing_index=0))
    assert m.state is MissionState.WAITING_AT_CROSSING
    assert m.snapshot().message.startswith('At crossing 1 of 1')
    assert m.on_go(5.0).commands == (ConfirmCrossing(),)


def test_go_while_degraded_says_already_moving():
    m, _ = navigating()
    m.on_localization(Quality.DEGRADED)
    reply = m.on_go(5.0)
    assert not reply.accepted
    assert reply.text == 'Already moving.'


# --- POSITION_LOST ------------------------------------------------------------

def test_lost_pauses_and_waits_for_go_after_recovery():
    m, _ = navigating()
    assert m.on_localization(Quality.LOST) == (Pause(),)
    assert m.state is MissionState.POSITION_LOST
    assert m.snapshot().message == "I'm not sure where we are. Stopped, waiting for GPS."
    m.on_paused(True)                       # the goal sender confirms
    assert m.state is MissionState.POSITION_LOST
    reply = m.on_go(5.0)
    assert not reply.accepted
    m.on_localization(Quality.DEGRADED)     # position back (not yet good)
    assert m.state is MissionState.PAUSED   # still stopped: no surprise start
    assert m.snapshot().message == 'Position found again. Press go to continue.'
    assert m.on_go(10.0).commands == (Resume(),)
    m.on_paused(False)
    assert m.state is MissionState.GPS_DEGRADED


def test_lost_on_a_crossing_finishes_the_crossing_first():
    m, route = navigating()
    m.on_progress(progress(route, on_crossing=True, next_crossing_index=-1))
    assert m.on_localization(Quality.LOST) == ()          # keep going: in the road
    assert m.state is not MissionState.POSITION_LOST
    commands = m.on_progress(progress(route, remaining=590.0, on_crossing=False))
    assert of_type(commands, Pause) == [Pause()]           # off the road: now stop
    assert m.state is MissionState.POSITION_LOST


def test_lost_while_already_paused_sends_no_second_pause():
    m, _ = navigating()
    m.on_pause()
    m.on_paused(True)
    assert m.on_localization(Quality.LOST) == ()
    assert m.state is MissionState.POSITION_LOST
    m.on_localization(Quality.GOOD)
    assert m.state is MissionState.PAUSED


def test_no_new_trip_while_lost():
    m = Mission()
    m.on_localization(Quality.LOST)
    (finish,) = m.start(GOAL, ROBOT)
    assert isinstance(finish, Finish)
    assert finish.error_code is MissionError.NO_POSITION


def test_no_setting_off_while_lost():
    m = Mission()
    (plan,) = m.start(GOAL, ROBOT)
    m.on_plan_result(plan.plan_id, True, 'ok', PLAN)
    m.on_localization(Quality.LOST)
    reply = m.on_go(0.0)
    assert not reply.accepted
    assert reply.commands == ()


# --- Rerouting ----------------------------------------------------------------

def test_off_route_stops_replans_and_waits_for_go():
    m, route = navigating()
    here = (51.4960, -0.1745)
    commands = m.on_progress(progress(route, off_route=True), robot_latlon=here)
    assert of_type(commands, StopRobot) and of_type(commands, ClearRoute)
    (plan,) = of_type(commands, CallPlanner)
    assert (plan.start_lat, plan.start_lon) == here
    assert (plan.goal_lat, plan.goal_lon) == (GOAL.latitude, GOAL.longitude)
    assert m.state is MissionState.REROUTING
    assert m.snapshot().message == "We've left the route. Stopping to find a new one."
    assert m.on_go(1.0).text == 'Finding a new route, please wait.'

    new_plan = PlanSummary(length=420.0, goal_snap_distance=0.0)
    assert m.on_plan_result(plan.plan_id, True, 'ok', new_plan) == ()
    assert m.state is MissionState.READY
    message = m.snapshot().message
    assert message.startswith('New route: ')
    assert message.endswith('Press go to continue.')
    (send,) = m.on_go(2.0).commands
    assert isinstance(send, SendRoute) and send.route_id != route
    m.on_route_set(send.route_id, True, 'ok')
    assert m.state is MissionState.NAVIGATING
    assert m.route_id == send.route_id


def test_progress_of_the_old_route_is_ignored_after_a_reroute():
    m, route = navigating()
    m.on_progress(progress(route, off_route=True), robot_latlon=ROBOT)
    assert m.on_progress(progress(route, arrived=True)) == ()
    assert m.state is MissionState.REROUTING


@pytest.mark.parametrize('setup', ['degraded', 'paused', 'waiting', 'on_crossing'])
def test_off_route_is_ignored_when_it_may_be_our_error_or_the_robot_is_held(setup):
    m, route = navigating()
    flags = {}
    if setup == 'degraded':
        m.on_localization(Quality.DEGRADED)
    elif setup == 'paused':
        m.on_pause()
        m.on_paused(True)
    elif setup == 'waiting':
        flags = {'waiting_at_crossing': True, 'next_crossing_index': 0}
    else:
        flags = {'on_crossing': True}
    commands = m.on_progress(progress(route, off_route=True, **flags), robot_latlon=ROBOT)
    assert not of_type(commands, CallPlanner)
    assert m.state is not MissionState.REROUTING


def test_off_route_without_a_position_fails_safely():
    m, route = navigating()
    commands = m.on_progress(progress(route, off_route=True), robot_latlon=None)
    assert of_type(commands, StopRobot)
    (finish,) = of_type(commands, Finish)
    assert finish.error_code is MissionError.NAVIGATION_FAILED
    assert m.state is MissionState.FAILED


def reroute_once(m, route, remaining=600.0):
    """Leave the route, accept the new plan and set off again; return the new route id."""
    commands = m.on_progress(progress(route, remaining, off_route=True), robot_latlon=ROBOT)
    (plan,) = of_type(commands, CallPlanner)
    m.on_plan_result(plan.plan_id, True, 'ok', PLAN)
    (send,) = m.on_go(next(GO_TIMES)).commands
    m.on_route_set(send.route_id, True, 'ok')
    return send.route_id


def test_too_many_reroutes_fail():
    m, route = navigating(MissionConfig(max_reroutes=2))
    route = reroute_once(m, route)
    route = reroute_once(m, route)
    commands = m.on_progress(progress(route, off_route=True), robot_latlon=ROBOT)
    (finish,) = of_type(commands, Finish)
    assert finish.message == 'I keep losing the route. The robot has stopped.'
    assert m.state is MissionState.FAILED


def test_reroute_count_restarts_after_enough_progress():
    m, route = navigating(MissionConfig(max_reroutes=1, reroute_reset_distance=100.0))
    route = reroute_once(m, route)
    m.on_progress(progress(route, 600.0))
    m.on_progress(progress(route, 490.0))     # 110 m of progress on the new route
    route = reroute_once(m, route, remaining=490.0)
    assert m.state is MissionState.NAVIGATING


def test_cancel_while_rerouting():
    m, route = navigating()
    m.on_progress(progress(route, off_route=True), robot_latlon=ROBOT)
    commands = m.cancel()
    (finish,) = of_type(commands, Finish)
    assert finish.error_code is MissionError.CANCELED
    assert m.state is MissionState.IDLE


def test_failed_replan_fails_the_mission():
    m, route = navigating()
    commands = m.on_progress(progress(route, off_route=True), robot_latlon=ROBOT)
    (plan,) = of_type(commands, CallPlanner)
    (finish,) = m.on_plan_result(plan.plan_id, False, 'no path')
    assert finish.error_code is MissionError.PLANNING_FAILED
    assert m.state is MissionState.FAILED


# --- Compatibility and validation ----------------------------------------------

def test_without_localization_status_nothing_changes():
    m, route = navigating()
    m.on_progress(progress(route, 500.0))
    assert m.state is MissionState.NAVIGATING
    assert m.snapshot().warnings == ()


def test_new_states_keep_the_old_numbers():
    assert [int(s) for s in MissionState] == list(range(11))
    assert MissionState.GPS_DEGRADED == 8
    assert MissionState.POSITION_LOST == 9
    assert MissionState.REROUTING == 10


@pytest.mark.parametrize('kwargs', [{'max_reroutes': -1}, {'reroute_reset_distance': -1.0}])
def test_invalid_config_raises(kwargs):
    with pytest.raises(ValueError):
        MissionConfig(**kwargs)
