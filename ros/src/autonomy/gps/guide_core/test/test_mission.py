"""Tests for guide_core.mission (the mission manager's state machine)."""

import math

from guide_core.destinations import Destination
from guide_core.mission import (
    build_summary, CallPlanner, ClearRoute, ConfirmCrossing, CrossingInfo, describe_crossing,
    Finish, Mission, MissionConfig, MissionError, MissionGoal, MissionState, NavStatus, Pause,
    PlanSummary, Progress, Resume, SendRoute, StopRobot,
)
import pytest

STATION = Destination('South Kensington Station', 51.4941, -0.1738,
                      aliases=('south ken',), note='end of the pedestrian subway, step-free')
DESTINATIONS = (STATION,)
ROBOT = (51.4986289, -0.1752868)
TUNNEL = 'route uses 193 m of tunnel (no GPS there)'
DEMO_PLAN = PlanSummary(
    length=649.265, goal_snap_distance=4.7,
    crossings=(CrossingInfo('unknown', 'minor'), CrossingInfo('unknown', 'minor'),
               CrossingInfo('unknown', 'none'), CrossingInfo('signalled', 'major', True),
               CrossingInfo('signalled', 'major', True), CrossingInfo('signalled', 'minor')),
    warnings=(TUNNEL,))
GOAL = MissionGoal(destination='south ken')


def planned_mission(goal=GOAL):
    """Return a mission in READY: started and planned (the demo plan)."""
    m = Mission(DESTINATIONS)
    (call,) = m.start(goal, ROBOT)
    m.on_plan_result(call.plan_id, True, 'ok', DEMO_PLAN)
    return m


def active_mission():
    """Return a mission in NAVIGATING: the user pressed go, the follower took route 1."""
    m = planned_mission()
    reply = m.on_go(now=0.0)
    (send,) = reply.commands
    m.on_route_set(send.route_id, True, 'ok', sent_stamp=10.0)
    return m


def progress(m, remaining=300.0, **kwargs):
    return Progress(route_id=m.route_id, distance_remaining=remaining, **kwargs)


def finish_of(commands):
    (finish,) = [c for c in commands if isinstance(c, Finish)]
    return finish


# --- read-back text ---

def test_summary_of_demo_route():
    assert build_summary(STATION.name, STATION.note, DEMO_PLAN, 5.0) == (
        'South Kensington Station, end of the pedestrian subway, step-free. '
        'About 650 m walk, 6 road crossings, 2 with an island. '
        'Warning: route uses 193 m of tunnel (no GPS there). Press go to start.')


def test_summary_reports_goal_far_from_requested_point():
    plan = PlanSummary(length=120.0, goal_snap_distance=12.4)
    text = build_summary('Central Library', '', plan, 5.0)
    assert 'The route ends 12 m from Central Library.' in text


def test_summary_without_crossings_when_setting_off_directly():
    plan = PlanSummary(length=120.0, goal_snap_distance=0.0)
    assert build_summary('X', '', plan, 5.0, skip_confirmation=True) == (
        'X. About 120 m walk, no road crossings. Setting off.')


def test_summary_single_crossing():
    plan = PlanSummary(length=100.0, goal_snap_distance=0.0,
                       crossings=(CrossingInfo('signalled', 'minor'),))
    assert 'About 100 m walk, 1 road crossing.' in build_summary('X', '', plan, 5.0)


@pytest.mark.parametrize('info, text', [
    (CrossingInfo('signalled', 'major', True), 'signalled, major road, with an island'),
    (CrossingInfo('unknown', 'none'), 'unknown, not a mapped road'),
])
def test_describe_crossing(info, text):
    assert describe_crossing(info) == text


# --- starting a goal ---

def test_new_mission_is_idle():
    m = Mission(DESTINATIONS)
    assert m.state is MissionState.IDLE
    assert m.can_accept()
    assert m.snapshot().route_id == 0


def test_start_by_name_calls_planner():
    m = Mission(DESTINATIONS)
    assert m.start(GOAL, ROBOT) == (
        CallPlanner(1, ROBOT[0], ROBOT[1], STATION.lat, STATION.lon),)
    assert m.state is MissionState.PLANNING
    snap = m.snapshot()
    assert snap.destination_name == 'South Kensington Station'
    assert snap.destination_note == STATION.note
    assert snap.message == 'Planning a route to South Kensington Station.'


def test_start_by_latlon_calls_planner():
    m = Mission(DESTINATIONS)
    commands = m.start(MissionGoal(latitude=51.49, longitude=-0.17), ROBOT)
    assert commands == (CallPlanner(1, ROBOT[0], ROBOT[1], 51.49, -0.17),)
    assert m.snapshot().destination_name == 'the requested point'


@pytest.mark.parametrize('goal', [
    MissionGoal(destination=''),
    MissionGoal(destination='   '),
    MissionGoal(latitude=91.0, longitude=0.0),
    MissionGoal(latitude=51.0, longitude=None),
    MissionGoal(latitude=math.nan, longitude=0.0),
])
def test_invalid_goal_fails(goal):
    m = Mission(DESTINATIONS)
    finish = finish_of(m.start(goal, ROBOT))
    assert finish.error_code is MissionError.INVALID_GOAL
    assert m.state is MissionState.FAILED
    assert m.can_accept()


def test_no_position_fails():
    m = Mission(DESTINATIONS)
    finish = finish_of(m.start(GOAL, None))
    assert finish.error_code is MissionError.NO_POSITION


def test_unknown_destination_fails_with_spoken_message():
    m = Mission(DESTINATIONS)
    finish = finish_of(m.start(MissionGoal(destination='swimming pool'), ROBOT))
    assert finish.error_code is MissionError.UNKNOWN_DESTINATION
    assert finish.message == "I don't know a place called 'swimming pool'."
    assert m.snapshot().message == finish.message


def test_ambiguous_destination_fails_and_lists_options():
    a = Destination('North Library', 51.0, 0.0, aliases=('library',))
    b = Destination('South Library', 51.001, 0.0, aliases=('library',))
    m = Mission((a, b))
    finish = finish_of(m.start(MissionGoal(destination='library'), (51.0, 0.0)))
    assert finish.error_code is MissionError.AMBIGUOUS_DESTINATION
    assert 'Option 1: North Library' in finish.message


def test_busy_while_mission_runs():
    m = Mission(DESTINATIONS)
    m.start(GOAL, ROBOT)                     # PLANNING
    assert not m.can_accept()
    with pytest.raises(RuntimeError):
        m.start(GOAL, ROBOT)
    m.on_plan_result(1, True, 'ok', DEMO_PLAN)   # READY
    assert not m.can_accept()
    (send,) = m.on_go(0.0).commands
    m.on_route_set(send.route_id, True, 'ok')    # NAVIGATING
    assert not m.can_accept()


# --- planning and READY ---

def test_plan_success_waits_in_ready_with_summary():
    m = planned_mission()
    assert m.state is MissionState.READY
    snap = m.snapshot()
    assert snap.summary.endswith('Press go to start.')
    assert snap.message == snap.summary
    assert snap.route_length == 649.265
    assert snap.num_crossings == 6
    assert snap.warnings == (TUNNEL,)


def test_plan_failure_fails():
    m = Mission(DESTINATIONS)
    m.start(GOAL, ROBOT)
    finish = finish_of(m.on_plan_result(1, False, 'Goal is 45.0 m from the nearest walkway'))
    assert finish.error_code is MissionError.PLANNING_FAILED
    assert finish.message == "I couldn't plan a route: Goal is 45.0 m from the nearest walkway"
    assert m.state is MissionState.FAILED


def test_answer_to_older_plan_request_is_ignored():
    m = Mission(DESTINATIONS)
    m.start(GOAL, ROBOT)
    assert m.on_plan_result(99, True, 'ok', DEMO_PLAN) == ()
    assert m.state is MissionState.PLANNING


def test_skip_confirmation_sends_route_at_once():
    m = Mission(DESTINATIONS)
    m.start(MissionGoal(destination='south ken', skip_confirmation=True), ROBOT)
    assert m.on_plan_result(1, True, 'ok', DEMO_PLAN) == (SendRoute(1),)
    assert m.snapshot().summary.endswith('Setting off.')
    m.on_route_set(1, True, 'ok')
    assert m.state is MissionState.NAVIGATING
    assert m.route_id == 1


def test_go_in_ready_sends_route_once():
    m = planned_mission()
    reply = m.on_go(now=0.0)
    assert reply.accepted and reply.commands == (SendRoute(1),)
    assert m.state is MissionState.READY          # until the follower accepts
    again = m.on_go(now=5.0)
    assert not again.accepted and again.commands == ()
    assert again.text == 'Starting, please wait.'


def test_route_rejected_by_follower_fails_without_stopping_robot():
    m = planned_mission()
    m.on_go(0.0)
    commands = m.on_route_set(1, False, 'crossing 2 overlaps')
    assert commands == (Finish(MissionError.PLANNING_FAILED,
                               'The route could not be loaded: crossing 2 overlaps', 0.0),)
    assert m.state is MissionState.FAILED


# --- following the route ---

def test_navigating_snapshot():
    m = active_mission()
    snap = m.snapshot()
    assert snap.state is MissionState.NAVIGATING
    assert snap.route_id == 1
    assert snap.message == 'Setting off.'


def test_progress_for_another_route_is_ignored():
    m = active_mission()
    m.on_progress(Progress(route_id=7, distance_remaining=10.0, waiting_at_crossing=True))
    assert m.state is MissionState.NAVIGATING


def test_waiting_at_crossing_is_copied_from_follower():
    m = active_mission()
    m.on_progress(progress(m, waiting_at_crossing=True, next_crossing_index=3))
    assert m.state is MissionState.WAITING_AT_CROSSING
    assert m.snapshot().message == (
        'At crossing 4 of 6 (signalled, major road, with an island). '
        'Press go when it is safe to cross.')


def test_go_at_crossing_confirms_it():
    m = active_mission()
    m.on_progress(progress(m, waiting_at_crossing=True, next_crossing_index=0))
    reply = m.on_go(now=10.0)
    assert reply.accepted and reply.commands == (ConfirmCrossing(),)
    m.on_progress(progress(m, on_crossing=True, next_crossing_index=1))
    assert m.state is MissionState.NAVIGATING
    assert m.snapshot().message == 'Crossing now.'


def test_go_while_moving_is_ignored():
    m = active_mission()
    reply = m.on_go(now=10.0)
    assert not reply.accepted and reply.commands == ()
    assert reply.text == 'Already moving.'


def test_pause_and_resume():
    m = active_mission()
    reply = m.on_pause()
    assert reply.accepted and reply.commands == (Pause(),)
    assert not m.on_pause().accepted               # pause pending: no second Pause
    m.on_paused(True)
    assert m.state is MissionState.PAUSED
    assert m.snapshot().message == 'Paused.'
    reply = m.on_go(now=10.0)
    assert reply.accepted and reply.commands == (Resume(),)
    m.on_paused(False)
    assert m.state is MissionState.NAVIGATING
    assert m.snapshot().message == 'Continuing.'


def test_pause_on_crossing_warns():
    m = active_mission()
    m.on_progress(progress(m, on_crossing=True, next_crossing_index=1))
    reply = m.on_pause()
    assert reply.accepted and reply.commands == (Pause(),)   # always obeyed
    assert reply.text == 'Paused in the road. Press go to continue crossing.'
    m.on_paused(True)
    assert m.snapshot().message == reply.text


def test_pause_at_crossing_then_go_only_resumes():
    # Safety: a paused robot waiting at a crossing needs one go to resume and
    # a SECOND go to cross; one press must never do both.
    m = active_mission()
    m.on_progress(progress(m, waiting_at_crossing=True, next_crossing_index=3))
    m.on_pause()
    m.on_paused(True)
    assert m.on_go(now=10.0).commands == (Resume(),)
    m.on_paused(False)
    assert m.state is MissionState.WAITING_AT_CROSSING
    assert m.snapshot().message.startswith('At crossing 4 of 6')
    assert m.on_go(now=12.0).commands == (ConfirmCrossing(),)


def test_quick_second_go_is_ignored():
    m = active_mission()
    m.on_progress(progress(m, waiting_at_crossing=True, next_crossing_index=0))
    m.on_pause()
    m.on_paused(True)
    assert m.on_go(now=10.0).commands == (Resume(),)
    early = m.on_go(now=10.2)                      # paused topic not updated yet
    assert not early.accepted and early.commands == ()
    m.on_paused(False)                             # still waiting at the crossing
    assert m.on_go(now=10.5).commands == ()        # within min_go_interval
    assert m.on_go(now=11.5).commands == (ConfirmCrossing(),)


def test_resume_pending_blocks_second_go():
    m = Mission(DESTINATIONS, MissionConfig(min_go_interval=0.0))
    (call,) = m.start(GOAL, ROBOT)
    m.on_plan_result(call.plan_id, True, 'ok', DEMO_PLAN)
    m.on_route_set(m.on_go(0.0).commands[0].route_id, True, 'ok')
    m.on_pause()
    m.on_paused(True)
    m.on_go(now=1.0)
    reply = m.on_go(now=1.0)
    assert reply.text == 'Resuming, please wait.' and reply.commands == ()


def test_pause_from_elsewhere_is_shown():
    m = active_mission()
    m.on_paused(True)                              # e.g. guide/pause called from the CLI
    assert m.state is MissionState.PAUSED
    assert m.snapshot().message == 'Paused. Press go to continue.'


def test_pause_message_changes_once_when_confirmed():
    # The user hears "Paused." once, when the goal sender confirms it.
    m = active_mission()
    v = m.version
    assert m.on_pause().text == 'Paused.'
    assert m.version == v and m.snapshot().message == 'Setting off.'
    m.on_paused(True)
    assert m.version == v + 1
    assert m.snapshot().state is MissionState.PAUSED
    assert m.snapshot().message == 'Paused.'


def test_arrival_finishes_mission():
    m = active_mission()
    commands = m.on_progress(progress(m, remaining=0.4, arrived=True))
    assert commands == (ClearRoute(), Finish(
        MissionError.NONE, 'You have arrived at South Kensington Station.', 0.0))
    assert m.state is MissionState.ARRIVED
    assert m.can_accept()
    assert m.route_id == 0


def test_next_mission_uses_new_ids_and_ignores_old_route():
    m = active_mission()
    m.on_progress(progress(m, arrived=True))
    (call,) = m.start(GOAL, ROBOT)
    assert call.plan_id == 2
    m.on_plan_result(2, True, 'ok', DEMO_PLAN)
    (send,) = m.on_go(now=100.0).commands
    assert send == SendRoute(2)
    m.on_route_set(2, True, 'ok')
    m.on_progress(Progress(route_id=1, distance_remaining=0.0, arrived=True))
    assert m.state is MissionState.NAVIGATING


def test_failed_confirm_is_reported():
    m = active_mission()
    m.on_confirm_result(False, 'Not at a crossing')
    assert m.snapshot().message == 'Could not start the crossing: Not at a crossing'


def test_failed_pause_call_can_be_retried():
    m = active_mission()
    (pause,) = m.on_pause().commands
    m.on_service_failed(pause, 'guide/pause not available')
    assert m.snapshot().message == 'Something went wrong: guide/pause not available'
    assert m.on_pause().commands == (Pause(),)


# --- Nav2 failures ---

@pytest.mark.parametrize('status', [NavStatus.ABORTED, NavStatus.REJECTED])
def test_nav_failure_pauses_and_waits_for_go(status):
    m = active_mission()
    assert m.on_nav_status(status, stamp=11.0) == (Pause(),)
    m.on_paused(True)
    assert m.state is MissionState.PAUSED
    assert m.snapshot().message == (
        'The robot could not continue (1 of 3). Press go to try again.')
    assert m.on_go(now=12.0).commands == (Resume(),)


def test_other_statuses_are_ignored():
    m = active_mission()
    assert m.on_nav_status(NavStatus.CANCELED, stamp=11.0) == ()
    assert m.on_nav_status(NavStatus.SUCCEEDED, stamp=12.0) == ()
    assert m.state is MissionState.NAVIGATING


def test_repeated_status_counts_once():
    m = active_mission()
    m.on_nav_status(NavStatus.ABORTED, stamp=11.0)
    assert m.on_nav_status(NavStatus.ABORTED, stamp=11.0) == ()
    assert '(1 of 3)' in m.snapshot().message


def test_status_from_before_route_was_sent_is_ignored():
    m = active_mission()                           # route sent at stamp 10.0
    assert m.on_nav_status(NavStatus.ABORTED, stamp=9.5) == ()
    assert m.state is MissionState.NAVIGATING


def test_failures_in_a_row_fail_the_mission():
    m = active_mission()
    m.on_progress(progress(m, remaining=200.0))
    m.on_nav_status(NavStatus.ABORTED, stamp=11.0)
    m.on_nav_status(NavStatus.ABORTED, stamp=12.0)
    commands = m.on_nav_status(NavStatus.ABORTED, stamp=13.0)
    assert commands == (StopRobot(), ClearRoute(), Finish(
        MissionError.NAVIGATION_FAILED,
        'Navigation failed 3 times in a row. The robot has stopped.', 200.0))
    assert m.state is MissionState.FAILED


def test_moving_on_resets_failure_count():
    m = active_mission()
    m.on_progress(progress(m, remaining=200.0))
    m.on_nav_status(NavStatus.ABORTED, stamp=11.0)
    m.on_nav_status(NavStatus.ABORTED, stamp=12.0)
    m.on_progress(progress(m, remaining=194.0))    # 6 m further on
    m.on_nav_status(NavStatus.ABORTED, stamp=13.0)
    assert '(1 of 3)' in m.snapshot().message
    assert m.state is not MissionState.FAILED


def test_failure_while_paused_sends_no_second_pause():
    m = active_mission()
    m.on_pause()
    m.on_paused(True)
    assert m.on_nav_status(NavStatus.ABORTED, stamp=11.0) == ()


# --- cancel ---

def test_cancel_while_planning():
    m = Mission(DESTINATIONS)
    m.start(GOAL, ROBOT)
    assert m.cancel() == (Finish(MissionError.CANCELED,
                                 'Cancelled. The robot has stopped.', 0.0),)
    assert m.state is MissionState.IDLE
    assert m.on_plan_result(1, True, 'ok', DEMO_PLAN) == ()   # late answer dropped
    assert m.state is MissionState.IDLE


def test_cancel_while_navigating_stops_robot():
    m = active_mission()
    m.on_progress(progress(m, remaining=321.0))
    assert m.cancel() == (StopRobot(), ClearRoute(), Finish(
        MissionError.CANCELED, 'Cancelled. The robot has stopped.', 321.0))
    assert m.state is MissionState.IDLE
    assert m.route_id == 0


def test_cancel_while_route_is_being_sent():
    m = planned_mission()
    m.on_go(now=0.0)                               # SendRoute(1) pending
    commands = m.cancel()
    assert commands[:2] == (StopRobot(), ClearRoute())
    assert m.on_route_set(1, True, 'ok') == ()     # late acceptance dropped
    assert m.state is MissionState.IDLE


def test_cancel_when_idle_does_nothing():
    assert Mission(DESTINATIONS).cancel() == ()


# --- misc ---

def test_version_changes_with_snapshot():
    m = active_mission()
    v = m.version
    m.on_progress(progress(m, waiting_at_crossing=True, next_crossing_index=0))
    assert m.version > v
    v = m.version
    m.on_progress(progress(m, remaining=299.0, waiting_at_crossing=True,
                           next_crossing_index=0))
    assert m.version == v                          # nothing visible changed


@pytest.mark.parametrize('kwargs', [
    {'max_nav_failures': 0},
    {'min_go_interval': -1.0},
])
def test_config_rejects_bad_values(kwargs):
    with pytest.raises(ValueError):
        MissionConfig(**kwargs)


def test_pick_nearest_goes_to_the_nearest_of_several_and_says_so():
    near = Destination('North Library', 51.0, 0.0, aliases=('library',))
    far = Destination('South Library', 51.01, 0.0, aliases=('library',))
    m = Mission((far, near), MissionConfig(pick_nearest=True))
    commands = m.start(MissionGoal(destination='library'), (51.0001, 0.0))
    assert not any(isinstance(c, Finish) for c in commands)   # not refused as ambiguous
    m.on_plan_result(commands[0].plan_id, True, '', DEMO_PLAN)
    assert 'North Library, the nearest of 2 places with that name.' in m.snapshot().message
