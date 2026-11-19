"""Tests for guide_core.mission with online lookup (destinations not in the curated list).

The geocoder's answers are given as Destinations with source 'nominatim',
as the mission manager node will build them from guide/geocode responses.
"""

from guide_core.destinations import Destination
from guide_core.geocoder import Status
from guide_core.mission import (
    build_summary, CallPlanner, Finish, LookUpOnline, Mission, MissionConfig, MissionError,
    MissionGoal, MissionState, PlanSummary, SendRoute, StopRobot)
import pytest

STATION = Destination('South Kensington Station', 51.4941, -0.1738,
                      aliases=('south ken',), note='end of the pedestrian subway, step-free')
ROBOT = (51.4986289, -0.1752868)
HALL = Destination('Royal Albert Hall', 51.501007, -0.1774058, note='large area',
                   source='nominatim')
GALLERY = Destination('Small Gallery', 51.4990, -0.1760, source='nominatim')
MUSEUMS = (Destination('Natural History Museum', 51.4965109, -0.1760019, note='large area',
                       source='nominatim'),
           Destination('Science Museum', 51.4973983, -0.1746726, note='large area',
                       source='nominatim'))
HALL_PLAN = PlanSummary(length=352.0, goal_snap_distance=38.0)
HALL_GOAL = MissionGoal(destination='royal albert hall')


ONLINE = MissionConfig(online_lookup=True)   # as the node configures it


def looking_up(goal=HALL_GOAL):
    """Return a mission waiting for the geocoder, and its LookUpOnline command."""
    m = Mission((STATION,), ONLINE)
    (lookup,) = m.start(goal, ROBOT)
    return m, lookup


def found(m, lookup, destinations=(HALL,), from_cache=False):
    """Answer the lookup with OK and return the commands."""
    return m.on_lookup_result(lookup.lookup_id, Status.OK, destinations, from_cache)


def finish_of(commands):
    (finish,) = [c for c in commands if isinstance(c, Finish)]
    return finish


def test_a_name_not_in_the_list_is_looked_up_online():
    m, lookup = looking_up()
    assert lookup == LookUpOnline(1, 'royal albert hall')
    assert m.state is MissionState.PLANNING
    assert m.snapshot().message == "Searching OpenStreetMap for 'royal albert hall'."


def test_a_curated_name_is_never_looked_up():
    m = Mission((STATION,), ONLINE)
    (call,) = m.start(MissionGoal(destination='south ken'), ROBOT)
    assert isinstance(call, CallPlanner)


def test_without_online_lookup_an_unknown_name_fails_at_once():
    m = Mission((STATION,))   # online_lookup is off by default (as before Step 7b)
    finish = finish_of(m.start(HALL_GOAL, ROBOT))
    assert finish.error_code is MissionError.UNKNOWN_DESTINATION
    assert finish.message == "I don't know a place called 'royal albert hall'."


def test_one_result_is_planned_from_where_the_robot_was():
    m, lookup = looking_up()
    (call,) = found(m, lookup)
    assert call == CallPlanner(1, ROBOT[0], ROBOT[1], HALL.lat, HALL.lon)
    snapshot = m.snapshot()
    assert snapshot.state is MissionState.PLANNING
    assert snapshot.destination_name == 'Royal Albert Hall'
    assert snapshot.destination_note == ''        # 'large area' is said in the read-back instead
    assert snapshot.message == 'Planning a route to Royal Albert Hall.'


def test_read_back_says_where_it_was_found_and_that_it_is_a_large_area():
    m, lookup = looking_up()
    (call,) = found(m, lookup)
    assert m.on_plan_result(call.plan_id, True, 'ok', HALL_PLAN) == ()
    assert m.state is MissionState.READY
    assert m.snapshot().summary == (
        'Found in OpenStreetMap: Royal Albert Hall. About 350 m walk, no road crossings. '
        'Royal Albert Hall is a large area: the route goes to its nearest path, not to an '
        'entrance. The route ends 38 m from Royal Albert Hall. Press go to start.')


def test_a_cached_answer_is_read_back_as_saved_results():
    m, lookup = looking_up()
    (call,) = found(m, lookup, from_cache=True)
    m.on_plan_result(call.plan_id, True, 'ok', HALL_PLAN)
    assert m.snapshot().summary.startswith(
        'Found in saved OpenStreetMap results: Royal Albert Hall.')


def test_a_small_place_has_no_large_area_sentence():
    m, lookup = looking_up(MissionGoal(destination='small gallery'))
    (call,) = found(m, lookup, (GALLERY,))
    m.on_plan_result(call.plan_id, True, 'ok', PlanSummary(length=60.0, goal_snap_distance=2.0))
    assert m.snapshot().summary == ('Found in OpenStreetMap: Small Gallery. About 60 m walk, '
                                    'no road crossings. Press go to start.')


def test_skip_confirmation_does_not_apply_to_online_results():
    m, lookup = looking_up(MissionGoal(destination='royal albert hall', skip_confirmation=True))
    (call,) = found(m, lookup)
    assert m.on_plan_result(call.plan_id, True, 'ok', HALL_PLAN) == ()   # no SendRoute
    assert m.state is MissionState.READY
    assert m.snapshot().summary.endswith('Press go to start.')
    reply = m.on_go(now=0.0)
    assert reply.accepted and isinstance(reply.commands[0], SendRoute)


def test_skip_confirmation_still_applies_to_curated_destinations():
    m = Mission((STATION,), ONLINE)
    (call,) = m.start(MissionGoal(destination='south ken', skip_confirmation=True), ROBOT)
    (send,) = m.on_plan_result(call.plan_id, True, 'ok', HALL_PLAN)
    assert isinstance(send, SendRoute)


def test_several_results_ask_which_one():
    m, lookup = looking_up(MissionGoal(destination='museum'))
    finish = finish_of(found(m, lookup, MUSEUMS))
    assert finish.error_code is MissionError.AMBIGUOUS_DESTINATION
    assert finish.message.startswith("I found 2 places in OpenStreetMap for 'museum'. "
                                     'Option 1: Natural History Museum, large area, about ')
    assert finish.message.endswith('Which one?')
    assert m.state is MissionState.FAILED and m.can_accept()


@pytest.mark.parametrize('status, ending', [
    (Status.NO_RESULTS, 'OpenStreetMap has nothing by that name near here.'),
    (Status.DISABLED, 'Online search is switched off.'),
    (Status.OFFLINE, "I couldn't search online just now. Try another name."),
])
def test_nothing_usable_fails_as_unknown_destination(status, ending):
    m, lookup = looking_up()
    finish = finish_of(m.on_lookup_result(lookup.lookup_id, status, ()))
    assert finish.error_code is MissionError.UNKNOWN_DESTINATION
    assert finish.message.endswith(ending)
    assert m.snapshot().message == finish.message


def test_cancel_during_the_lookup_stops_and_ignores_the_late_answer():
    m, lookup = looking_up()
    commands = m.cancel()
    assert not any(isinstance(c, StopRobot) for c in commands)   # the robot never moved
    assert finish_of(commands).error_code is MissionError.CANCELED
    assert found(m, lookup) == () and m.state is MissionState.IDLE


def test_an_answer_to_an_older_lookup_is_ignored():
    m, first = looking_up()
    m.cancel()
    (second,) = m.start(MissionGoal(destination='museum'), ROBOT)
    assert second.lookup_id == first.lookup_id + 1
    assert found(m, first) == ()                     # the old lookup's answer
    assert m.state is MissionState.PLANNING


def test_a_late_planner_answer_cannot_end_a_lookup():
    m = Mission((STATION,), ONLINE)
    (old_call,) = m.start(MissionGoal(destination='south ken'), ROBOT)
    m.cancel()
    m.start(HALL_GOAL, ROBOT)                         # now waiting for the geocoder
    assert m.on_plan_result(old_call.plan_id, True, 'ok', HALL_PLAN) == ()
    assert m.state is MissionState.PLANNING


def test_go_while_looking_up_asks_to_wait():
    m, _ = looking_up()
    reply = m.on_go(now=0.0)
    assert not reply.accepted and reply.text == 'Still planning, please wait.'


def test_build_summary_with_found_in_and_large_area():
    plan = PlanSummary(length=120.0, goal_snap_distance=0.0)
    assert build_summary('X', '', plan, 5.0, found_in='OpenStreetMap', large_area=True) == (
        'Found in OpenStreetMap: X. About 120 m walk, no road crossings. '
        'X is a large area: the route goes to its nearest path, not to an entrance. '
        'Press go to start.')
