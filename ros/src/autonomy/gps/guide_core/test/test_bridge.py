"""Tests for guide_core.bridge (the gps_bridge node's logic)."""

import json

from guide_core.bridge import (
    Bridge, CancelMission, clean_target, CrossingWait, Done, Failed, failure_slug,
    failure_text, FAILURE_TEXTS, ModeRequest, parse_mode, REJECTED_TEXT, Request, Say, SendGo,
    StartMission, Status, status_line, UNAVAILABLE_TEXT,
)
from guide_core.mission import MissionError, MissionState
import pytest


def mode(mode_name, target='', stop=False):
    """Return a /middleman/mode message as the middleman writes it."""
    return json.dumps({'mode': mode_name, 'target': target, 'intent': 'goto', 'stop': stop})


GPS_STATION = mode('gps', 'south kensington station')
STOP = mode('stop', stop=True)
IDLE = mode('idle')


def started_bridge():
    """Return a bridge with one trip started (goal 1, the station)."""
    bridge = Bridge()
    bridge.on_mode(GPS_STATION)
    return bridge


def says(commands):
    return [c.text for c in commands if isinstance(c, Say)]


def statuses(commands):
    return [c.text for c in commands if isinstance(c, Status)]


# --- parse_mode ---

def test_parse_gps_with_target_starts():
    assert parse_mode(GPS_STATION) == ModeRequest(Request.START, 'south kensington station')


def test_parse_target_spaces_collapsed():
    assert parse_mode(mode('gps', '  royal   albert hall ')).target == 'royal albert hall'


def test_parse_stop_cancels():
    assert parse_mode(STOP) == ModeRequest(Request.CANCEL)


def test_parse_gps_with_stop_flag_cancels():
    assert parse_mode(mode('gps', 'x', stop=True)).request is Request.CANCEL


def test_parse_other_modes_cancel():
    for name in ('location', 'object', 'follow', 'manual', 'help', 'idle'):
        assert parse_mode(mode(name, 'kitchen')).request is Request.CANCEL


def test_parse_gps_without_target_cancels():
    assert parse_mode(mode('gps', '   ')).request is Request.CANCEL


def test_parse_missing_fields_cancels():
    assert parse_mode('{}').request is Request.CANCEL


def test_parse_not_json_or_not_object_is_none():
    for text in ('', 'gps', '[1, 2]', '"gps"', None):
        assert parse_mode(text) is None


def test_parse_custom_mode_name():
    assert parse_mode(mode('outdoor', 'x'), gps_mode='outdoor').request is Request.START
    assert parse_mode(mode('gps', 'x'), gps_mode='outdoor').request is Request.CANCEL


def test_clean_target_none_and_numbers():
    assert clean_target(None) == ''
    assert clean_target(12) == '12'


# --- status_line, failure_text, failure_slug ---

def test_status_line_words():
    target = 'station'
    assert status_line(MissionState.PLANNING, target) == 'planning:station'
    assert status_line(MissionState.READY, target) == 'ready:station'
    assert status_line(MissionState.NAVIGATING, target) == 'going:station'
    assert status_line(MissionState.PAUSED, target) == 'paused:station'
    assert status_line(MissionState.GPS_DEGRADED, target) == 'gps_weak:station'
    assert status_line(MissionState.POSITION_LOST, target) == 'position_lost:station'
    assert status_line(MissionState.REROUTING, target) == 'rerouting:station'


def test_status_line_crossing_has_no_target():
    assert status_line(MissionState.WAITING_AT_CROSSING, 'station') == 'waiting:crossing'


def test_status_line_end_states_empty():
    for state in (MissionState.IDLE, MissionState.ARRIVED, MissionState.FAILED):
        assert status_line(state, 'station') == ''


def test_status_line_plain_int_and_unknown_state():
    assert status_line(3, 'station') == 'going:station'
    assert status_line(42, 'station') == 'state_42:station'


def test_failure_text_prefers_message():
    assert failure_text(MissionError.PLANNING_FAILED, ' No path. ') == 'No path.'


def test_failure_text_fallbacks():
    assert failure_text(MissionError.UNKNOWN_DESTINATION, '') == "I couldn't find that place."
    assert failure_text(99, '') == 'GPS navigation failed.'


def test_failure_slug():
    assert failure_slug(MissionError.UNKNOWN_DESTINATION) == 'unknown_destination'
    assert failure_slug(2) == 'unknown_destination'
    assert failure_slug(99) == 'error_99'


# --- starting and cancelling ---

def test_gps_mode_starts_goal():
    bridge = Bridge()
    assert bridge.on_mode(GPS_STATION) == (StartMission(1, 'south kensington station'),)
    assert bridge.active and bridge.goal_id == 1


def test_bad_json_does_nothing():
    bridge = started_bridge()
    assert bridge.on_mode('not json') == ()
    assert bridge.active


def test_stop_cancels_active_trip():
    bridge = started_bridge()
    assert bridge.on_mode(STOP) == (
        CancelMission(1), Status('cancelled:south kensington station'))
    assert not bridge.active


def test_idle_without_trip_is_noop():
    """The middleman sends idle after every done / failure: nothing to cancel."""
    assert Bridge().on_mode(IDLE) == ()


def test_other_mode_cancels_active_trip():
    bridge = started_bridge()
    assert CancelMission(1) in bridge.on_mode(mode('location', 'kitchen'))


def test_new_gps_target_replaces_trip():
    bridge = started_bridge()
    commands = bridge.on_mode(mode('gps', 'royal albert hall'))
    assert commands == (CancelMission(1), Status('cancelled:south kensington station'),
                        StartMission(2, 'royal albert hall'))
    assert bridge.target == 'royal albert hall'


def test_cancel_while_waiting_clears_crossing_wait():
    bridge = started_bridge()
    bridge.on_state(1, MissionState.WAITING_AT_CROSSING, 'At crossing 1 of 6.')
    assert bridge.on_mode(STOP) == (
        CancelMission(1), CrossingWait(False), Status('cancelled:south kensington station'))


# --- state updates ---

def test_state_message_spoken_once():
    bridge = started_bridge()
    first = bridge.on_state(1, MissionState.PLANNING, 'Planning a route to X.')
    again = bridge.on_state(1, MissionState.PLANNING, 'Planning a route to X.')
    assert says(first) == ['Planning a route to X.']
    assert again == ()


def test_status_published_on_change_only():
    bridge = started_bridge()
    assert statuses(bridge.on_state(1, MissionState.READY, 'Summary.')) == [
        'ready:south kensington station']
    assert statuses(bridge.on_state(1, MissionState.READY, 'Summary.')) == []
    assert statuses(bridge.on_state(1, MissionState.NAVIGATING, 'Setting off.')) == [
        'going:south kensington station']


def test_new_message_same_state_spoken():
    bridge = started_bridge()
    bridge.on_state(1, MissionState.NAVIGATING, 'Setting off.')
    assert says(bridge.on_state(1, MissionState.NAVIGATING, 'Crossing now.')) == [
        'Crossing now.']


def test_repeated_line_after_another_spoken_again():
    bridge = started_bridge()
    bridge.on_state(1, MissionState.PAUSED, 'Paused.')
    bridge.on_state(1, MissionState.NAVIGATING, 'Continuing.')
    assert says(bridge.on_state(1, MissionState.PAUSED, 'Paused.')) == ['Paused.']


def test_empty_message_not_spoken():
    bridge = started_bridge()
    assert says(bridge.on_state(1, MissionState.PLANNING, '  ')) == []


def test_end_state_messages_not_spoken():
    """The middleman says "Arrived." and speaks failures; cancel has its own reply."""
    for state in (MissionState.ARRIVED, MissionState.FAILED, MissionState.IDLE):
        bridge = started_bridge()
        assert bridge.on_state(1, state, 'Some sentence.') == ()


def test_unknown_state_message_spoken():
    bridge = started_bridge()
    commands = bridge.on_state(1, 42, 'Something new.')
    assert says(commands) == ['Something new.']
    assert statuses(commands) == ['state_42:south kensington station']


def test_crossing_wait_on_and_off():
    bridge = started_bridge()
    waiting = bridge.on_state(1, MissionState.WAITING_AT_CROSSING, 'At crossing 1 of 6.')
    assert waiting == (CrossingWait(True), Status('waiting:crossing'),
                       Say('At crossing 1 of 6.'))
    crossing = bridge.on_state(1, MissionState.NAVIGATING, 'Crossing now.')
    assert crossing[0] == CrossingWait(False)


def test_state_for_old_goal_ignored():
    bridge = started_bridge()
    bridge.on_mode(mode('gps', 'royal albert hall'))
    assert bridge.on_state(1, MissionState.NAVIGATING, 'Old news.') == ()


def test_state_after_trip_ended_ignored():
    bridge = started_bridge()
    bridge.on_mode(STOP)
    assert bridge.on_state(1, MissionState.IDLE, 'Cancelled. The robot has stopped.') == ()


def test_new_trip_speaks_same_line_again():
    bridge = started_bridge()
    bridge.on_state(1, MissionState.PLANNING, 'Planning a route to X.')
    bridge.on_mode(GPS_STATION)
    assert says(bridge.on_state(2, MissionState.PLANNING, 'Planning a route to X.')) == [
        'Planning a route to X.']


# --- the user's go ---

def test_go_without_trip_ignored():
    assert Bridge().on_go() == ()


def test_go_forwarded_to_current_goal():
    bridge = started_bridge()
    bridge.on_mode(mode('gps', 'royal albert hall'))
    assert bridge.on_go() == (SendGo(2),)


def test_refused_go_spoken_every_time():
    bridge = started_bridge()
    reply = 'Still planning, please wait.'
    assert bridge.on_go_reply(1, False, reply) == (Say(reply),)
    assert bridge.on_go_reply(1, False, reply) == (Say(reply),)


def test_accepted_go_silent():
    assert started_bridge().on_go_reply(1, True, 'Starting.') == ()


def test_go_reply_for_old_goal_ignored():
    bridge = started_bridge()
    bridge.on_mode(STOP)
    assert bridge.on_go_reply(1, False, 'Nothing to start.') == ()


# --- results ---

def test_arrival_publishes_done():
    bridge = started_bridge()
    assert bridge.on_result(1, MissionError.NONE, 'You have arrived at X.') == (
        Status('arrived:south kensington station'), Done('south kensington station'))
    assert not bridge.active


def test_arrival_clears_crossing_wait():
    bridge = started_bridge()
    bridge.on_state(1, MissionState.WAITING_AT_CROSSING, 'At crossing 6 of 6.')
    assert bridge.on_result(1, MissionError.NONE, '')[0] == CrossingWait(False)


def test_failure_publishes_reason():
    bridge = started_bridge()
    text = "I couldn't find 'south kensington station'."
    assert bridge.on_result(1, MissionError.UNKNOWN_DESTINATION, text) == (
        Status('failed:unknown_destination'), Failed(text))


def test_failure_without_message_uses_fixed_text():
    bridge = started_bridge()
    commands = bridge.on_result(1, MissionError.NO_POSITION, '')
    assert commands[-1] == Failed(FAILURE_TEXTS[MissionError.NO_POSITION])


def test_failure_not_spoken_by_bridge():
    bridge = started_bridge()
    assert says(bridge.on_result(1, MissionError.PLANNING_FAILED, 'No path.')) == []


def test_cancel_by_someone_else_tells_middleman():
    bridge = started_bridge()
    commands = bridge.on_result(1, MissionError.CANCELED, 'Cancelled. The robot has stopped.')
    assert commands == (Status('cancelled:south kensington station'),
                        Failed('Cancelled. The robot has stopped.'))


def test_result_after_our_cancel_ignored():
    bridge = started_bridge()
    bridge.on_mode(STOP)
    assert bridge.on_result(1, MissionError.CANCELED, 'Cancelled.') == ()


def test_result_for_replaced_goal_ignored():
    bridge = started_bridge()
    bridge.on_mode(mode('gps', 'royal albert hall'))
    assert bridge.on_result(1, MissionError.NONE, '') == ()
    assert bridge.active


def test_second_result_ignored():
    bridge = started_bridge()
    bridge.on_result(1, MissionError.NONE, '')
    assert bridge.on_result(1, MissionError.NONE, '') == ()


@pytest.mark.parametrize('event, status, text', [
    ('on_goal_rejected', 'failed:rejected', REJECTED_TEXT),
    ('on_server_unavailable', 'failed:unavailable', UNAVAILABLE_TEXT),
])
def test_goal_not_sent(event, status, text):
    bridge = started_bridge()
    assert getattr(bridge, event)(1) == (Status(status), Failed(text))
    assert not bridge.active
    assert getattr(bridge, event)(1) == ()


# --- a whole trip ---

def test_whole_trip():
    bridge = Bridge()
    out = list(bridge.on_mode(GPS_STATION))
    out += bridge.on_state(1, MissionState.PLANNING, 'Planning a route to Station.')
    out += bridge.on_state(1, MissionState.READY, 'Station. Press go to start.')
    out += bridge.on_go()
    out += bridge.on_go_reply(1, True, 'Starting.')
    out += bridge.on_state(1, MissionState.NAVIGATING, 'Setting off.')
    out += bridge.on_state(1, MissionState.WAITING_AT_CROSSING, 'At crossing 1 of 1.')
    out += bridge.on_go()
    out += bridge.on_state(1, MissionState.NAVIGATING, 'Crossing now.')
    out += bridge.on_state(1, MissionState.ARRIVED, 'You have arrived at Station.')
    out += bridge.on_result(1, MissionError.NONE, 'You have arrived at Station.')
    assert says(out) == ['Planning a route to Station.', 'Station. Press go to start.',
                         'Setting off.', 'At crossing 1 of 1.', 'Crossing now.']
    assert statuses(out) == ['planning:south kensington station',
                             'ready:south kensington station',
                             'going:south kensington station', 'waiting:crossing',
                             'going:south kensington station',
                             'arrived:south kensington station']
    assert [c for c in out if isinstance(c, (SendGo, Done, CrossingWait))] == [
        SendGo(1), CrossingWait(True), SendGo(1), CrossingWait(False),
        Done('south kensington station')]


# --- refusing a trip while something is missing (Step 10.4) ---

NO_GPS = "I can't navigate by GPS yet: there is no GPS signal."


def test_blocker_refuses_start():
    bridge = Bridge()
    bridge.set_blocker(NO_GPS)
    assert bridge.on_mode(GPS_STATION) == (Status('failed:not_ready'), Failed(NO_GPS))
    assert not bridge.active and bridge.goal_id == 0


def test_blocker_cleared_allows_start():
    bridge = Bridge()
    bridge.set_blocker(NO_GPS)
    bridge.set_blocker('')
    assert bridge.blocker == ''
    assert bridge.on_mode(GPS_STATION) == (StartMission(1, 'south kensington station'),)


def test_blocker_does_not_stop_running_trip():
    bridge = started_bridge()
    bridge.set_blocker(NO_GPS)
    assert bridge.active
    assert says(bridge.on_state(1, MissionState.NAVIGATING, 'Setting off.')) == ['Setting off.']


def test_blocker_new_request_still_cancels_old_trip():
    bridge = started_bridge()
    bridge.set_blocker(NO_GPS)
    assert bridge.on_mode(mode('gps', 'royal albert hall')) == (
        CancelMission(1), Status('cancelled:south kensington station'),
        Status('failed:not_ready'), Failed(NO_GPS))


def test_blocker_ignored_for_stop():
    bridge = started_bridge()
    bridge.set_blocker(NO_GPS)
    assert bridge.on_mode(STOP) == (
        CancelMission(1), Status('cancelled:south kensington station'))


def test_blocker_none_and_spaces_mean_no_blocker():
    bridge = Bridge()
    bridge.set_blocker(None)
    assert bridge.blocker == ''
    bridge.set_blocker('   ')
    assert bridge.blocker == ''


# --- auto_go: no go button ---

def auto_bridge():
    """Return an auto_go bridge with one trip started (goal 1, the station)."""
    bridge = Bridge(auto_go=True)
    bridge.on_mode(GPS_STATION)
    return bridge


def test_auto_go_starts_without_confirmation():
    bridge = Bridge(auto_go=True)
    assert bridge.on_mode(GPS_STATION) == (
        StartMission(1, 'south kensington station', skip_confirmation=True),)


def test_auto_go_sends_go_once_when_waiting_at_a_crossing():
    bridge = auto_bridge()
    message = 'At crossing 1 of 2 (signalled). Press go when it is safe to cross.'
    first = bridge.on_state(1, MissionState.WAITING_AT_CROSSING, message)
    assert SendGo(1) in first
    assert says(first) == ['At crossing 1 of 2 (signalled).']   # no "Press go ..."
    assert SendGo(1) not in bridge.on_state(1, MissionState.WAITING_AT_CROSSING, message)
    # the next crossing gets its own go
    bridge.on_state(1, MissionState.NAVIGATING, 'Crossing now.')
    assert SendGo(1) in bridge.on_state(1, MissionState.WAITING_AT_CROSSING, message)


def test_auto_go_sends_go_when_the_route_waits_for_a_start():
    # a place found online is read back and waits, even with skip_confirmation
    bridge = auto_bridge()
    commands = bridge.on_state(1, MissionState.READY,
                               'Museum. About 200 m walk. Press go to start.')
    assert SendGo(1) in commands
    assert says(commands) == ['Museum. About 200 m walk.']


def test_auto_go_refused_is_silent_and_sent_again():
    bridge = auto_bridge()
    bridge.on_state(1, MissionState.READY, 'Route. Press go to start.')
    assert bridge.on_go_reply(1, False, "I'm not sure where we are yet. Please wait.") == ()
    assert SendGo(1) in bridge.on_state(1, MissionState.READY, 'Route. Press go to start.')


def test_auto_go_leaves_a_pause_alone():
    bridge = auto_bridge()
    commands = bridge.on_state(1, MissionState.PAUSED, 'Paused. Press go to continue.')
    assert SendGo(1) not in commands


def test_auto_go_stop_still_cancels():
    bridge = auto_bridge()
    assert CancelMission(1) in bridge.on_mode(STOP)
    assert not bridge.active


def test_without_auto_go_nothing_is_sent_or_cut():
    bridge = started_bridge()
    message = 'At crossing 1 of 2. Press go when it is safe to cross.'
    commands = bridge.on_state(1, MissionState.WAITING_AT_CROSSING, message)
    assert SendGo(1) not in commands
    assert says(commands) == [message]
