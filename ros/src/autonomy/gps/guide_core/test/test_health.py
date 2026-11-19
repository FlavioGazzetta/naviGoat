"""Tests for guide_core.health (the /gps/health logic)."""

import json
import math

from guide_core.health import (
    evaluate, gps_part, Health, HealthConfig, HealthInputs, PART_NAMES, position_part,
    READY_TEXT,
)
from guide_core.localization_quality import Quality
import pytest

NOW = 100.0
CONFIG = HealthConfig()


def all_ok(**changes):
    """Return inputs where everything is ready, with some fields changed."""
    values = {'now': NOW, 'gps_message_time': NOW - 0.2, 'gps_fix_time': NOW - 0.2,
              'status_time': NOW - 0.4, 'quality': Quality.GOOD, 'initialised': True,
              'ever_localised': True, 'mission_ready': True, 'planner_ready': True,
              'nav2_ready': True, 'geocoder_ready': True}
    values.update(changes)
    return HealthInputs(**values)


# --- the gps part ---

def test_gps_never_heard():
    assert gps_part(HealthInputs(now=NOW), CONFIG).detail == 'no messages on /gps/fix'


def test_gps_topic_name_in_detail():
    inputs = HealthInputs(now=NOW, gps_topic='/ublox/fix')
    assert gps_part(inputs, CONFIG).detail == 'no messages on /ublox/fix'


def test_gps_silent_for_a_while():
    part = gps_part(all_ok(gps_message_time=NOW - 12.0), CONFIG)
    assert not part.ok and part.detail == 'no recent messages on /gps/fix'


def test_gps_silent_detail_does_not_count_seconds():
    """The detail must not change every second, or /gps/health would too."""
    assert (gps_part(all_ok(gps_message_time=NOW - 3.0), CONFIG)
            == gps_part(all_ok(gps_message_time=NOW - 30.0), CONFIG))


def test_gps_messages_without_fix():
    part = gps_part(all_ok(gps_fix_time=NOW - 30.0), CONFIG)
    assert not part.ok and part.detail == 'GPS has no fix'


def test_gps_never_fixed():
    assert gps_part(all_ok(gps_fix_time=None), CONFIG).detail == 'GPS has no fix'


def test_gps_ok():
    part = gps_part(all_ok(), CONFIG)
    assert part.ok and part.detail == 'receiving fixes' and not part.blocks


def test_gps_timeout_edge():
    assert gps_part(all_ok(gps_message_time=NOW - 2.0, gps_fix_time=NOW - 2.0), CONFIG).ok
    assert not gps_part(all_ok(gps_fix_time=NOW - 2.1), CONFIG).ok


# --- the position part ---

def test_position_localizer_silent():
    part = position_part(all_ok(status_time=None), CONFIG)
    assert part.detail == 'localizer is not running' and part.blocks


def test_position_status_stale():
    assert not position_part(all_ok(status_time=NOW - 3.0), CONFIG).ok


def test_position_not_initialised():
    part = position_part(all_ok(initialised=False, quality=Quality.LOST), CONFIG)
    assert part.detail == 'not localised yet' and part.blocks


def test_position_lost():
    part = position_part(all_ok(quality=Quality.LOST), CONFIG)
    assert part.detail == 'position lost' and part.blocks


def test_position_settling_after_init_is_not_lost():
    """The localizer starts at LOST and improves after its hold time."""
    part = position_part(all_ok(quality=Quality.LOST, ever_localised=False), CONFIG)
    assert part.detail == 'not localised yet' and part.blocks


def test_position_degraded_is_ok():
    part = position_part(all_ok(quality=Quality.DEGRADED), CONFIG)
    assert part.ok and not part.blocks and part.detail == 'GPS weak, position less certain'


def test_settling_refusal_is_still_working_out():
    health = evaluate(all_ok(quality=Quality.LOST, ever_localised=False))
    assert health.summary == "I can't navigate by GPS yet: I'm still working out where we are."


# --- the whole picture ---

def test_everything_ready():
    health = evaluate(all_ok())
    assert health.ready and health.summary == READY_TEXT
    assert all(p.ok for p in health.parts)


def test_parts_in_fixed_order():
    assert tuple(p.name for p in evaluate(all_ok()).parts) == PART_NAMES


def test_nothing_running():
    health = evaluate(HealthInputs(now=NOW))
    assert not health.ready
    assert health.summary == "GPS navigation isn't running."


def test_no_gps_signal_refusal():
    health = evaluate(all_ok(gps_message_time=None, gps_fix_time=None, initialised=False,
                             quality=Quality.LOST))
    assert health.summary == "I can't navigate by GPS yet: there is no GPS signal."


def test_still_localising_refusal():
    health = evaluate(all_ok(initialised=False, quality=Quality.LOST))
    assert health.summary == "I can't navigate by GPS yet: I'm still working out where we are."


def test_localizer_down_refusal():
    health = evaluate(all_ok(status_time=None))
    assert health.summary == "I can't navigate by GPS: the localizer isn't running."


def test_position_lost_refusal():
    health = evaluate(all_ok(quality=Quality.LOST))
    assert health.summary == "I can't navigate by GPS: I've lost our position."


def test_planner_down_refusal():
    health = evaluate(all_ok(planner_ready=False))
    assert not health.ready
    assert health.summary == "I can't plan a route: the route planner isn't running."
    assert health.part('planner').detail == 'route planner is not running'


def test_nav2_down_refusal():
    health = evaluate(all_ok(nav2_ready=False))
    assert health.summary == "I can't drive: Nav2 isn't running."


def test_mission_down_explains_first():
    health = evaluate(all_ok(mission_ready=False, nav2_ready=False, status_time=None))
    assert health.summary == "GPS navigation isn't running."


def test_position_explains_before_planner_and_nav2():
    health = evaluate(all_ok(initialised=False, planner_ready=False, nav2_ready=False))
    assert health.summary.startswith("I can't navigate by GPS yet")


def test_gps_lost_with_position_still_ready():
    """In a tunnel: no GPS, but SLAM keeps the position, so a trip may start."""
    health = evaluate(all_ok(gps_fix_time=NOW - 60.0, quality=Quality.DEGRADED))
    assert health.ready
    assert not health.part('gps').ok


def test_geocoder_down_still_ready():
    health = evaluate(all_ok(geocoder_ready=False))
    assert health.ready
    assert health.part('geocoder').detail == 'geocoder is not running'


def test_to_json():
    data = json.loads(evaluate(all_ok(nav2_ready=False)).to_json())
    assert data['ready'] is False
    assert data['summary'] == "I can't drive: Nav2 isn't running."
    assert list(data['parts']) == list(PART_NAMES)
    assert data['parts']['nav2'] == {'ok': False, 'detail': 'Nav2 is not running'}


def test_to_json_same_inputs_same_text():
    assert evaluate(all_ok()).to_json() == evaluate(all_ok(now=NOW + 0.5)).to_json()


def test_health_part_lookup():
    health = evaluate(all_ok())
    assert isinstance(health, Health)
    assert health.part('gps').name == 'gps'


@pytest.mark.parametrize('field', ['gps_timeout', 'status_timeout'])
def test_config_rejects_non_positive(field):
    with pytest.raises(ValueError):
        HealthConfig(**{field: 0.0})
    with pytest.raises(ValueError):
        HealthConfig(**{field: math.nan})


def test_nav2_not_required_does_not_block():
    health = evaluate(HealthInputs(
        now=100.0, gps_message_time=99.5, gps_fix_time=99.5, status_time=99.5,
        quality=Quality.GOOD, initialised=True, ever_localised=True, mission_ready=True,
        planner_ready=True, nav2_ready=False, geocoder_ready=True, nav2_required=False))
    assert health.ready
    nav2 = next(p for p in health.parts if p.name == 'nav2')
    assert not nav2.ok and nav2.detail == 'Nav2 is not running'
