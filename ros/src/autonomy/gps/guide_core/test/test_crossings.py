"""Tests for guide_core.crossings."""

import math

from guide_core.crossings import (
    CrossingError, crossings_from_dict, crossings_to_dicts, prepare_crossings, RouteCrossing,
    stop_points,
)
from guide_core.route import InvalidRouteError, Route
from guide_core.walk_costs import CrossingType
import pytest

SIG = CrossingType.SIGNALLED
UNSIG = CrossingType.UNSIGNALLED
UNK = CrossingType.UNKNOWN


def _c(start, end, crossing_type=SIG, road='major', tactile=False, island=False):
    """Shorthand for a RouteCrossing."""
    return RouteCrossing(start, end, crossing_type, road, tactile, island)


def _route(length=100.0):
    """Return a straight route of the given length along x."""
    return Route([[0.0, 0.0], [length, 0.0]])


def _entry(**changes):
    """One valid route-file crossing entry, with optional changes."""
    entry = {'start_s': 10.0, 'end_s': 16.0, 'type': 'signalled',
             'crossed_road': 'major', 'tactile_paving': True}
    entry.update(changes)
    return entry


def _data(crossings):
    """Route-file data with the given crossings value."""
    return {'frame_id': 'map', 'waypoints': [[0.0, 0.0], [100.0, 0.0]],
            'crossings': crossings}


# The demo route's crossings (Imperial -> South Kensington, 649.3 m).
DEMO = (_c(240.8, 246.8, UNK, 'none'),
        _c(390.2, 398.5, SIG, 'major'),
        _c(400.3, 407.4, SIG, 'major'),
        _c(418.5, 425.0, SIG, 'minor'))


# RouteCrossing

def test_has_island_defaults_false():
    crossing = RouteCrossing(start_s=1.0, end_s=2.0, crossing_type=SIG,
                             crossed_road='minor', tactile_paving=False)
    assert crossing.has_island is False


# prepare_crossings

def test_prepare_empty():
    assert prepare_crossings([], 100.0) == ()


def test_prepare_sorts_by_start():
    result = prepare_crossings([_c(50, 55), _c(10, 15)], 100.0)
    assert [c.start_s for c in result] == [10.0, 50.0]


def test_prepare_keeps_valid_crossings_unchanged():
    crossings = (_c(10, 15, UNK, 'none', True, True), _c(50, 55))
    assert prepare_crossings(crossings, 100.0) == crossings


@pytest.mark.parametrize('start, end', [(10, 10), (12, 10)])
def test_prepare_rejects_start_not_before_end(start, end):
    with pytest.raises(CrossingError):
        prepare_crossings([_c(start, end)], 100.0)


@pytest.mark.parametrize('start, end', [(-1, 5), (95, 101)])
def test_prepare_rejects_outside_route(start, end):
    with pytest.raises(CrossingError):
        prepare_crossings([_c(start, end)], 100.0)


def test_prepare_clamps_tiny_overshoot():
    result = prepare_crossings([_c(-0.0005, 5), _c(95, 100.0005)], 100.0)
    assert result[0].start_s == 0.0
    assert result[1].end_s == 100.0


@pytest.mark.parametrize('start, end', [(math.nan, 5), (1, math.inf)])
def test_prepare_rejects_non_finite(start, end):
    with pytest.raises(CrossingError):
        prepare_crossings([_c(start, end)], 100.0)


def test_prepare_rejects_bad_crossed_road():
    with pytest.raises(CrossingError):
        prepare_crossings([_c(10, 15, road='motorway')], 100.0)


def test_prepare_rejects_type_given_as_string():
    with pytest.raises(CrossingError):
        prepare_crossings([_c(10, 15, crossing_type='signalled')], 100.0)


@pytest.mark.parametrize('tactile, island', [(1, False), (False, 'yes')])
def test_prepare_rejects_non_bool_flags(tactile, island):
    with pytest.raises(CrossingError):
        prepare_crossings([_c(10, 15, tactile=tactile, island=island)], 100.0)


def test_prepare_rejects_non_crossing_item():
    with pytest.raises(CrossingError):
        prepare_crossings([(10, 15)], 100.0)


def test_prepare_merges_overlapping():
    result = prepare_crossings([_c(10, 16), _c(14, 20)], 100.0)
    assert len(result) == 1
    assert (result[0].start_s, result[0].end_s) == (10.0, 20.0)


def test_prepare_keeps_touching_separate():
    # Two halves meeting at an island node: the meeting point is the stop.
    result = prepare_crossings([_c(10, 16), _c(16, 20)], 100.0)
    assert len(result) == 2
    assert stop_points(result, 1.5) == (8.5, 16.0)


def test_prepare_keeps_island_gap_separate():
    # Cromwell Road: the two halves are 1.8 m apart -> stay two crossings.
    result = prepare_crossings([_c(390.2, 398.5), _c(400.3, 407.4)], 649.3)
    assert len(result) == 2


@pytest.mark.parametrize('first, second, expected', [
    (SIG, SIG, SIG),
    (SIG, UNK, UNK),
    (UNK, UNSIG, UNSIG),
    (SIG, UNSIG, UNSIG),
])
def test_prepare_merged_type_is_worst(first, second, expected):
    result = prepare_crossings([_c(10, 16, first), _c(14, 20, second)], 100.0)
    assert result[0].crossing_type is expected


def test_prepare_merged_road_and_flags():
    result = prepare_crossings(
        [_c(10, 16, road='minor', tactile=True, island=False),
         _c(14, 20, road='major', tactile=False, island=True)], 100.0)
    assert result[0].crossed_road == 'major'
    assert result[0].tactile_paving is False
    assert result[0].has_island is True


def test_prepare_merges_chain_of_three():
    result = prepare_crossings([_c(10, 16), _c(15, 22), _c(21, 30), _c(40, 45)], 100.0)
    assert [(c.start_s, c.end_s) for c in result] == [(10.0, 30.0), (40.0, 45.0)]


# crossings_from_dict

def test_from_dict_missing_key_gives_empty():
    data = {'frame_id': 'map', 'waypoints': [[0.0, 0.0], [100.0, 0.0]]}
    assert crossings_from_dict(data, _route()) == ()


def test_from_dict_null_gives_empty():
    # 'crossings:' with nothing after it loads as None.
    assert crossings_from_dict(_data(None), _route()) == ()


def test_from_dict_reads_entry():
    result = crossings_from_dict(_data([_entry()]), _route())
    assert result == (_c(10.0, 16.0, SIG, 'major', True, False),)


def test_from_dict_reads_has_island():
    result = crossings_from_dict(_data([_entry(has_island=True)]), _route())
    assert result[0].has_island is True


def test_from_dict_rejects_not_list():
    with pytest.raises(CrossingError):
        crossings_from_dict(_data({'start_s': 1}), _route())


def test_from_dict_rejects_entry_not_mapping():
    with pytest.raises(CrossingError):
        crossings_from_dict(_data([[10, 16]]), _route())


def test_from_dict_rejects_missing_field():
    entry = _entry()
    del entry['crossed_road']
    with pytest.raises(CrossingError, match='crossed_road'):
        crossings_from_dict(_data([entry]), _route())


def test_from_dict_rejects_unknown_field():
    with pytest.raises(CrossingError, match='start'):
        crossings_from_dict(_data([_entry(start=3.0)]), _route())


def test_from_dict_rejects_bad_type_name():
    with pytest.raises(CrossingError, match='type'):
        crossings_from_dict(_data([_entry(type='zebra')]), _route())


def test_from_dict_rejects_bool_as_number():
    with pytest.raises(CrossingError):
        crossings_from_dict(_data([_entry(start_s=True)]), _route())


def test_from_dict_accepts_ints():
    result = crossings_from_dict(_data([_entry(start_s=10, end_s=16)]), _route())
    assert (result[0].start_s, result[0].end_s) == (10.0, 16.0)


def test_from_dict_error_is_invalid_route_error():
    with pytest.raises(InvalidRouteError):
        crossings_from_dict(_data('nonsense'), _route())


def test_from_dict_checks_route_length():
    with pytest.raises(CrossingError, match='outside'):
        crossings_from_dict(_data([_entry(end_s=150.0)]), _route())


# crossings_to_dicts

def test_to_dicts_fields_and_order():
    result = crossings_to_dicts([_c(10, 16, UNK, 'none', True, True)])
    assert result == [{'start_s': 10.0, 'end_s': 16.0, 'type': 'unknown',
                       'crossed_road': 'none', 'tactile_paving': True,
                       'has_island': True}]
    assert list(result[0]) == ['start_s', 'end_s', 'type', 'crossed_road',
                               'tactile_paving', 'has_island']


def test_to_dicts_round_trip():
    crossings = (_c(10, 16, UNK, 'none', True, False), _c(50, 55, UNSIG, 'minor'))
    assert crossings_from_dict(_data(crossings_to_dicts(crossings)), _route()) == crossings


def test_to_dicts_rounds_to_millimetre():
    result = crossings_to_dicts([_c(10.123456, 16.98765)])
    assert (result[0]['start_s'], result[0]['end_s']) == (10.123, 16.988)


# stop_points

def test_stop_points_no_crossings():
    assert stop_points((), 1.5) == ()


def test_stop_points_margin_before_start():
    assert stop_points((_c(50, 55),), 1.5) == (48.5,)


def test_stop_points_zero_margin_is_start():
    assert stop_points((_c(50, 55),), 0.0) == (50.0,)


def test_stop_points_clamped_at_route_start():
    assert stop_points((_c(0.5, 5),), 1.5) == (0.0,)


def test_stop_points_island_middle():
    # 400.3 - 1.5 = 398.8 is in the first half of the 1.8 m island -> middle.
    stops = stop_points((_c(390.2, 398.5), _c(400.3, 407.4)), 1.5)
    assert stops == pytest.approx((388.7, 399.4))


def test_stop_points_demo_route():
    assert stop_points(DEMO, 1.5) == pytest.approx((239.3, 388.7, 399.4, 417.0))


@pytest.mark.parametrize('margin', [-1.0, math.nan])
def test_stop_points_rejects_bad_margin(margin):
    with pytest.raises(ValueError):
        stop_points(DEMO, margin)
