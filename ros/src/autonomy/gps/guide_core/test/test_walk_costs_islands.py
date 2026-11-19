"""Tests for the pedestrian-island part of guide_core.walk_costs."""

from guide_core.walk_costs import CostConfig, edge_cost, is_island_tagged
import pytest

WALKWAY = {'highway': 'footway'}
CROSSING_WAY = {'highway': 'footway', 'footway': 'crossing'}
SIGNALLED_MAJOR_NODE = {'highway': 'crossing', 'crossing': 'traffic_signals',
                        'crossed_road': 'major'}           # penalty 20


def test_island_penalty_default():
    assert CostConfig().island_penalty == 50.0


def test_zero_island_penalty_allowed():
    assert CostConfig(island_penalty=0.0).island_penalty == 0.0


def test_negative_island_penalty_rejected():
    with pytest.raises(ValueError):
        CostConfig(island_penalty=-1.0)


def test_onto_island_default_unchanged():
    assert edge_cost(4.0, WALKWAY, {}, {}) == pytest.approx(4.0)


def test_onto_island_adds_penalty():
    assert edge_cost(4.0, WALKWAY, {}, {}, onto_island=True) == pytest.approx(54.0)


def test_onto_island_ignored_on_roads():
    # Walking along a road is never "stepping onto an island".
    assert edge_cost(4.0, {'highway': 'residential'}, {}, {}, onto_island=True) \
        == pytest.approx(40.0)


def test_onto_island_with_crossing_node_adds_both():
    cost = edge_cost(4.0, CROSSING_WAY, {}, SIGNALLED_MAJOR_NODE, onto_island=True)
    assert cost == pytest.approx(4.0 + 20.0 + 50.0)


def test_island_tagged_way_onto_crossing_node():
    edge = dict(CROSSING_WAY, **{'crossing:island': 'yes'})
    assert edge_cost(4.0, edge, {}, SIGNALLED_MAJOR_NODE) == pytest.approx(74.0)


def test_island_tagged_way_and_onto_island_charged_once():
    edge = dict(CROSSING_WAY, **{'crossing:island': 'yes'})
    assert edge_cost(4.0, edge, {}, SIGNALLED_MAJOR_NODE, onto_island=True) \
        == pytest.approx(74.0)


def test_island_tagged_way_fallback_without_crossing_node():
    # Crossing way with no crossing node: crossed road unknown -> major (20).
    edge = dict(CROSSING_WAY, **{'crossing': 'traffic_signals', 'crossing:island': 'yes'})
    assert edge_cost(4.0, edge, {}, {}) == pytest.approx(74.0)


@pytest.mark.parametrize('tags, expected', [
    ({'crossing:island': 'yes'}, True),
    ({'traffic_calming': 'island'}, True),
    ({'crossing:island': 'no'}, False),
    ({'traffic_calming': 'table'}, False),
    (None, False),
])
def test_is_island_tagged(tags, expected):
    assert is_island_tagged(tags) is expected
