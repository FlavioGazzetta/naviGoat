"""Tests for guide_core.walk_costs (plain dicts; no network, no osmnx)."""

import math

from guide_core.walk_costs import (
    classify_crossing, CostConfig, crossing_penalty, CrossingType, edge_cost, walkway_class,
    WalkwayClass,
)
import pytest

S = CrossingType.SIGNALLED
U = CrossingType.UNSIGNALLED
K = CrossingType.UNKNOWN
FOOTWAY = {'highway': 'footway'}


def crossing_node(**tags):
    """Tags of a crossing node, plus any extra tags given."""
    return {'highway': 'crossing', **tags}


# --- walkway_class -----------------------------------------------------------

@pytest.mark.parametrize('highway, expected', [
    ('footway', WalkwayClass.WALKWAY),
    ('pedestrian', WalkwayClass.WALKWAY),
    ('path', WalkwayClass.WALKWAY),
    ('residential', WalkwayClass.MINOR_ROAD),
    ('service', WalkwayClass.MINOR_ROAD),
    ('bridleway', WalkwayClass.MINOR_ROAD),
    ('trunk', WalkwayClass.MAJOR_ROAD),
    ('primary_link', WalkwayClass.MAJOR_ROAD),
    ('tertiary', WalkwayClass.MAJOR_ROAD),
    ('steps', WalkwayClass.FORBIDDEN),
    ('corridor', WalkwayClass.FORBIDDEN),
    (None, WalkwayClass.MINOR_ROAD),
    ('something_new', WalkwayClass.MINOR_ROAD),
])
def test_walkway_class(highway, expected):
    assert walkway_class(highway) is expected


# --- classify_crossing -------------------------------------------------------

@pytest.mark.parametrize('tags', [
    {'crossing': 'traffic_signals'},
    {'crossing:signals': 'yes'},
    {'crossing_ref': 'pelican'},
    {'crossing_ref': 'puffin'},
    {'crossing_ref': 'toucan'},
    {'crossing': ' Traffic_Signals '},
])
def test_signalled_crossings(tags):
    assert classify_crossing(tags) is S


@pytest.mark.parametrize('tags', [
    {'crossing': 'unmarked'},
    {'crossing': 'uncontrolled'},
    {'crossing': 'marked'},
    {'crossing': 'zebra'},
    {'crossing:signals': 'no'},
    {'crossing_ref': 'zebra'},
])
def test_unsignalled_crossings(tags):
    assert classify_crossing(tags) is U


@pytest.mark.parametrize('tags', [
    {},
    {'crossing': 'raised'},
    {'highway': 'crossing'},
])
def test_unknown_crossings(tags):
    assert classify_crossing(tags) is K


def test_contradictory_tags_count_as_unsignalled():
    assert classify_crossing({'crossing': 'traffic_signals'}, {'crossing': 'uncontrolled'}) is U


def test_evidence_combined_from_all_tag_dicts():
    assert classify_crossing({'highway': 'crossing'}, {'crossing': 'traffic_signals'}) is S


# --- crossing_penalty --------------------------------------------------------

@pytest.mark.parametrize('crossing_type, road, expected', [
    (S, 'minor', 10.0),
    (U, 'minor', 30.0),
    (K, 'minor', 40.0),
    (S, 'major', 20.0),
    (U, 'major', 300.0),
    (K, 'major', 300.0),
    (U, None, 300.0),
    (S, None, 20.0),
    (U, 'none', 30.0),
    (K, 'none', 40.0),
])
def test_crossing_penalty_table(crossing_type, road, expected):
    assert crossing_penalty(crossing_type, road, CostConfig()) == expected


# --- CostConfig --------------------------------------------------------------

def test_cost_config_defaults():
    c = CostConfig()
    assert (c.minor_road_factor, c.major_road_factor) == (10.0, 50.0)
    assert (c.signalled_minor, c.unsignalled_minor, c.unknown_minor) == (10.0, 30.0, 40.0)
    assert (c.signalled_major, c.unsignalled_major, c.unknown_major) == (20.0, 300.0, 300.0)


@pytest.mark.parametrize('kwargs', [
    {'minor_road_factor': 0.5},
    {'major_road_factor': 0.9},
    {'unsignalled_major': -1.0},
    {'signalled_minor': math.nan},
    {'unknown_minor': math.inf},
])
def test_cost_config_rejects_invalid(kwargs):
    with pytest.raises(ValueError):
        CostConfig(**kwargs)


# --- edge_cost ---------------------------------------------------------------

def test_plain_footway_costs_its_length():
    assert edge_cost(10.0, FOOTWAY, {}, {}) == pytest.approx(10.0)


def test_minor_road_costs_ten_times_length():
    assert edge_cost(10.0, {'highway': 'residential'}, {}, {}) == pytest.approx(100.0)


def test_major_road_costs_fifty_times_length():
    assert edge_cost(10.0, {'highway': 'primary'}, {}, {}) == pytest.approx(500.0)


def test_steps_are_forbidden():
    assert edge_cost(10.0, {'highway': 'steps'}, {}, {}) is None


def test_stepping_onto_signalled_crossing_of_minor_road():
    node = crossing_node(crossing='traffic_signals', crossed_road='minor')
    assert edge_cost(5.0, FOOTWAY, {}, node) == pytest.approx(15.0)


def test_leaving_crossing_node_is_not_charged():
    node = crossing_node(crossing='traffic_signals', crossed_road='minor')
    assert edge_cost(5.0, FOOTWAY, node, {}) == pytest.approx(5.0)


def test_road_edge_into_crossing_node_is_not_charged():
    node = crossing_node(crossing='unmarked', crossed_road='minor')
    assert edge_cost(5.0, {'highway': 'residential'}, {}, node) == pytest.approx(50.0)


def test_unknown_crossed_road_counts_as_major():
    node = crossing_node(crossing='unmarked')
    assert edge_cost(5.0, FOOTWAY, {}, node) == pytest.approx(305.0)


def test_crossing_way_without_crossing_nodes_is_charged():
    way = {'highway': 'footway', 'footway': 'crossing', 'crossing': 'traffic_signals'}
    assert edge_cost(8.0, way, {}, {}) == pytest.approx(28.0)


def test_crossing_way_leaving_crossing_node_is_not_charged_again():
    way = {'highway': 'footway', 'footway': 'crossing', 'crossing': 'traffic_signals'}
    node = crossing_node(crossing='traffic_signals', crossed_road='minor')
    assert edge_cost(8.0, way, node, {}) == pytest.approx(8.0)


@pytest.mark.parametrize('length', [-1.0, math.nan, math.inf])
def test_invalid_length_rejected(length):
    with pytest.raises(ValueError):
        edge_cost(length, FOOTWAY, {}, {})


def test_custom_config_is_used():
    config = CostConfig(minor_road_factor=2.0)
    assert edge_cost(10.0, {'highway': 'residential'}, {}, {}, config) == pytest.approx(20.0)


def test_way_tags_give_type_when_node_has_none():
    way = {'highway': 'footway', 'footway': 'crossing', 'crossing': 'traffic_signals'}
    node = crossing_node(crossed_road='minor')
    assert edge_cost(5.0, way, {}, node) == pytest.approx(15.0)
