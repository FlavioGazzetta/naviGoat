"""Tests for crossing a road by walking along another road through a junction.

walk_costs.edge_cost charges it as an unknown crossing of the most important
other road meeting there, and route_planner reports it as a crossing (so the
robot stops before it). Junction nodes carry junction_major_dirs /
junction_minor_dirs (road directions of each class, set by fetch_osm_graph).
"""

from guide_core.geo import enu_to_latlon
from guide_core.route_planner import PlannerConfig, RoutePlanner
from guide_core.walk_costs import (
    crossed_road_at_junction, crossed_road_leaving_junction, CrossingType, edge_cost,
)
import networkx as nx
import pytest

ORIGIN = (51.4988, -0.1749)
# A minor road straight across a major road: 2 directions of each.
JUNCTION = {'junction_major_dirs': '2', 'junction_minor_dirs': '2'}
MINOR_ROAD = {'highway': 'residential'}
FOOTWAY = {'highway': 'footway'}


# --- walk_costs -------------------------------------------------------------------

def test_minor_road_into_major_junction_pays_unknown_major():
    assert edge_cost(5.0, MINOR_ROAD, {}, JUNCTION) == pytest.approx(5 * 10 + 300)


def test_major_road_past_minor_side_street_crosses_minor():
    # Walking the major road itself, the minor road meeting it is crossed.
    assert edge_cost(5.0, {'highway': 'primary'}, {}, JUNCTION) == pytest.approx(5 * 50 + 40)


def test_footway_into_junction_node_not_charged():
    assert edge_cost(5.0, FOOTWAY, {}, JUNCTION) == pytest.approx(5.0)


def test_minor_road_into_ordinary_node_unchanged():
    assert edge_cost(5.0, MINOR_ROAD, {}, {}) == pytest.approx(50.0)


@pytest.mark.parametrize('highway, major, minor, expected', [
    ('residential', 2, 2, 'major'),   # minor road across a major one
    ('residential', 1, 2, 'major'),   # minor road ending at a major one
    ('residential', 0, 3, 'minor'),   # side street of a minor road
    ('residential', 0, 2, None),      # the minor road just continues
    ('primary', 2, 1, 'minor'),       # major road past a minor side street
    ('primary', 3, 0, 'major'),       # two major roads meet
    ('primary', 2, 0, None),          # the major road just continues
    ('footway', 2, 2, None),          # not walking along a road
])
def test_crossed_road_at_junction(highway, major, minor, expected):
    tags = {'junction_major_dirs': str(major), 'junction_minor_dirs': str(minor)}
    assert crossed_road_at_junction({'highway': highway}, tags) == expected


# --- route_planner ----------------------------------------------------------------

def _graph(detour):
    """Return a minor road N-J-S (30 m) straight across a major road at junction J (0, 0).

    With detour set, there is also a pavement route via a signalled crossing
    `detour` m east: N -> NE -> crossing -> SE -> S, cost 2 x detour + 30 + 20.
    Straight down the road: 30 x 10 + 300 = 600.
    """
    g = nx.MultiDiGraph()

    def node(name, east, north, **tags):
        lat, lon = enu_to_latlon(east, north, *ORIGIN)
        g.add_node(name, x=lon, y=lat, **tags)

    def way(a, b, tags):
        g.add_edge(a, b, **tags)
        g.add_edge(b, a, **tags)

    node('N', 0, 15)
    node('J', 0, 0, **JUNCTION)
    node('S', 0, -15)
    way('N', 'J', MINOR_ROAD)
    way('J', 'S', MINOR_ROAD)
    if detour is not None:
        node('NE', detour, 15)
        node('C', detour, 0, highway='crossing', crossing='traffic_signals',
             crossed_road='major')
        node('SE', detour, -15)
        way('N', 'NE', FOOTWAY)
        way('NE', 'C', {'highway': 'footway', 'footway': 'crossing'})
        way('C', 'SE', {'highway': 'footway', 'footway': 'crossing'})
        way('SE', 'S', FOOTWAY)
    return g


def _plan(graph, config=None):
    planner = RoutePlanner(graph, *ORIGIN, config)
    return planner.plan(*enu_to_latlon(0, 15, *ORIGIN), *enu_to_latlon(0, -15, *ORIGIN))


def test_planner_prefers_signalled_crossing_over_junction():
    planned = _plan(_graph(detour=100.0))                # 250 < 600
    assert max(x for x, _ in planned.route.points) == pytest.approx(100.0, abs=0.01)
    assert [c.crossing_type for c in planned.crossings] == [CrossingType.SIGNALLED]


def test_junction_reported_as_crossing_when_only_way():
    planned = _plan(_graph(detour=None))
    assert len(planned.crossings) == 1
    c = planned.crossings[0]
    assert c.crossing_type is CrossingType.UNKNOWN and c.crossed_road == 'major'
    # Junction at s = 15, +/- junction_half_width (10 m).
    assert (c.start_s, c.end_s) == (pytest.approx(5.0), pytest.approx(25.0))


def test_junction_half_width_is_used():
    planned = _plan(_graph(detour=None), PlannerConfig(junction_half_width=4.0))
    c = planned.crossings[0]
    assert (c.start_s, c.end_s) == (pytest.approx(11.0), pytest.approx(19.0))


def test_invalid_junction_half_width_rejected():
    with pytest.raises(ValueError):
        PlannerConfig(junction_half_width=0.0)


def test_side_street_reported_as_minor_crossing():
    # Walking a street with pavements (sidewalk=both) A-J-B past a side
    # street at J: a minor-road crossing, +/- side_street_half_width (5 m).
    g = nx.MultiDiGraph()
    for name, east, north, tags in (
            ('A', 0, 0, {}), ('B', 0, -40, {}), ('X', 30, -20, {}),
            ('J', 0, -20, {'junction_major_dirs': '0', 'junction_minor_dirs': '3'})):
        lat, lon = enu_to_latlon(east, north, *ORIGIN)
        g.add_node(name, x=lon, y=lat, **tags)
    street = {'highway': 'residential', 'sidewalk': 'both'}
    for u, v, tags in (('A', 'J', street), ('J', 'B', street), ('J', 'X', MINOR_ROAD)):
        g.add_edge(u, v, **tags)
        g.add_edge(v, u, **tags)
    planner = RoutePlanner(g, *ORIGIN)
    planned = planner.plan(*enu_to_latlon(0, 0, *ORIGIN), *enu_to_latlon(0, -40, *ORIGIN))
    assert planned.warnings == ()            # A and B are on a street with pavements
    assert len(planned.crossings) == 1
    c = planned.crossings[0]
    assert (c.crossing_type, c.crossed_road) == (CrossingType.UNKNOWN, 'minor')
    assert (c.start_s, c.end_s) == (pytest.approx(15.0), pytest.approx(25.0))


def test_invalid_side_street_half_width_rejected():
    with pytest.raises(ValueError):
        PlannerConfig(side_street_half_width=0.0)


# --- Turning off a major road ------------------------------------------------------

@pytest.mark.parametrize('highway, major, minor, expected', [
    ('residential', 2, 1, 'major'),   # leaving a junction on a major road
    ('residential', 0, 3, None),      # leaving a minor junction
    ('primary', 2, 1, None),          # leaving along the major road itself
    ('footway', 2, 1, None),          # not walking along a road
])
def test_crossed_road_leaving_junction(highway, major, minor, expected):
    tags = {'junction_major_dirs': str(major), 'junction_minor_dirs': str(minor)}
    assert crossed_road_leaving_junction({'highway': highway}, tags) == expected


def test_minor_road_leaving_major_junction_pays_unknown_major():
    assert edge_cost(5.0, MINOR_ROAD, JUNCTION, {}) == pytest.approx(5 * 10 + 300)


def test_straight_across_along_minor_road_charged_twice():
    entering = edge_cost(5.0, MINOR_ROAD, {}, JUNCTION)
    leaving = edge_cost(5.0, MINOR_ROAD, JUNCTION, {})
    assert entering + leaving == pytest.approx(2 * (5 * 10 + 300))


def test_turn_off_major_road_reported_as_major_crossing():
    # Walking the major road M1-J (pavement tagged), then turning into the
    # side street J-X: the major road is crossed at J, +/- 10 m.
    g = nx.MultiDiGraph()
    for name, east, north, tags in (
            ('M1', 0, 0, {}), ('M2', 60, 0, {}), ('X', 30, -40, {}),
            ('J', 30, 0, {'junction_major_dirs': '2', 'junction_minor_dirs': '1'})):
        lat, lon = enu_to_latlon(east, north, *ORIGIN)
        g.add_node(name, x=lon, y=lat, **tags)
    major = {'highway': 'primary', 'sidewalk': 'both'}
    side = {'highway': 'residential', 'sidewalk': 'both'}
    for u, v, tags in (('M1', 'J', major), ('J', 'M2', major), ('J', 'X', side)):
        g.add_edge(u, v, **tags)
        g.add_edge(v, u, **tags)
    planned = RoutePlanner(g, *ORIGIN).plan(*enu_to_latlon(0, 0, *ORIGIN),
                                            *enu_to_latlon(30, -40, *ORIGIN))
    assert len(planned.crossings) == 1
    c = planned.crossings[0]
    assert (c.crossing_type, c.crossed_road) == (CrossingType.UNKNOWN, 'major')
    assert (c.start_s, c.end_s) == (pytest.approx(20.0), pytest.approx(40.0))
