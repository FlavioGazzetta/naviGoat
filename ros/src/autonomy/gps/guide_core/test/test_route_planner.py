"""Tests for guide_core.route_planner on small hand-built graphs (no osmnx, no network)."""

import math

from guide_core.geo import enu_to_latlon
from guide_core.route import Route
from guide_core.route_planner import PlannedRoute, PlannerConfig, PlanningError, RoutePlanner
from guide_core.walk_costs import CostConfig, CrossingType
import networkx as nx
import pytest

ORIGIN = (51.4988, -0.1749)
FOOTWAY = {'highway': 'footway'}
CROSSING_WAY = {'highway': 'footway', 'footway': 'crossing'}


def latlon(east, north):
    """(lat, lon) of a point given in metres east/north of the origin."""
    return enu_to_latlon(east, north, *ORIGIN)


def make_graph(nodes, edges):
    """Build an osmnx-shaped graph.

    nodes: {name: (east, north, tags)}. edges: [(a, b, tags)], each added in
    both directions (walking graphs are two-way).
    """
    graph = nx.MultiDiGraph()
    for name, (east, north, tags) in nodes.items():
        lat, lon = latlon(east, north)
        graph.add_node(name, x=lon, y=lat, **tags)
    for a, b, tags in edges:
        graph.add_edge(a, b, **tags)
        graph.add_edge(b, a, **tags)
    return graph


def plan(graph, start, goal, config=None):
    """Plan between two (east, north) points."""
    return RoutePlanner(graph, *ORIGIN, config).plan(*latlon(*start), *latlon(*goal))


def road_graph():
    """Return a major road with a pavement on each side.

    Unsignalled crossing at x=0 (C0), signalled crossing at x=100 (C2),
    steps at x=50 (forbidden). Choices from N0 to S0:
    straight across C0 = 20 m, cost 320; via steps = 120 m, forbidden;
    round via C2 = 220 m, cost 240.
    """
    nodes = {
        'N0': (0, 10, {}), 'N1': (50, 10, {}), 'N2': (100, 10, {}),
        'S0': (0, -10, {}), 'S1': (50, -10, {}), 'S2': (100, -10, {}),
        'C0': (0, 0, {'highway': 'crossing', 'crossing': 'uncontrolled', 'crossed_road': 'major'}),
        'C2': (100, 0, {'highway': 'crossing', 'crossing': 'traffic_signals',
                        'crossed_road': 'major'}),
    }
    edges = [
        ('N0', 'N1', FOOTWAY), ('N1', 'N2', FOOTWAY),
        ('S0', 'S1', FOOTWAY), ('S1', 'S2', FOOTWAY),
        ('N0', 'C0', CROSSING_WAY), ('C0', 'S0', CROSSING_WAY),
        ('N2', 'C2', CROSSING_WAY), ('C2', 'S2', CROSSING_WAY),
        ('N1', 'S1', {'highway': 'steps'}),
        ('C0', 'C2', {'highway': 'primary'}),  # the road itself
    ]
    return make_graph(nodes, edges)


def shared_space_graph(crossing_tags):
    """Return Exhibition Road in miniature.

    A straight pedestrian line A-M-B with a crossing node M halfway, and a
    parallel road R1-R2 joined to A and B.

    Straight: 100 m + crossing penalty. Detour via the road, with the
    minor road factor set to 3 (SHARED_SPACE_CONFIG; the default is 10):
    14.1 + 80 x 3 + 14.1 = 268.3 cost (108.3 m).
    """
    pedestrian = {'highway': 'pedestrian'}
    nodes = {
        'A': (0, 0, {}), 'B': (0, -100, {}),
        'M': (0, -50, {'highway': 'crossing', **crossing_tags}),
        'R1': (10, -10, {}), 'R2': (10, -90, {}),
    }
    edges = [('A', 'M', pedestrian), ('M', 'B', pedestrian),
             ('A', 'R1', FOOTWAY), ('R1', 'R2', {'highway': 'unclassified'}),
             ('R2', 'B', FOOTWAY)]
    return make_graph(nodes, edges)


# Minor road factor 3 (the default is 10), so the road detour in
# shared_space_graph stays cheaper than an unknown major crossing.
SHARED_SPACE_CONFIG = PlannerConfig(costs=CostConfig(minor_road_factor=3.0))


def test_prefers_signalled_crossing():
    result = plan(road_graph(), start=(0, 12), goal=(0, -12))
    assert isinstance(result, PlannedRoute)
    assert isinstance(result.route, Route)
    assert result.length == pytest.approx(220.0)
    assert len(result.crossings) == 1
    crossing = result.crossings[0]
    assert crossing.crossing_type is CrossingType.SIGNALLED
    assert crossing.crossed_road == 'major'
    assert crossing.start_s == pytest.approx(100.0)
    assert crossing.end_s == pytest.approx(120.0)


def test_goes_straight_across_when_penalty_removed():
    config = PlannerConfig(costs=CostConfig(unsignalled_major=0.0))
    result = plan(road_graph(), (0, 12), (0, -12), config)
    assert result.length == pytest.approx(20.0)
    crossing = result.crossings[0]
    assert crossing.crossing_type is CrossingType.UNSIGNALLED
    assert crossing.start_s == pytest.approx(0.0)
    assert crossing.end_s == pytest.approx(20.0)


def test_steps_only_connection_gives_no_route():
    nodes = {'N0': (0, 10, {}), 'N1': (50, 10, {}), 'S1': (50, -10, {}), 'S0': (0, -10, {})}
    edges = [('N0', 'N1', FOOTWAY), ('N1', 'S1', {'highway': 'steps'}), ('S1', 'S0', FOOTWAY)]
    with pytest.raises(PlanningError, match='No walkable route'):
        plan(make_graph(nodes, edges), (0, 12), (0, -12))


def test_route_points_and_latlons():
    result = plan(road_graph(), (0, 12), (0, -12))
    expected = [(0, 10), (100, 10), (100, -10), (0, -10)]
    assert len(result.route.points) == len(expected)
    for got, want in zip(result.route.points, expected):
        assert got == pytest.approx(want, abs=1e-6)
    assert len(result.latlons) == len(expected)
    assert result.latlons[0] == pytest.approx(latlon(0, 10), abs=1e-9)
    assert result.latlons[-1] == pytest.approx(latlon(0, -10), abs=1e-9)


def test_snap_distances_reported():
    result = plan(road_graph(), (0, 12), (0, -12))
    assert result.start_snap_distance == pytest.approx(2.0)
    assert result.goal_snap_distance == pytest.approx(2.0)


def test_start_too_far_from_walkway_rejected():
    # 80 m from the nearest walkway (limit 60) and from any node (limit 30).
    with pytest.raises(PlanningError, match='start is 80.0 m'):
        plan(road_graph(), (0, 90), (0, -12))


def test_same_start_and_goal_node_rejected():
    with pytest.raises(PlanningError, match='same point'):
        plan(road_graph(), (0, 12), (1, 11))


def test_kerb_points_kept_on_straight_route():
    nodes = {
        'P0': (0, 0, {}), 'K1': (30, 0, {}), 'K2': (40, 0, {}), 'P3': (60, 0, {}),
        'C': (35, 0, {'highway': 'crossing', 'crossing': 'traffic_signals',
                      'crossed_road': 'minor', 'tactile_paving': 'yes'}),
    }
    edges = [('P0', 'K1', FOOTWAY), ('K1', 'C', CROSSING_WAY),
             ('C', 'K2', CROSSING_WAY), ('K2', 'P3', FOOTWAY)]
    result = plan(make_graph(nodes, edges), (0, 1), (60, 1))
    xs = [p[0] for p in result.route.points]
    assert xs == pytest.approx([0, 30, 40, 60], abs=1e-6)  # kerbs kept, C dropped
    crossing = result.crossings[0]
    assert crossing.start_s == pytest.approx(30.0)
    assert crossing.end_s == pytest.approx(40.0)
    assert crossing.crossing_type is CrossingType.SIGNALLED
    assert crossing.crossed_road == 'minor'
    assert crossing.tactile_paving is True


def test_crossing_node_without_crossing_ways():
    nodes = {
        'F0': (0, 0, {}), 'F1': (10, 0, {}), 'F3': (30, 0, {}), 'F4': (40, 0, {}),
        'X': (20, 0, {'highway': 'crossing', 'crossing': 'unmarked', 'crossed_road': 'minor'}),
    }
    edges = [('F0', 'F1', FOOTWAY), ('F1', 'X', FOOTWAY),
             ('X', 'F3', FOOTWAY), ('F3', 'F4', FOOTWAY)]
    result = plan(make_graph(nodes, edges), (0, 1), (40, 1))
    xs = [p[0] for p in result.route.points]
    assert xs == pytest.approx([0, 20, 40], abs=1e-6)  # the crossing node is kept
    crossing = result.crossings[0]
    assert crossing.start_s == pytest.approx(17.0)     # node at 20 m, +/- 3 m
    assert crossing.end_s == pytest.approx(23.0)
    assert crossing.crossing_type is CrossingType.UNSIGNALLED
    assert crossing.tactile_paving is False


def test_node_crossing_span_clamped_to_route_start():
    nodes = {'F0': (0, 0, {}), 'F2': (40, 0, {}),
             'X': (2, 0, {'highway': 'crossing', 'crossing': 'unmarked', 'crossed_road': 'minor'})}
    result = plan(make_graph(nodes, [('F0', 'X', FOOTWAY), ('X', 'F2', FOOTWAY)]), (0, 1), (40, 1))
    crossing = result.crossings[0]
    assert crossing.start_s == pytest.approx(0.0)  # 2 - 3 clamped to the route start
    assert crossing.end_s == pytest.approx(5.0)


def test_parallel_edges_usable_one_used():
    nodes = {'A': (0, 0, {}), 'B': (10, 0, {})}
    graph = make_graph(nodes, [('A', 'B', {'highway': 'steps'}), ('A', 'B', FOOTWAY)])
    result = plan(graph, (0, 1), (10, 1))
    assert result.length == pytest.approx(10.0)
    assert result.crossings == ()


def test_crossing_of_no_public_road_does_not_force_detour():
    # 'none' -> minor penalty (unknown type: 40): straight costs 140 < 268.
    result = plan(shared_space_graph({'crossed_road': 'none'}), (0, 1), (0, -101),
                  SHARED_SPACE_CONFIG)
    assert result.length == pytest.approx(100.0)
    assert result.crossings[0].crossed_road == 'none'


def test_crossing_without_road_information_still_treated_as_major():
    # No label -> major (unknown type: 300): straight costs 400 > 268, so detour.
    result = plan(shared_space_graph({}), (0, 1), (0, -101), SHARED_SPACE_CONFIG)
    assert result.length == pytest.approx(20 * math.sqrt(2) + 80)


@pytest.mark.parametrize('coords', [
    (math.nan, -0.1749, 51.4988, -0.1749),
    (51.4988, -0.1749, 51.4988, math.inf),
])
def test_invalid_coordinates_rejected(coords):
    planner = RoutePlanner(road_graph(), *ORIGIN)
    with pytest.raises(PlanningError):
        planner.plan(*coords)


@pytest.mark.parametrize('kwargs', [
    {'max_snap_distance': 0.0},
    {'max_snap_distance': -1.0},
    {'max_snap_distance': math.nan},
    {'simplify_tolerance': -0.5},
    {'crossing_half_width': 0.0},
])
def test_invalid_planner_config_rejected(kwargs):
    with pytest.raises(ValueError):
        PlannerConfig(**kwargs)


def test_node_without_coordinates_rejected():
    graph = nx.MultiDiGraph()
    graph.add_node('A')
    with pytest.raises(ValueError, match='numeric x'):
        RoutePlanner(graph, *ORIGIN)
