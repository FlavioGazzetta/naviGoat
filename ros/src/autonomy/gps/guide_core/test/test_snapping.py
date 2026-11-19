"""Tests for start/goal snapping (walkways first, roads as a fallback) and plan warnings."""

from guide_core.geo import enu_to_latlon
from guide_core.route_planner import PlannerConfig, RoutePlanner
import networkx as nx
import pytest

ORIGIN = (51.4988, -0.1749)
FOOTWAY = {'highway': 'footway'}
SIDEWALK = {'highway': 'footway', 'footway': 'sidewalk'}
CROSSING_WAY = {'highway': 'footway', 'footway': 'crossing'}
ROAD = {'highway': 'residential'}


def _graph(nodes, ways):
    """nodes: {name: (east, north, tags)}; ways: [(node names, tags)], two-way."""
    g = nx.MultiDiGraph()
    for name, (east, north, tags) in nodes.items():
        lat, lon = enu_to_latlon(east, north, *ORIGIN)
        g.add_node(name, x=lon, y=lat, **tags)
    for names, tags in ways:
        for u, v in zip(names, names[1:]):
            g.add_edge(u, v, **tags)
            g.add_edge(v, u, **tags)
    return g


def _plan(graph, start, goal, config=None):
    planner = RoutePlanner(graph, *ORIGIN, config)
    return planner.plan(*enu_to_latlon(*start, *ORIGIN), *enu_to_latlon(*goal, *ORIGIN))


def street():
    """Return a pavement S-G (40 m) and, 10 m south, a road R1-R2 joined to it by roads."""
    nodes = {'S': (0, 0, {}), 'G': (40, 0, {}), 'R1': (0, -10, {}), 'R2': (40, -10, {})}
    ways = [(['S', 'G'], SIDEWALK), (['S', 'R1', 'R2', 'G'], ROAD)]
    return _graph(nodes, ways)


def test_walkway_preferred_over_nearer_road_node():
    # Goal 1 m from road node R2 but 9 m from pavement node G: snaps to G.
    planned = _plan(street(), (0, 1), (40, -9))
    assert planned.goal_snap_distance == pytest.approx(9.0, abs=0.01)
    assert planned.route.points[-1] == pytest.approx((40.0, 0.0), abs=0.01)
    assert planned.warnings == ()


def test_falls_back_to_road_with_warning():
    planned = _plan(street(), (0, 1), (40, -9), PlannerConfig(walkway_snap_distance=5.0))
    assert planned.goal_snap_distance == pytest.approx(1.0, abs=0.01)
    assert any('snapped 1.0 m to a road' in w for w in planned.warnings)


def test_crossing_node_is_not_a_snap_target():
    nodes = {'S0': (0, -20, {}), 'S': (0, 0, {}), 'C': (0, 5, {'highway': 'crossing'}),
             'N': (0, 10, {}), 'N0': (0, 30, {})}
    ways = [(['S0', 'S'], SIDEWALK), (['S', 'C', 'N'], CROSSING_WAY), (['N', 'N0'], SIDEWALK)]
    planned = _plan(_graph(nodes, ways), (0, -20), (0, 5))   # goal exactly on C
    assert planned.goal_snap_distance == pytest.approx(5.0, abs=0.01)


def test_long_detour_warned():
    # Start and goal 20 m apart, but the only path goes 400 m east and back.
    nodes = {'A': (0, 0, {}), 'B': (400, 0, {}), 'C': (400, -20, {}), 'D': (0, -20, {})}
    planned = _plan(_graph(nodes, [(['A', 'B', 'C', 'D'], FOOTWAY)]), (0, 0), (0, -20))
    assert planned.length == pytest.approx(820.0)
    assert any('41.0x the straight-line distance' in w for w in planned.warnings)


def test_normal_plan_has_no_warnings():
    assert _plan(street(), (0, 1), (40, 1)).warnings == ()


@pytest.mark.parametrize('kwargs', [
    {'walkway_snap_distance': 0.0},
    {'detour_warning_ratio': 0.5},
    {'detour_warning_extra': -1.0},
])
def test_invalid_snap_and_warning_config_rejected(kwargs):
    with pytest.raises(ValueError):
        PlannerConfig(**kwargs)


def test_goal_snaps_to_best_not_nearest_walkway_node():
    # Goal point P (0, -5). Pavement node N is 5 m away but only reachable by
    # an 830 m loop; pavement node G is 15 m away (well inside the 5 + 15 m
    # slack) on the direct pavement.
    # N: 830 + 30 x 5 = 980; G: 29.2 + 30 x 15 = 479.2 -> G.
    nodes = {'S': (0, 20, {}), 'G': (15, -5, {}), 'N': (0, -10, {}),
             'L1': (400, 20, {}), 'L2': (400, -10, {})}
    ways = [(['S', 'G'], SIDEWALK), (['S', 'L1', 'L2', 'N'], SIDEWALK)]
    planned = _plan(_graph(nodes, ways), (0, 20), (0, -5))
    assert planned.goal_snap_distance == pytest.approx(15.0, abs=0.01)
    assert planned.length == pytest.approx(29.15, abs=0.01)   # S to G: sqrt(15^2 + 25^2)


def test_goal_does_not_stop_short_to_save_a_little():
    # Same layout, but N is reached by a short path:
    # N: 30 + 30 x 5 = 180; G: 29.2 + 30 x 15 = 479.2 -> the nearer node N.
    nodes = {'S': (0, 20, {}), 'G': (15, -5, {}), 'N': (0, -10, {}), 'K': (0, 0, {})}
    ways = [(['S', 'G'], SIDEWALK), (['S', 'K'], SIDEWALK), (['K', 'N'], SIDEWALK)]
    planned = _plan(_graph(nodes, ways), (0, 20), (0, -5))
    assert planned.goal_snap_distance == pytest.approx(5.0, abs=0.01)


@pytest.mark.parametrize('kwargs', [{'goal_snap_weight': -1.0}, {'goal_snap_slack': -1.0}])
def test_invalid_goal_snap_config_rejected(kwargs):
    with pytest.raises(ValueError):
        PlannerConfig(**kwargs)


def test_goal_never_moves_across_the_road():
    # Goal on the south pavement S0; the north pavement node N0 is 20 m away
    # (outside the 15 m slack) and would save a 300 crossing: not chosen.
    nodes = {'N0': (0, 10, {}), 'N1': (50, 10, {}), 'S0': (0, -10, {}), 'S1': (50, -10, {}),
             'C': (50, 0, {'highway': 'crossing', 'crossing': 'uncontrolled',
                           'crossed_road': 'major'})}
    ways = [(['N0', 'N1'], SIDEWALK), (['S0', 'S1'], SIDEWALK), (['N1', 'C', 'S1'], CROSSING_WAY)]
    planned = _plan(_graph(nodes, ways), (0, 12), (0, -10))
    assert planned.route.points[-1] == pytest.approx((0.0, -10.0), abs=0.01)
