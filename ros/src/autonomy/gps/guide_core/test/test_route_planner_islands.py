"""Tests for pedestrian islands in guide_core.route_planner.

Test graph (metres east, north of the origin), crossing a major road from
S1 (0, 0) to N1 (0, 18) in one of two ways:

  Two-stage, straight over:  S1 -> C1 (crossing node) -> I1 =island= I2 -> C2 (crossing node) -> N1
      cost 18 m + 2 x 20 (signalled major) + 50 (island) = 108
  Single-stage, detour of D: S1 -> S2 (D, 0) -> C3 (D, 9) -> N2 (D, 18) -> N1
      cost 2D + 18 + 20 = 2D + 38

Both pavements also run 100 m west (to S0, N0), as real pavements join the
rest of the network; otherwise each would itself look like an island.
The island I1-I2 is either a footway=traffic_island way (default) or, as on
Cromwell Road, a plain footway with crossing:island=no on both halves.
"""

from guide_core.geo import enu_to_latlon
from guide_core.route_planner import PlannerConfig, RoutePlanner
from guide_core.walk_costs import CostConfig
import networkx as nx
import pytest

ORIGIN = (51.4988, -0.1749)
CROSSING_NODE = {'highway': 'crossing', 'crossing': 'traffic_signals', 'crossed_road': 'major'}
SIDEWALK = {'highway': 'footway', 'footway': 'sidewalk'}
CROSSING_WAY = {'highway': 'footway', 'footway': 'crossing'}
ISLAND_WAY = {'highway': 'footway', 'footway': 'traffic_island'}


def _add_node(graph, name, east, north, **tags):
    lat, lon = enu_to_latlon(east, north, *ORIGIN)
    graph.add_node(name, x=lon, y=lat, **tags)


def _add_way(graph, nodes, tags):
    """Add a two-way way through nodes (each piece stored in both directions)."""
    for u, v in zip(nodes, nodes[1:]):
        graph.add_edge(u, v, **tags)
        graph.add_edge(v, u, **tags)


def road_graph(detour, island_way=ISLAND_WAY, halves=CROSSING_WAY):
    """Return the two-option graph above, with the single-stage crossing `detour` m east."""
    g = nx.MultiDiGraph()
    _add_node(g, 'S0', -100, 0)
    _add_node(g, 'N0', -100, 18)
    _add_node(g, 'S1', 0, 0)
    _add_node(g, 'C1', 0, 4, **CROSSING_NODE)
    _add_node(g, 'I1', 0, 8)
    _add_node(g, 'I2', 0, 10)
    _add_node(g, 'C2', 0, 14, **CROSSING_NODE)
    _add_node(g, 'N1', 0, 18)
    _add_node(g, 'S2', detour, 0)
    _add_node(g, 'C3', detour, 9, **CROSSING_NODE)
    _add_node(g, 'N2', detour, 18)
    _add_way(g, ['S1', 'C1', 'I1'], halves)
    _add_way(g, ['I1', 'I2'], island_way)
    _add_way(g, ['I2', 'C2', 'N1'], halves)
    _add_way(g, ['S0', 'S1', 'S2'], SIDEWALK)
    _add_way(g, ['N0', 'N1'], SIDEWALK)
    _add_way(g, ['S2', 'C3', 'N2'], CROSSING_WAY)
    _add_way(g, ['N2', 'N1'], SIDEWALK)
    return g


def plan(graph, start, goal, island_penalty=50.0, island_max_length=30.0):
    """Plan between two (east, north) points; return the PlannedRoute."""
    config = PlannerConfig(island_max_length=island_max_length,
                           costs=CostConfig(island_penalty=island_penalty))
    planner = RoutePlanner(graph, *ORIGIN, config)
    return planner.plan(*enu_to_latlon(*start, *ORIGIN), *enu_to_latlon(*goal, *ORIGIN))


def max_east(planned):
    return max(x for x, _ in planned.route.points)


def test_island_nodes_counted():
    assert RoutePlanner(road_graph(20.0), *ORIGIN).island_node_count == 2


def test_short_detour_avoids_island():
    # 2 x 20 + 38 = 78 < 108: the single-stage crossing wins.
    planned = plan(road_graph(20.0), (0, 0), (0, 18))
    assert max_east(planned) == pytest.approx(20.0, abs=0.01)


def test_reverse_direction_also_avoids_island():
    planned = plan(road_graph(20.0), (0, 18), (0, 0))
    assert max_east(planned) == pytest.approx(20.0, abs=0.01)


def test_single_stage_crossing_has_no_island():
    planned = plan(road_graph(20.0), (0, 0), (0, 18))
    assert len(planned.crossings) == 1
    assert planned.crossings[0].has_island is False


def test_without_penalty_uses_island_and_marks_both_halves():
    # Penalty 0: 58 < 78, straight over the island.
    planned = plan(road_graph(20.0), (0, 0), (0, 18), island_penalty=0.0)
    assert max_east(planned) == pytest.approx(0.0, abs=0.01)
    assert [(c.start_s, c.end_s) for c in planned.crossings] == [
        (pytest.approx(0.0, abs=0.01), pytest.approx(8.0, abs=0.01)),
        (pytest.approx(10.0, abs=0.01), pytest.approx(18.0, abs=0.01))]
    assert [c.has_island for c in planned.crossings] == [True, True]


def test_island_charged_once_per_island():
    # Detour 45 m: 128. Charged once the island route costs 108 and wins;
    # charged twice (158) the detour would win. Also: discouraged, not banned.
    planned = plan(road_graph(45.0), (0, 0), (0, 18))
    assert max_east(planned) == pytest.approx(0.0, abs=0.01)


def test_island_tagged_crossing_node():
    # A crossing known only from its node, tagged crossing:island=yes.
    g = nx.MultiDiGraph()
    _add_node(g, 'S', 0, 0)
    _add_node(g, 'C', 0, 9, **CROSSING_NODE, **{'crossing:island': 'yes'})
    _add_node(g, 'N', 0, 18)
    _add_way(g, ['S', 'C', 'N'], SIDEWALK)
    planned = plan(g, (0, 0), (0, 18))
    assert RoutePlanner(g, *ORIGIN).island_node_count == 1
    assert len(planned.crossings) == 1
    assert planned.crossings[0].has_island is True


# --- Islands found from the graph's shape (no island tags) ----------------------

PLAIN_LINK = {'highway': 'footway'}
HALF_NO_ISLAND = dict(CROSSING_WAY, **{'crossing': 'traffic_signals', 'crossing:island': 'no'})


def test_untagged_island_found_from_shape():
    # Cromwell Road mapping: plain 2 m footway between two halves that say
    # crossing:island=no. Still an island: it can only be left over crossings.
    g = road_graph(20.0, island_way=PLAIN_LINK, halves=HALF_NO_ISLAND)
    assert RoutePlanner(g, *ORIGIN).island_node_count == 2
    planned = plan(g, (0, 0), (0, 18), island_penalty=0.0)
    assert [c.has_island for c in planned.crossings] == [True, True]


def test_untagged_island_is_charged():
    # Same as test_short_detour_avoids_island, without any island tags.
    g = road_graph(20.0, island_way=PLAIN_LINK, halves=HALF_NO_ISLAND)
    assert max_east(plan(g, (0, 0), (0, 18))) == pytest.approx(20.0, abs=0.01)


def test_two_halves_meeting_at_one_node_is_island():
    # No link at all: both crossing ways end at the same (non-crossing) node.
    g = nx.MultiDiGraph()
    for name, north in (('S1', 0), ('C1', 4), ('M', 9), ('C2', 14), ('N1', 18)):
        _add_node(g, name, 0, north, **(CROSSING_NODE if name.startswith('C') else {}))
    _add_node(g, 'S0', -100, 0)
    _add_node(g, 'N0', -100, 18)
    _add_way(g, ['S1', 'C1', 'M', 'C2', 'N1'], CROSSING_WAY)
    _add_way(g, ['S0', 'S1'], SIDEWALK)
    _add_way(g, ['N0', 'N1'], SIDEWALK)
    planner = RoutePlanner(g, *ORIGIN)
    assert planner.island_node_count == 1
    # The kerb-to-kerb run is split at the island node: two crossings.
    planned = plan(g, (0, 0), (0, 18))
    assert [(c.start_s, c.end_s) for c in planned.crossings] == [
        (pytest.approx(0.0, abs=0.01), pytest.approx(9.0, abs=0.01)),
        (pytest.approx(9.0, abs=0.01), pytest.approx(18.0, abs=0.01))]
    assert [c.has_island for c in planned.crossings] == [True, True]


def test_pavement_corner_is_not_island():
    # Two crossings whose kerbs K1, K2 are 2 m apart, but K1 also joins the
    # pavement: the corner can be left on foot, so it is no island.
    g = nx.MultiDiGraph()
    _add_node(g, 'A', 0, -10)
    _add_node(g, 'CA', 0, -5, **CROSSING_NODE)
    _add_node(g, 'K1', 0, 0)
    _add_node(g, 'K2', 2, 0)
    _add_node(g, 'CB', 7, 0, **CROSSING_NODE)
    _add_node(g, 'B', 12, 0)
    _add_node(g, 'P', 0, 100)
    _add_way(g, ['A', 'CA', 'K1'], CROSSING_WAY)
    _add_way(g, ['K2', 'CB', 'B'], CROSSING_WAY)
    _add_way(g, ['P', 'K1', 'K2'], SIDEWALK)
    assert RoutePlanner(g, *ORIGIN).island_node_count == 0


def test_long_walkway_piece_is_not_island():
    g = road_graph(20.0, island_way=PLAIN_LINK, halves=HALF_NO_ISLAND)
    planner = RoutePlanner(g, *ORIGIN, PlannerConfig(island_max_length=1.0))  # link is 2 m
    assert planner.island_node_count == 0


@pytest.mark.parametrize('value', [-1.0, float('nan')])
def test_invalid_island_max_length_rejected(value):
    with pytest.raises(ValueError):
        PlannerConfig(island_max_length=value)
