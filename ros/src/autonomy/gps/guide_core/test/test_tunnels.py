"""Tests for tunnels: extra cost in walk_costs, surface-only snapping in route_planner.

Planner graph (metres east, north of the origin), from A0 (0, 10) to B0 (0, -110):

  Tunnel, straight:  A0 - A (0, 0) =tunnel= T (0, -50) =tunnel= B (0, -100) - B0
      tunnel part 100 m, cost 100 x tunnel_factor
  Surface detour:    A - M (east, -50) - B, 2 x sqrt(east^2 + 50^2) m
"""

from guide_core.geo import enu_to_latlon
from guide_core.route_planner import PlannerConfig, RoutePlanner
from guide_core.walk_costs import CostConfig, edge_cost
import networkx as nx
import pytest

ORIGIN = (51.4988, -0.1749)
FOOTWAY = {'highway': 'footway'}
TUNNEL = {'highway': 'footway', 'tunnel': 'yes'}


# --- walk_costs -------------------------------------------------------------------

def test_tunnel_factor_default():
    assert CostConfig().tunnel_factor == 3.0


def test_tunnel_footway_costs_three_times():
    assert edge_cost(10.0, TUNNEL, {}, {}) == pytest.approx(30.0)


def test_building_passage_not_affected():
    edge = {'highway': 'footway', 'tunnel': 'building_passage'}
    assert edge_cost(10.0, edge, {}, {}) == pytest.approx(10.0)


def test_tunnel_road_multiplies_both_factors():
    edge = {'highway': 'residential', 'tunnel': 'yes'}
    assert edge_cost(10.0, edge, {}, {}) == pytest.approx(10 * 10 * 3)


def test_tunnel_factor_below_one_rejected():
    with pytest.raises(ValueError):
        CostConfig(tunnel_factor=0.5)


# --- route_planner ----------------------------------------------------------------

def _graph(detour_east):
    g = nx.MultiDiGraph()

    def node(name, east, north):
        lat, lon = enu_to_latlon(east, north, *ORIGIN)
        g.add_node(name, x=lon, y=lat)

    def way(nodes, tags):
        for u, v in zip(nodes, nodes[1:]):
            g.add_edge(u, v, **tags)
            g.add_edge(v, u, **tags)

    for name, east, north in (('A0', 0, 10), ('A', 0, 0), ('T', 0, -50), ('B', 0, -100),
                              ('B0', 0, -110), ('M', detour_east, -50)):
        node(name, east, north)
    way(['A0', 'A'], FOOTWAY)
    way(['B', 'B0'], FOOTWAY)
    way(['A', 'T', 'B'], TUNNEL)
    way(['A', 'M', 'B'], FOOTWAY)
    return g


def _plan(graph, goal=(0, -110), tunnel_factor=3.0):
    config = PlannerConfig(costs=CostConfig(tunnel_factor=tunnel_factor))
    planner = RoutePlanner(graph, *ORIGIN, config)
    return planner.plan(*enu_to_latlon(0, 10, *ORIGIN), *enu_to_latlon(*goal, *ORIGIN))


def _max_east(planned):
    return max(x for x, _ in planned.route.points)


def test_similar_surface_route_preferred():
    # Surface 107.7 m vs tunnel 100 m x 3 = 300: stays on the surface.
    assert _max_east(_plan(_graph(20.0))) == pytest.approx(20.0, abs=0.01)


def test_tunnel_used_without_factor():
    assert _max_east(_plan(_graph(20.0), tunnel_factor=1.0)) == pytest.approx(0.0, abs=0.01)


def test_tunnel_used_when_surface_much_longer():
    # Surface 2 x sqrt(200^2 + 50^2) = 412 m > 300: the tunnel wins.
    assert _max_east(_plan(_graph(200.0))) == pytest.approx(0.0, abs=0.01)


def test_goal_may_be_underground():
    # Goal right on the tunnel node T (e.g. a station reached by a subway):
    # snapped there, reached through the tunnel, with a no-GPS warning.
    planned = _plan(_graph(20.0), goal=(0, -50))
    assert planned.goal_snap_distance == pytest.approx(0.0, abs=0.01)
    assert planned.route.points[-1] == pytest.approx((0.0, -50.0), abs=0.01)
    assert any('50 m of tunnel' in w for w in planned.warnings)


def test_start_never_underground():
    # Start right on T: the robot's GPS position is on the surface, so it
    # snaps to the nearest surface walkway node, M (20 m).
    planner = RoutePlanner(_graph(20.0), *ORIGIN)
    planned = planner.plan(*enu_to_latlon(0, -50, *ORIGIN), *enu_to_latlon(0, -110, *ORIGIN))
    assert planned.start_snap_distance == pytest.approx(20.0, abs=0.01)


def test_surface_route_has_no_tunnel_warning():
    assert _plan(_graph(20.0)).warnings == ()


def test_tunnel_nodes_not_start_snap_targets():
    # A, T, B touch tunnel edges: only A0, B0, M can be a start.
    assert RoutePlanner(_graph(20.0), *ORIGIN).usable_node_count == 3
