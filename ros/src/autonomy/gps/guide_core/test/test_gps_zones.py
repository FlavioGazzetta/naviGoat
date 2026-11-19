"""Tests for the simulator's GPS zone map."""

from guide_core.geo import latlon_to_enu
from guide_core.gps_error import PROFILES
from guide_core.gps_zones import (
    GpsZone, GpsZoneMap, SegmentIndex, surface_segments, TUNNEL_ZONE_NAME, tunnel_zones,
    zone_map_from_dict)
import networkx as nx
import pytest

ORIGIN = (51.4988, -0.1749)
CANYON = PROFILES['urban_canyon']
NO_FIX = PROFILES['no_fix']
OPEN = PROFILES['open']


class FakeLineString:
    """Stand-in for a shapely LineString (only .coords is used)."""

    def __init__(self, coords):
        self.coords = coords


def lonlat(east, north):
    """Return (lon, lat) of an ENU point near the origin (inverse of latlon_to_enu)."""
    lat0, lon0 = ORIGIN
    e1, n1 = latlon_to_enu(lat0 + 0.001, lon0 + 0.001, lat0, lon0)
    return lon0 + 0.001 * east / e1, lat0 + 0.001 * north / n1


def graph_with(edges, as_strings=False):
    """Return a MultiDiGraph with nodes 1..4 on a 100 m east line and the given edges."""
    g = nx.MultiDiGraph()
    for node, east in ((1, 0.0), (2, 100.0), (3, 200.0), (4, 300.0)):
        lon, lat = lonlat(east, 0.0)
        g.add_node(node, x=str(lon) if as_strings else lon, y=str(lat) if as_strings else lat)
    for u, v, data in edges:
        g.add_edge(u, v, **data)
    return g


def test_circle_zone_contains_points_within_radius():
    zone = GpsZone('c', CANYON, ((10.0, 10.0),), 5.0)
    assert zone.contains(13.0, 14.0)
    assert not zone.contains(14.0, 14.0)


def test_corridor_zone_follows_its_segments_with_round_ends():
    zone = GpsZone('c', CANYON, ((0.0, 0.0), (100.0, 0.0), (100.0, 100.0)), 5.0)
    assert zone.contains(50.0, 4.9)
    assert zone.contains(104.0, 60.0)
    assert not zone.contains(50.0, 5.1)
    assert zone.contains(-3.0, 0.0)
    assert not zone.contains(-6.0, 0.0)
    assert not zone.contains(50.0, 50.0)


@pytest.mark.parametrize('points, half_width', [((), 5.0), (((0.0, 0.0),), -1.0)])
def test_invalid_zone_raises(points, half_width):
    with pytest.raises(ValueError):
        GpsZone('bad', CANYON, points, half_width)


def test_default_applies_outside_all_zones():
    zone_map = GpsZoneMap(OPEN, [GpsZone('c', CANYON, ((0.0, 0.0),), 5.0)])
    assert zone_map.conditions_at(20.0, 0.0) == (OPEN, '')
    assert zone_map.conditions_at(1.0, 0.0) == (CANYON, 'c')


def test_no_fix_wins_over_overlapping_zone():
    zone_map = GpsZoneMap(OPEN, [GpsZone('tunnel', NO_FIX, ((0.0, 0.0),), 5.0),
                                 GpsZone('canyon', CANYON, ((0.0, 0.0),), 50.0)])
    assert zone_map.conditions_at(0.0, 0.0) == (NO_FIX, 'tunnel')
    assert zone_map.conditions_at(20.0, 0.0) == (CANYON, 'canyon')


def test_larger_reported_sigma_wins_between_fix_zones():
    zone_map = GpsZoneMap(OPEN, [GpsZone('canyon', CANYON, ((0.0, 0.0),), 50.0),
                                 GpsZone('open', OPEN, ((0.0, 0.0),), 50.0)])
    assert zone_map.conditions_at(0.0, 0.0)[1] == 'canyon'


def test_tunnel_zones_follow_wkt_geometry_and_string_coordinates():
    lon0, lat0 = lonlat(100.0, 0.0)
    lon1, lat1 = lonlat(150.0, 40.0)
    lon2, lat2 = lonlat(200.0, 0.0)
    wkt = f'LINESTRING ({lon0} {lat0}, {lon1} {lat1}, {lon2} {lat2})'
    g = graph_with([(2, 3, {'tunnel': 'yes', 'geometry': wkt})], as_strings=True)
    zones = tunnel_zones(g, *ORIGIN)
    assert len(zones) == 1
    assert zones[0].name == TUNNEL_ZONE_NAME
    assert zones[0].conditions == NO_FIX
    assert zones[0].contains(150.0, 38.0)
    assert not zones[0].contains(150.0, 0.0)


def test_tunnel_zones_accept_shapely_style_geometry():
    coords = [lonlat(100.0, 0.0), lonlat(150.0, 40.0), lonlat(200.0, 0.0)]
    g = graph_with([(2, 3, {'tunnel': 'yes', 'geometry': FakeLineString(coords)})])
    assert tunnel_zones(g, *ORIGIN)[0].contains(150.0, 38.0)


def test_tunnel_zones_without_geometry_run_node_to_node():
    g = graph_with([(1, 2, {'tunnel': 'yes'})])
    zone = tunnel_zones(g, *ORIGIN, half_width=4.0)[0]
    assert zone.contains(50.0, 3.9)
    assert not zone.contains(50.0, 4.1)
    assert zone.half_width == 4.0


def test_only_tunnel_yes_edges_become_zones():
    g = graph_with([(1, 2, {'tunnel': 'building_passage'}), (2, 3, {'highway': 'footway'}),
                    (3, 4, {'tunnel': 'yes'})])
    zones = tunnel_zones(g, *ORIGIN)
    assert len(zones) == 1
    assert zones[0].contains(250.0, 0.0)


def test_both_directions_of_a_tunnel_give_one_zone():
    g = graph_with([(1, 2, {'tunnel': 'yes', 'osmid': '42'}),
                    (2, 1, {'tunnel': 'yes', 'osmid': '42'})])
    assert len(tunnel_zones(g, *ORIGIN)) == 1


def test_zone_map_from_dict_builds_default_tunnels_and_extra_zones():
    g = graph_with([(3, 4, {'tunnel': 'yes'})])
    lon, lat = lonlat(0.0, 0.0)
    zone_map = zone_map_from_dict({
        'default': {'profile': 'urban_canyon'},
        'tunnels_from_graph': True,
        'tunnel_half_width': 6.0,
        'zones': [{'name': 'open square', 'conditions': {'profile': 'open'},
                   'points': [[lat, lon]], 'half_width': 30.0}],
    }, *ORIGIN, graph=g)
    assert zone_map.conditions_at(100.0, 0.0) == (CANYON, '')
    assert zone_map.conditions_at(250.0, 5.9) == (NO_FIX, TUNNEL_ZONE_NAME)
    assert zone_map.conditions_at(10.0, 10.0) == (OPEN, 'open square')


def test_zone_map_from_dict_defaults_to_perfect_without_zones():
    zone_map = zone_map_from_dict({}, *ORIGIN)
    assert zone_map.conditions_at(0.0, 0.0) == (PROFILES['perfect'], '')
    assert zone_map.zones == ()


def test_zone_map_from_dict_ignores_graph_unless_asked():
    g = graph_with([(1, 2, {'tunnel': 'yes'})])
    assert zone_map_from_dict({}, *ORIGIN, graph=g).zones == ()


def test_tunnels_from_graph_needs_a_graph():
    with pytest.raises(ValueError):
        zone_map_from_dict({'tunnels_from_graph': True}, *ORIGIN)


@pytest.mark.parametrize('values', [
    {'defualt': {}},
    {'zones': [{'name': 'x', 'points': [[51.5, -0.17]], 'radius': 5.0}]},
])
def test_zone_map_from_dict_rejects_unknown_keys(values):
    with pytest.raises(ValueError):
        zone_map_from_dict(values, *ORIGIN)


# --- Surface check: a street above a subway keeps GPS ---------------------------

def street_over_subway():
    """Return a graph: subway along y=0 (x 100..300), pavement along y=4 above it."""
    g = graph_with([(2, 4, {'tunnel': 'yes', 'osmid': '1'})])
    for node, east in ((5, 100.0), (6, 300.0)):
        lon, lat = lonlat(east, 4.0)
        g.add_node(node, x=lon, y=lat)
    g.add_edge(5, 6, highway='footway', osmid='2')
    g.add_edge(6, 5, highway='footway', osmid='2')
    return g


def test_segment_index_finds_the_nearest_segment_within_range():
    index = SegmentIndex([((0.0, 0.0), (100.0, 0.0)), ((0.0, 30.0), (100.0, 30.0))])
    assert index.count == 2
    assert index.nearest_distance(50.0, 4.0, 10.0) == pytest.approx(4.0)
    assert index.nearest_distance(50.0, 26.0, 10.0) == pytest.approx(4.0)
    assert index.nearest_distance(50.0, 15.0, 10.0) == float('inf')
    assert index.nearest_distance(-3.0, 0.0, 5.0) == pytest.approx(3.0)


def test_surface_segments_skip_tunnels_and_duplicates():
    segments = surface_segments(street_over_subway(), *ORIGIN)
    assert len(segments) == 1
    (a, b), = segments
    assert a[1] == pytest.approx(4.0, abs=0.01) and b[1] == pytest.approx(4.0, abs=0.01)


def test_pavement_above_a_subway_keeps_gps_and_the_subway_loses_it():
    zone_map = zone_map_from_dict({'default': {'profile': 'open'}, 'tunnels_from_graph': True},
                                  *ORIGIN, graph=street_over_subway())
    assert zone_map.surface is not None
    assert zone_map.conditions_at(200.0, 3.5) == (OPEN, '')                 # on the pavement
    assert zone_map.conditions_at(200.0, 0.5) == (NO_FIX, TUNNEL_ZONE_NAME)  # in the subway


def test_surface_check_can_be_switched_off():
    zone_map = zone_map_from_dict({'tunnels_from_graph': True, 'tunnel_surface_check': False},
                                  *ORIGIN, graph=street_over_subway())
    assert zone_map.surface is None
    assert zone_map.conditions_at(200.0, 3.5)[1] == TUNNEL_ZONE_NAME


def test_only_tunnel_zones_yield_to_the_surface():
    surface = SegmentIndex([((0.0, 3.0), (100.0, 3.0))])
    canyon = GpsZone('canyon', CANYON, ((0.0, 0.0), (100.0, 0.0)), 10.0)
    zone_map = GpsZoneMap(OPEN, [canyon], surface)
    assert zone_map.conditions_at(50.0, 2.9) == (CANYON, 'canyon')


def test_a_tunnel_with_no_surface_nearby_still_has_no_fix():
    tunnel = GpsZone(TUNNEL_ZONE_NAME, NO_FIX, ((0.0, 0.0), (100.0, 0.0)), 5.0,
                     yields_to_surface=True)
    surface = SegmentIndex([((0.0, 50.0), (100.0, 50.0))])
    assert GpsZoneMap(OPEN, [tunnel], surface).conditions_at(50.0, 2.0)[1] == TUNNEL_ZONE_NAME
