"""Tests for guide_core.geocoding: Nominatim request parameters and parsing.

The test/data/nominatim_*.json files are real responses saved once (15 Nov
2026) with the project User-Agent. These tests never use the network.
Search results (c) OpenStreetMap contributors, ODbL.
"""

import json
import math
from pathlib import Path

from guide_core.geo import latlon_to_enu
from guide_core.geocoding import (
    ATTRIBUTION, BBox, bbox_from_meta, GeocodeConfig, GeocodeError, is_large_area,
    labels, parse_results, search_params, searchable_query)
import pytest

DATA = Path(__file__).parent / 'data'
ORIGIN = (51.4988, -0.1749)
GRAPH_BBOX = BBox.around(*ORIGIN, 1500.0)   # the area of south_ken_walk.graphml
SCIENCE_MUSEUM = (51.4973983, -0.1746726)


def load(name):
    with open(DATA / f'nominatim_{name}.json', encoding='utf-8') as f:
        return json.load(f)


def entry(name='Test Place', lat='51.4974', lon='-0.1747', **extra):
    """Build a minimal Nominatim result."""
    result = {'osm_type': 'node', 'osm_id': 1, 'lat': lat, 'lon': lon,
              'category': 'amenity', 'type': 'cafe', 'name': name,
              'display_name': f'{name}, Exhibition Road, London'}
    result.update(extra)
    return result


def metres(a, b):
    x, y = latlon_to_enu(b[0], b[1], a[0], a[1])
    return math.hypot(x, y)


# --- BBox -------------------------------------------------------------------

@pytest.mark.parametrize('values', [
    (51.5, -0.2, 51.4, -0.1),           # min_lat > max_lat
    (51.4, -0.1, 51.5, -0.2),           # min_lon > max_lon
    (float('nan'), -0.2, 51.5, -0.1),   # not finite
    (-91.0, -0.2, 51.5, -0.1),          # latitude out of range
])
def test_bbox_rejects_bad_values(values):
    with pytest.raises(GeocodeError):
        BBox(*values)


def test_bbox_around_is_square_of_given_half_width():
    box = GRAPH_BBOX
    west_east = metres((ORIGIN[0], box.min_lon), (ORIGIN[0], box.max_lon))
    south_north = metres((box.min_lat, ORIGIN[1]), (box.max_lat, ORIGIN[1]))
    assert west_east == pytest.approx(3000.0, abs=1.0)
    assert south_north == pytest.approx(3000.0, abs=1.0)
    assert box.centre == pytest.approx(ORIGIN, abs=1e-6)


def test_bbox_contains_edges_but_not_outside():
    box = BBox(51.0, -1.0, 52.0, 1.0)
    assert box.contains(51.0, -1.0) and box.contains(52.0, 1.0) and box.contains(51.5, 0.0)
    assert not box.contains(52.0001, 0.0) and not box.contains(51.5, -1.0001)


def test_bbox_shrunk_moves_every_edge_inwards_by_margin():
    big, small = GRAPH_BBOX, GRAPH_BBOX.shrunk(30.0)
    lat0, lon0 = ORIGIN
    moved = [metres((big.min_lat, lon0), (small.min_lat, lon0)),
             metres((big.max_lat, lon0), (small.max_lat, lon0)),
             metres((lat0, big.min_lon), (lat0, small.min_lon)),
             metres((lat0, big.max_lon), (lat0, small.max_lon))]
    assert moved == pytest.approx([30.0] * 4, abs=0.1)


def test_bbox_shrunk_too_much_raises():
    with pytest.raises(GeocodeError):
        BBox.around(*ORIGIN, 20.0).shrunk(30.0)


def test_viewbox_is_left_top_right_bottom():
    assert BBox(51.49, -0.185, 51.505, -0.165).viewbox() == \
        '-0.1850000,51.5050000,-0.1650000,51.4900000'


def test_bbox_from_meta_uses_origin_and_dist():
    # The real south_ken_walk.meta.yaml (fields that matter here).
    meta = {'origin_lat': 51.4988, 'origin_lon': -0.1749, 'dist_m': 1500.0,
            'network_type': 'walk'}
    assert bbox_from_meta(meta) == GRAPH_BBOX


def test_bbox_from_meta_explicit_bbox_wins():
    meta = {'origin_lat': 51.4988, 'origin_lon': -0.1749, 'dist_m': 1500.0,
            'bbox': {'min_lat': 51.49, 'min_lon': -0.18, 'max_lat': 51.50, 'max_lon': -0.17}}
    assert bbox_from_meta(meta) == BBox(51.49, -0.18, 51.50, -0.17)


@pytest.mark.parametrize('meta', [
    {'origin_lat': 51.4988, 'origin_lon': -0.1749},            # no dist_m
    {'bbox': {'min_lat': 51.49, 'min_lon': -0.18, 'max_lat': 51.50}},  # bbox incomplete
    'not a mapping',
])
def test_bbox_from_meta_rejects_incomplete(meta):
    with pytest.raises(GeocodeError):
        bbox_from_meta(meta)


# --- config and requests -----------------------------------------------------

@pytest.mark.parametrize('kwargs', [
    {'max_results': 0}, {'edge_margin': -1.0}, {'min_query_length': 0},
])
def test_config_rejects_bad_values(kwargs):
    with pytest.raises(GeocodeError):
        GeocodeConfig(**kwargs)


@pytest.mark.parametrize('query, expected', [
    ('  science   museum ', 'science museum'),   # spaces tidied, case kept
    ('V&A', 'V&A'),                              # normalises to 'v and a': long enough
    ('ab', ''),                                  # too short
    ('the a', ''),                               # leading 'the' dropped: 'a'
    ('?!', ''),                                  # punctuation only
])
def test_searchable_query(query, expected):
    assert searchable_query(query) == expected


def test_search_params_send_only_query_and_graph_area():
    params = search_params(' Science  Museum ', GRAPH_BBOX)
    assert params == {
        'q': 'Science Museum', 'format': 'jsonv2', 'limit': '5', 'addressdetails': '1',
        'accept-language': 'en', 'viewbox': GRAPH_BBOX.viewbox(), 'bounded': '1'}


def test_search_params_refuse_too_short_query():
    with pytest.raises(GeocodeError):
        search_params('ab', GRAPH_BBOX)


# --- parsing real responses --------------------------------------------------

def test_parse_empty_response():
    assert parse_results(load('empty'), GRAPH_BBOX).places == ()


def test_parse_science_museum():
    result = parse_results(load('science_museum'), GRAPH_BBOX)
    assert (result.outside, result.invalid, result.duplicates) == (0, 0, 0)
    (place,) = result.places
    assert place.name == 'Science Museum'
    assert (place.lat, place.lon) == SCIENCE_MUSEUM
    assert place.road == 'Frankland Road'
    assert place.osm_ref == 'way/27765411'
    assert (place.category, place.kind) == ('tourism', 'museum')
    assert place.size == pytest.approx(291.6, abs=2.0)   # 0.0042 deg of longitude
    assert is_large_area(place)


def test_parse_imperial_college_is_a_large_relation():
    (place,) = parse_results(load('imperial_college'), GRAPH_BBOX).places
    assert place.name == 'Imperial College London'
    assert place.osm_ref == 'relation/12233221'
    assert (place.category, place.kind) == ('amenity', 'university')
    assert place.size == pytest.approx(551.6, abs=3.0)
    assert is_large_area(place)


def test_parse_museum_keeps_nominatim_order():
    places = parse_results(load('museum'), GRAPH_BBOX).places
    assert [p.name for p in places] == [
        'Natural History Museum', 'Victoria and Albert Museum', 'Science Museum',
        'Polish Institute and Sikorski Museum', "The Clockmakers' Museum"]


def test_parse_small_places_are_not_large_areas():
    places = {p.name: p for p in parse_results(load('museum'), GRAPH_BBOX).places}
    assert not is_large_area(places['Polish Institute and Sikorski Museum'])  # about 32 m
    assert not is_large_area(places["The Clockmakers' Museum"])               # a node
    assert is_large_area(places['Natural History Museum'])


def test_parse_drops_places_outside_the_graph_area():
    # 300 m square around the Science Museum: the Polish Institute (~450 m north) is out.
    box = BBox.around(*SCIENCE_MUSEUM, 300.0)
    result = parse_results(load('museum'), box)
    assert 'Polish Institute and Sikorski Museum' not in [p.name for p in result.places]
    assert len(result.places) == 4 and result.outside == 1


def test_parse_drops_places_within_edge_margin():
    # The box's north edge is 10 m north of the place: inside the box, but
    # within the 30 m edge margin, so dropped; with no margin it is kept.
    box = BBox.around(*SCIENCE_MUSEUM, 100.0)
    data = [entry(lat=str(box.max_lat - 10.0 / 111195.0))]
    assert parse_results(data, box).outside == 1
    assert len(parse_results(data, box, GeocodeConfig(edge_margin=0.0)).places) == 1


def test_parse_keeps_at_most_max_results():
    places = parse_results(load('museum'), GRAPH_BBOX, GeocodeConfig(max_results=2)).places
    assert [p.name for p in places] == ['Natural History Museum', 'Victoria and Albert Museum']


@pytest.mark.parametrize('data', [{'error': 'Unable to geocode'}, None, 'text'])
def test_parse_rejects_a_response_that_is_not_a_list(data):
    with pytest.raises(GeocodeError):
        parse_results(data, GRAPH_BBOX)


def test_parse_skips_malformed_entries():
    data = [
        'not a mapping',
        entry(lat=None),
        entry(lon='east'),
        entry(lat='95.0'),
        entry(name='', display_name='', address={}),   # nothing to call it
        entry(name='Good Cafe'),
    ]
    result = parse_results(data, GRAPH_BBOX)
    assert [p.name for p in result.places] == ['Good Cafe']
    assert result.invalid == 5


def test_parse_merges_the_same_place_returned_twice():
    data = [entry(name='Royal College of Art', osm_type='way', osm_id=1),
            entry(name='royal college of art', osm_type='relation', osm_id=2,
                  lat='51.49745')]   # about 6 m away
    result = parse_results(data, GRAPH_BBOX)
    assert [p.osm_ref for p in result.places] == ['way/1']
    assert result.duplicates == 1


def test_parse_keeps_places_with_the_same_name_far_apart():
    data = [entry(name='Starbucks', lat='51.4974'), entry(name='Starbucks', lat='51.4994')]
    assert len(parse_results(data, GRAPH_BBOX).places) == 2


def test_name_from_house_number_and_road_when_unnamed():
    data = [entry(name='', category='building', type='house',
                  address={'house_number': '20', 'road': 'Princes Gate'})]
    (place,) = parse_results(data, GRAPH_BBOX).places
    assert place.name == '20 Princes Gate' and place.road == 'Princes Gate'


def test_name_from_display_name_when_nothing_else():
    data = [entry(name='', display_name='Queens Lawn, Imperial College, London')]
    assert parse_results(data, GRAPH_BBOX).places[0].name == 'Queens Lawn'


def test_size_is_zero_without_a_usable_bounding_box():
    data = [entry(name='A Cafe'), entry(name='B Cafe', boundingbox=['x', '1', '2', '3'])]
    assert [p.size for p in parse_results(data, GRAPH_BBOX).places] == [0.0, 0.0]


# --- spoken labels and attribution --------------------------------------------

def test_labels_are_names_when_names_differ():
    places = parse_results(load('museum'), GRAPH_BBOX).places
    assert labels(places) == tuple(p.name for p in places)


def test_labels_add_the_road_when_names_repeat():
    data = [entry(name='Starbucks', lat='51.4974', address={'road': 'Exhibition Road'}),
            entry(name='Starbucks', lat='51.4994', address={'road': 'Gloucester Road'}),
            entry(name='Pret', lat='51.4984')]
    places = parse_results(data, GRAPH_BBOX).places
    assert labels(places) == ('Starbucks, Exhibition Road', 'Starbucks, Gloucester Road', 'Pret')


def test_attribution_credits_openstreetmap_and_odbl():
    assert 'OpenStreetMap contributors' in ATTRIBUTION and 'ODbL' in ATTRIBUTION
