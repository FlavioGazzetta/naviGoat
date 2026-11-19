"""Tests for guide_core.geocode_cache: the on-disk cache of online searches.

Every test uses a temporary folder (pytest's tmp_path); the real cache in
~/.local/share/navigoat is never touched.
"""

import json
from pathlib import Path

from guide_core.geocode_cache import (
    CACHE_VERSION, CacheConfig, DAY, GeocodeCache, place_from_dict, place_to_dict)
from guide_core.geocoding import BBox, parse_results, Place
import pytest

NOW = 1794750660.0   # 15 Nov 2026 13:51 UTC
DATA = Path(__file__).parent / 'data'
GRAPH_BBOX = BBox.around(51.4988, -0.1749, 1500.0)
MUSEUM = Place(name='Science Museum', lat=51.4973983, lon=-0.1746726, road='Frankland Road',
               osm_type='way', osm_id=27765411, category='tourism', kind='museum', size=291.6)


def make_cache(tmp_path, **kwargs):
    cache = GeocodeCache(tmp_path / 'cache' / 'geocode_cache.json', **kwargs)
    assert cache.load() == ()
    return cache


def write_file(tmp_path, document):
    path = tmp_path / 'cache' / 'geocode_cache.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    text = document if isinstance(document, str) else json.dumps(document)
    path.write_text(text, encoding='utf-8')
    return path


def v1_document(**entries):
    return {'version': 1, 'provider': 'nominatim', 'attribution': 'x', 'entries': entries}


def raw_entry(query='Science Museum', places=None):
    return {'query': query, 'fetched_utc': '2026-11-15T13:51:00+00:00',
            'places': [place_to_dict(MUSEUM)] if places is None else places}


# --- basic use ------------------------------------------------------------------

def test_missing_file_is_an_empty_cache(tmp_path):
    cache = make_cache(tmp_path)
    assert len(cache) == 0 and cache.get('science museum', NOW) is None
    assert not cache.path.exists()   # nothing written until something is saved


def test_put_then_get(tmp_path):
    cache = make_cache(tmp_path)
    assert cache.put('Science Museum', (MUSEUM,), NOW)
    hit = cache.get('Science Museum', NOW + 60.0)
    assert hit.places == (MUSEUM,) and hit.query == 'Science Museum'
    assert hit.fresh and hit.age == pytest.approx(60.0)


@pytest.mark.parametrize('asked', ['The Science Museum', 'SCIENCE museum!', '  science   museum '])
def test_get_uses_the_normalised_query(tmp_path, asked):
    cache = make_cache(tmp_path)
    cache.put('Science Museum', (MUSEUM,), NOW)
    assert cache.get(asked, NOW).places == (MUSEUM,)


def test_other_query_is_a_miss(tmp_path):
    cache = make_cache(tmp_path)
    cache.put('Science Museum', (MUSEUM,), NOW)
    assert cache.get('science museum shop', NOW) is None


def test_saved_answers_survive_a_restart(tmp_path):
    make_cache(tmp_path).put('Science Museum', (MUSEUM,), NOW)
    again = make_cache(tmp_path)
    assert len(again) == 1 and again.get('science museum', NOW).places == (MUSEUM,)


def test_file_is_readable_json_in_the_documented_format(tmp_path):
    cache = make_cache(tmp_path)
    cache.put('The Science Museum', (MUSEUM,), NOW)
    document = json.loads(cache.path.read_text(encoding='utf-8'))
    assert document['version'] == CACHE_VERSION == 1
    assert document['provider'] == 'nominatim' and 'OpenStreetMap' in document['attribution']
    entry = document['entries']['science museum']
    assert entry['query'] == 'The Science Museum'
    assert entry['fetched_utc'] == '2026-11-15T13:51:00+00:00'
    assert entry['places'] == [place_to_dict(MUSEUM)]


def test_no_results_answer_is_saved_too(tmp_path):
    make_cache(tmp_path).put('zzqx no such place', (), NOW)
    hit = make_cache(tmp_path).get('zzqx no such place', NOW)
    assert hit is not None and hit.places == () and hit.fresh


def test_real_parsed_places_round_trip(tmp_path):
    with open(DATA / 'nominatim_museum.json', encoding='utf-8') as f:
        places = parse_results(json.load(f), GRAPH_BBOX).places
    make_cache(tmp_path).put('museum', places, NOW)
    assert make_cache(tmp_path).get('museum', NOW).places == places


def test_put_rejects_an_empty_query(tmp_path):
    with pytest.raises(ValueError):
        make_cache(tmp_path).put(' ?! ', (MUSEUM,), NOW)


@pytest.mark.parametrize('kwargs', [{'max_age_days': 0}, {'negative_max_age_days': -1.0}])
def test_config_rejects_bad_values(kwargs):
    with pytest.raises(ValueError):
        CacheConfig(**kwargs)


def test_place_from_dict_rejects_wrong_types():
    data = place_to_dict(MUSEUM)
    data['osm_id'] = 'way'
    with pytest.raises(ValueError):
        place_from_dict(data)


# --- freshness and area -------------------------------------------------------------

def test_places_are_fresh_for_180_days_then_stale(tmp_path):
    cache = make_cache(tmp_path)
    cache.put('Science Museum', (MUSEUM,), NOW)
    assert cache.get('science museum', NOW + 179 * DAY).fresh
    stale = cache.get('science museum', NOW + 181 * DAY)
    assert not stale.fresh and stale.places == (MUSEUM,)   # still returned: usable offline


def test_no_results_answers_are_stale_after_7_days(tmp_path):
    cache = make_cache(tmp_path)
    cache.put('zzqx', (), NOW)
    assert cache.get('zzqx', NOW + 6 * DAY).fresh
    assert not cache.get('zzqx', NOW + 8 * DAY).fresh


def test_a_clock_in_the_past_counts_as_fresh(tmp_path):
    cache = make_cache(tmp_path)
    cache.put('Science Museum', (MUSEUM,), NOW)
    hit = cache.get('science museum', NOW - 3600.0)
    assert hit.fresh and hit.age == 0.0


def test_area_filter_leaves_out_places_outside(tmp_path):
    cache = make_cache(tmp_path)
    cache.put('Science Museum', (MUSEUM,), NOW)
    elsewhere = BBox(51.0, -1.0, 51.1, -0.9)
    hit = cache.get('science museum', NOW, area=elsewhere)
    assert hit.places == ()
    assert cache.entry('science museum').places == (MUSEUM,)   # not deleted


# --- surviving updates and accidents ------------------------------------------------

def test_put_replaces_the_entry_and_keeps_its_unknown_fields(tmp_path):
    entry = raw_entry(places=[])
    entry['team_note'] = 'checked by hand'
    write_file(tmp_path, v1_document(**{'science museum': entry}))
    cache = GeocodeCache(tmp_path / 'cache' / 'geocode_cache.json')
    cache.load()
    cache.put('Science Museum', (MUSEUM,), NOW + DAY)
    saved = json.loads(cache.path.read_text(encoding='utf-8'))['entries']['science museum']
    assert saved['team_note'] == 'checked by hand'
    assert saved['places'] == [place_to_dict(MUSEUM)]


def test_unknown_top_level_fields_survive_a_save(tmp_path):
    document = v1_document(**{'science museum': raw_entry()})
    document['comment'] = 'copied from the lab laptop'
    path = write_file(tmp_path, document)
    cache = GeocodeCache(path)
    cache.load()
    cache.put('museum', (), NOW)
    assert json.loads(path.read_text(encoding='utf-8'))['comment'] == 'copied from the lab laptop'


def test_an_unreadable_entry_is_kept_unchanged_and_not_used(tmp_path):
    broken = {'query': 'Royal Albert Hall', 'places': 'not a list'}
    path = write_file(tmp_path, v1_document(**{'royal albert hall': broken,
                                               'science museum': raw_entry()}))
    cache = GeocodeCache(path)
    warnings = cache.load()
    assert len(warnings) == 1 and 'royal albert hall' in warnings[0]
    assert cache.get('royal albert hall', NOW) is None and len(cache) == 1
    cache.put('museum', (), NOW)
    assert json.loads(path.read_text(encoding='utf-8'))['entries']['royal albert hall'] == broken


@pytest.mark.parametrize('content', [
    '{"version": 1, "entries": {',      # cut off mid-write
    '[1, 2, 3]',                         # not an object
    '{"version": 1}',                    # no entries
    '{"version": "one", "entries": {}}',  # version not an integer
])
def test_a_corrupt_file_is_moved_aside_and_the_cache_starts_empty(tmp_path, content):
    path = write_file(tmp_path, content)
    cache = GeocodeCache(path)
    warnings = cache.load()
    assert len(warnings) == 1 and 'cannot be read' in warnings[0]
    assert len(cache) == 0 and not path.exists()
    (moved,) = path.parent.glob('geocode_cache.json.corrupt-*')
    assert moved.read_text(encoding='utf-8') == content     # kept for inspection
    assert cache.put('museum', (), NOW)                     # and the cache works again


def test_every_save_keeps_a_backup_of_the_previous_file(tmp_path):
    cache = make_cache(tmp_path)
    cache.put('Science Museum', (MUSEUM,), NOW)
    first = cache.path.read_text(encoding='utf-8')
    cache.put('museum', (), NOW)
    backup = cache.path.with_name('geocode_cache.json.bak')
    assert backup.read_text(encoding='utf-8') == first


def test_save_leaves_no_temporary_file_and_creates_folders(tmp_path):
    cache = GeocodeCache(tmp_path / 'a' / 'b' / 'geocode_cache.json')
    cache.load()
    cache.put('museum', (), NOW)
    assert sorted(p.name for p in cache.path.parent.iterdir()) == ['geocode_cache.json']


def test_a_file_from_a_newer_program_is_used_but_never_overwritten(tmp_path):
    document = v1_document(**{'science museum': raw_entry()})
    document['version'] = CACHE_VERSION + 1
    path = write_file(tmp_path, document)
    before = path.read_text(encoding='utf-8')
    cache = GeocodeCache(path)
    warnings = cache.load()
    assert cache.read_only and 'newer' in warnings[0]
    assert cache.get('science museum', NOW).places == (MUSEUM,)
    assert cache.put('museum', (), NOW) is False
    assert path.read_text(encoding='utf-8') == before


def test_an_older_file_is_migrated(tmp_path):
    # A made-up version 0 that stored a flat list; the migration turns it into version 1.
    def from_v0(document):
        return {'version': 1, 'entries': {normalise_key(e['query']): e
                                          for e in document['items']}}

    def normalise_key(query):
        return ' '.join(query.lower().split())

    path = write_file(tmp_path, {'version': 0, 'items': [raw_entry()]})
    cache = GeocodeCache(path, migrations={0: from_v0})
    warnings = cache.load()
    assert 'migrated from version 0' in warnings[0] and not cache.read_only
    assert cache.get('science museum', NOW).places == (MUSEUM,)
    cache.put('museum', (), NOW)
    assert json.loads(path.read_text(encoding='utf-8'))['version'] == CACHE_VERSION
    backup = json.loads(path.with_name('geocode_cache.json.bak').read_text(encoding='utf-8'))
    assert backup['version'] == 0                       # the old file is kept


def test_an_older_file_without_a_migration_is_read_only(tmp_path):
    path = write_file(tmp_path, {'version': 0, 'entries': {'science museum': raw_entry()}})
    cache = GeocodeCache(path, migrations={})
    warnings = cache.load()
    assert cache.read_only and 'no migration' in warnings[0]
    assert cache.get('science museum', NOW).places == (MUSEUM,)
    assert cache.put('museum', (), NOW) is False
