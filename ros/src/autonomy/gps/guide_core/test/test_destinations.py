"""Tests for guide_core.destinations (curated destination list and matching)."""

from guide_core.destinations import (
    Destination, DestinationError, destinations_from_dict, format_distance, MatchConfig, MatchType,
    normalise, Outcome, resolve,
)
import pytest

CENTRAL = Destination('Central Library', 51.4987, -0.1779,
                      aliases=('library', 'abbey library'), note='main entrance, step-free')
SCIENCE = Destination('Science Library', 51.4990, -0.1760, note='side entrance')
SOUTH_KEN = Destination('South Kensington Station', 51.4941, -0.1738,
                        aliases=('tube station', 'south ken'), note='subway exit, step-free')
GLOUCESTER = Destination('Gloucester Road Station', 51.4945, -0.1830)
DESTINATIONS = (CENTRAL, SCIENCE, SOUTH_KEN, GLOUCESTER)


def names(resolution):
    return [c.destination.name for c in resolution.candidates]


# --- normalise ---

@pytest.mark.parametrize('text, expected', [
    ('The Library', 'library'),
    ('  CENTRAL   library!! ', 'central library'),
    ("Queen's Tower", 'queens tower'),
    ('Café Nero', 'cafe nero'),
    ('V&A Museum', 'v and a museum'),
    ('the', 'the'),          # a lone "the" is kept: it is the whole name
    ('', ''),
])
def test_normalise(text, expected):
    assert normalise(text) == expected


# --- loading ---

def test_load_valid_list():
    data = {'destinations': [
        {'name': ' Central Library ', 'lat': 51.4987, 'lon': -0.1779,
         'aliases': ['library'], 'note': 'main entrance'},
        {'name': 'Gloucester Road Station', 'lat': 51, 'lon': 0},
    ]}
    loaded = destinations_from_dict(data)
    assert loaded == (
        Destination('Central Library', 51.4987, -0.1779, ('library',), 'main entrance'),
        Destination('Gloucester Road Station', 51.0, 0.0),
    )


GOOD = {'name': 'A', 'lat': 51.0, 'lon': 0.0}


@pytest.mark.parametrize('data', [
    [GOOD],                                                   # not a mapping
    {'places': [GOOD]},                                       # no 'destinations' key
    {'destinations': GOOD},                                   # not a list
    {'destinations': []},                                     # empty list
    {'destinations': [{'lat': 51.0, 'lon': 0.0}]},            # no name
    {'destinations': [{**GOOD, 'name': '  !! '}]},            # name empty after normalising
    {'destinations': [{**GOOD, 'lat': 91.0}]},                # latitude out of range
    {'destinations': [{**GOOD, 'lon': 'abc'}]},               # not a number
    {'destinations': [{**GOOD, 'lat': True}]},                # bool is not a number
    {'destinations': [{**GOOD, 'alias': ['a']}]},             # unknown key (typo)
    {'destinations': [{**GOOD, 'aliases': 'a'}]},             # aliases not a list
    {'destinations': [GOOD, {**GOOD, 'name': 'the a'}]},      # same name after normalising
])
def test_load_rejects_bad_lists(data):
    with pytest.raises(DestinationError):
        destinations_from_dict(data)


# --- resolve: outcomes ---

def test_exact_name_is_unique():
    r = resolve('South Kensington Station', DESTINATIONS)
    assert r.outcome is Outcome.UNIQUE
    best = r.candidates[0]
    assert best.destination is SOUTH_KEN
    assert best.match_type is MatchType.EXACT
    assert best.score == 1.0
    assert best.matched_text == 'South Kensington Station'


def test_exact_alias_wins_over_partial_matches():
    # "library" is an alias of Central Library, and also one of the words of
    # "Science Library": the curated alias decides.
    r = resolve('library', DESTINATIONS)
    assert r.outcome is Outcome.UNIQUE
    assert names(r)[:2] == ['Central Library', 'Science Library']
    assert r.candidates[0].matched_text == 'library'
    assert r.candidates[1].match_type is MatchType.FUZZY


def test_case_punctuation_and_the_are_ignored():
    r = resolve('  The LIBRARY! ', DESTINATIONS)
    assert r.outcome is Outcome.UNIQUE
    assert r.candidates[0].destination is CENTRAL
    assert r.candidates[0].match_type is MatchType.EXACT


@pytest.mark.parametrize('query, expected, matched', [
    ('libary', CENTRAL, 'library'),
    ('south kensingtn station', SOUTH_KEN, 'South Kensington Station'),
])
def test_spelling_mistake_is_unique_fuzzy(query, expected, matched):
    r = resolve(query, DESTINATIONS)
    assert r.outcome is Outcome.UNIQUE
    assert r.candidates[0].destination is expected
    assert r.candidates[0].match_type is MatchType.FUZZY
    assert r.candidates[0].matched_text == matched


def test_one_word_of_one_name_is_unique():
    r = resolve('kensington', DESTINATIONS)
    assert r.outcome is Outcome.UNIQUE
    assert r.candidates[0].destination is SOUTH_KEN


def test_word_shared_by_several_names_is_ambiguous():
    r = resolve('station', DESTINATIONS)
    assert r.outcome is Outcome.AMBIGUOUS
    assert set(names(r)[:2]) == {'South Kensington Station', 'Gloucester Road Station'}


def test_alias_shared_by_two_places_is_ambiguous():
    a = Destination('North Library', 51.0, 0.0, aliases=('library',))
    b = Destination('South Library', 51.001, 0.0, aliases=('library',))
    r = resolve('library', (a, b))
    assert r.outcome is Outcome.AMBIGUOUS
    assert all(c.match_type is MatchType.EXACT for c in r.candidates)


def test_unknown_place_is_not_found():
    r = resolve('swimming pool', DESTINATIONS)
    assert r.outcome is Outcome.NOT_FOUND
    assert r.candidates == ()
    assert r.message == "I don't know a place called 'swimming pool'."


@pytest.mark.parametrize('query', ['', '   ', '?!'])
def test_empty_query_is_not_found(query):
    r = resolve(query, DESTINATIONS)
    assert r.outcome is Outcome.NOT_FOUND
    assert r.message == "I didn't catch a destination name."


# --- resolve: distances and ranking ---

def test_no_robot_position_means_no_distance():
    r = resolve('library', DESTINATIONS)
    assert all(c.distance is None for c in r.candidates)
    assert r.message == 'Central Library, main entrance, step-free.'


def test_straight_line_distance_from_robot():
    r = resolve('tube station', DESTINATIONS, robot_latlon=(SOUTH_KEN.lat, SOUTH_KEN.lon))
    assert r.candidates[0].destination is SOUTH_KEN
    assert r.candidates[0].distance == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize('robot, first', [
    ((SOUTH_KEN.lat, SOUTH_KEN.lon), 'South Kensington Station'),
    ((GLOUCESTER.lat, GLOUCESTER.lon), 'Gloucester Road Station'),
])
def test_equal_scores_ranked_nearest_first(robot, first):
    r = resolve('station', DESTINATIONS, robot_latlon=robot)
    assert r.candidates[0].score == r.candidates[1].score
    assert names(r)[0] == first


def test_max_candidates_limits_list_not_outcome():
    r = resolve('station', DESTINATIONS, config=MatchConfig(max_candidates=1))
    assert r.outcome is Outcome.AMBIGUOUS
    assert len(r.candidates) == 1


# --- messages ---

@pytest.mark.parametrize('metres, spoken', [
    (4.0, 'about 10 m'),
    (304.0, 'about 300 m'),
    (995.0, 'about 1.0 km'),
    (1234.0, 'about 1.2 km'),
])
def test_format_distance(metres, spoken):
    assert format_distance(metres) == spoken


def test_unique_message_with_distance():
    r = resolve('library', DESTINATIONS, robot_latlon=(SOUTH_KEN.lat, SOUTH_KEN.lon))
    d = format_distance(r.candidates[0].distance)
    assert r.message == f'Central Library, main entrance, step-free, {d} away.'


def test_ambiguous_message_lists_options():
    r = resolve('station', DESTINATIONS)
    assert r.message == (
        "I found 2 places for 'station'. "
        'Option 1: Gloucester Road Station. '
        'Option 2: South Kensington Station, subway exit, step-free. Which one?')


def test_single_uncertain_match_asks_did_you_mean():
    r = resolve('kensingtn station', (SOUTH_KEN,))
    assert r.outcome is Outcome.AMBIGUOUS
    assert r.candidates[0].score < MatchConfig().unique_score
    assert r.message == ('Did you mean South Kensington Station, '
                         'subway exit, step-free?')


def test_ambiguous_message_reads_back_at_most_max_read_back():
    r = resolve('station', DESTINATIONS, config=MatchConfig(max_read_back=1))
    assert r.message == ("I found 2 places for 'station'. "
                         'Option 1: Gloucester Road Station. And 1 more. Which one?')


# --- config ---

@pytest.mark.parametrize('kwargs', [
    {'min_score': 0.0},
    {'unique_score': 0.4},          # below min_score
    {'unique_margin': -0.1},
    {'max_read_back': 0},
])
def test_config_rejects_bad_values(kwargs):
    with pytest.raises(ValueError):
        MatchConfig(**kwargs)
