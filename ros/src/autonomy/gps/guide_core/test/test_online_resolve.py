"""Tests for guide_core.online_resolve: geocoder answers -> Resolutions."""

from guide_core.destinations import (
    Destination, format_distance, MatchConfig, MatchType, Outcome)
from guide_core.geocoder import Status
from guide_core.online_resolve import not_found_text, resolve_online
import pytest

ROBOT = (51.4986289, -0.1752868)
HALL = Destination('Royal Albert Hall', 51.501007, -0.1774058, note='large area',
                   source='nominatim')
MUSEUMS = tuple(Destination(name, lat, lon, note=note, source='nominatim')
                for name, lat, lon, note in [
    ('Natural History Museum', 51.4965109, -0.1760019, 'large area'),
    ('Victoria and Albert Museum', 51.4968838, -0.1716372, 'large area'),
    ('Science Museum', 51.4973983, -0.1746726, 'large area'),
    ('Polish Institute and Sikorski Museum', 51.5014362, -0.1728653, ''),
    ("The Clockmakers' Museum", 51.4971162, -0.1762760, ''),
                ])


def test_one_result_is_unique_and_says_where_it_was_found():
    resolution = resolve_online('royal albert hall', Status.OK, (HALL,), robot_latlon=ROBOT)
    assert resolution.outcome is Outcome.UNIQUE
    (only,) = resolution.candidates
    assert only.destination == HALL and only.match_type is MatchType.FUZZY
    assert only.distance == pytest.approx(300.0, abs=15.0)
    assert resolution.message == ('Found in OpenStreetMap: Royal Albert Hall, large area, '
                                  f'{format_distance(only.distance)} away.')


def test_a_cached_answer_says_saved_results():
    resolution = resolve_online('royal albert hall', Status.OK, (HALL,), from_cache=True)
    assert resolution.message == ('Found in saved OpenStreetMap results: '
                                  'Royal Albert Hall, large area.')


def test_several_results_are_ambiguous_in_the_geocoders_order():
    resolution = resolve_online('museum', Status.OK, MUSEUMS, robot_latlon=ROBOT)
    assert resolution.outcome is Outcome.AMBIGUOUS
    assert [c.destination.name for c in resolution.candidates] == [d.name for d in MUSEUMS]
    message = resolution.message
    assert message.startswith("I found 5 places in OpenStreetMap for 'museum'. "
                              'Option 1: Natural History Museum, large area, about ')
    assert 'Option 3: Science Museum' in message and 'Option 4' not in message
    assert message.endswith(' And 2 more. Which one?')


def test_candidates_are_limited_to_max_candidates():
    resolution = resolve_online('museum', Status.OK, MUSEUMS,
                                config=MatchConfig(max_candidates=2))
    assert len(resolution.candidates) == 2 and 'And' not in resolution.message


def test_score_says_how_well_the_name_matches():
    resolution = resolve_online('museum', Status.OK, MUSEUMS)
    assert all(c.score == pytest.approx(0.9) for c in resolution.candidates)  # word subset


@pytest.mark.parametrize('status, ending', [
    (Status.OK, ', and OpenStreetMap has nothing by that name near here.'),   # OK but empty
    (Status.NO_RESULTS, ', and OpenStreetMap has nothing by that name near here.'),
    (Status.DISABLED, '. Online search is switched off.'),
    (Status.OFFLINE, ", and I couldn't search online just now. Try another name."),
    (Status.RATE_LIMITED, ', and online search is paused for now. Try another name.'),
    (Status.ERROR, ", and I couldn't search online just now. Try another name."),
    (99, ", and I couldn't search online just now. Try another name."),       # unknown status
])
def test_nothing_usable_is_not_found_with_the_reason(status, ending):
    resolution = resolve_online('  Zzqx   Place ', status, ())
    assert resolution.outcome is Outcome.NOT_FOUND and resolution.candidates == ()
    assert resolution.message == "I don't know a place called 'Zzqx Place'" + ending


def test_candidates_are_ignored_unless_status_is_ok():
    resolution = resolve_online('royal albert hall', Status.OFFLINE, (HALL,))
    assert resolution.outcome is Outcome.NOT_FOUND


def test_not_found_text_accepts_plain_status_values():
    assert not_found_text('x', Status(2)).endswith('switched off.')
