"""Tests for guide_core.geocoder: the lookup flow, with a fake Nominatim.

No network: fetch is a fake that returns the canned responses in test/data
or raises FetchError. Clocks are fake too, so no test sleeps.
"""

import json
from pathlib import Path

from guide_core.geocode_cache import DAY, GeocodeCache
from guide_core.geocoder import FetchError, Geocoder, Status
from guide_core.geocoding import BBox
from guide_core.rate_limit import RateLimitConfig, RateLimiter
import pytest

DATA = Path(__file__).parent / 'data'
GRAPH_BBOX = BBox.around(51.4988, -0.1749, 1500.0)
NOW = 1794750660.0   # 15 Nov 2026 13:51 UTC


def canned(name):
    with open(DATA / f'nominatim_{name}.json', encoding='utf-8') as f:
        return json.load(f)


class FakeNominatim:
    """Answers like the real server; records every request."""

    def __init__(self, answers=None, error=None):
        self.answers = answers or {'science museum': canned('science_museum'),
                                   'museum': canned('museum')}
        self.error = error          # a FetchError to raise instead of answering
        self.requests = []

    def __call__(self, params):
        self.requests.append(params)
        if self.error is not None:
            raise self.error
        return self.answers.get(params['q'].lower(), [])


class Clock:
    """Fake wall clock, monotonic clock and sleep, moving together."""

    def __init__(self):
        self.wall, self.mono, self.slept = NOW, 0.0, []

    def advance(self, seconds):
        self.wall += seconds
        self.mono += seconds

    def sleep(self, seconds):
        self.slept.append(seconds)
        self.advance(seconds)


@pytest.fixture
def clock():
    return Clock()


def make(tmp_path, clock, fake=None, limiter=None, online=True):
    cache = GeocodeCache(tmp_path / 'geocode_cache.json')
    cache.load()
    fake = fake if fake is not None else FakeNominatim()
    geocoder = Geocoder(GRAPH_BBOX, cache, fake, limiter=limiter, online=online,
                        wall_clock=lambda: clock.wall, monotonic=lambda: clock.mono,
                        sleep=clock.sleep)
    return geocoder, fake


def test_status_values_match_the_service_definition():
    assert [s.value for s in Status] == [0, 1, 2, 3, 4, 5]
    assert [s.name for s in Status] == ['OK', 'NO_RESULTS', 'DISABLED', 'OFFLINE',
                                        'RATE_LIMITED', 'ERROR']


def test_first_lookup_asks_nominatim_and_saves_the_answer(tmp_path, clock):
    geocoder, fake = make(tmp_path, clock)
    result = geocoder.lookup('  Science   Museum ')
    assert result.status is Status.OK and not result.from_cache
    assert [p.name for p in result.places] == ['Science Museum']
    assert result.message.startswith('Nominatim: 1 place(s) in ')
    (params,) = fake.requests
    assert params['q'] == 'Science Museum' and params['viewbox'] == GRAPH_BBOX.viewbox()
    assert geocoder.cache.entry('science museum').places == result.places


@pytest.mark.parametrize('again', ['Science Museum', 'the science museum', 'SCIENCE MUSEUM'])
def test_asking_again_uses_the_cache_without_a_request(tmp_path, clock, again):
    geocoder, fake = make(tmp_path, clock)
    first = geocoder.lookup('Science Museum')
    clock.advance(60.0)
    second = geocoder.lookup(again)
    assert len(fake.requests) == 1
    assert second.status is Status.OK and second.from_cache and not second.stale
    assert second.places == first.places
    assert second.message == 'From cache (saved 1 min ago): 1 place(s)'


def test_no_results_is_saved_and_not_asked_again(tmp_path, clock):
    geocoder, fake = make(tmp_path, clock)
    assert geocoder.lookup('zzqx no such place').status is Status.NO_RESULTS
    clock.advance(5.0)
    again = geocoder.lookup('zzqx no such place')
    assert again.status is Status.NO_RESULTS and again.from_cache
    assert len(fake.requests) == 1


def test_switched_off_and_not_cached_is_disabled(tmp_path, clock):
    geocoder, fake = make(tmp_path, clock, online=False)
    result = geocoder.lookup('Science Museum')
    assert result.status is Status.DISABLED and result.places == ()
    assert fake.requests == []


def test_switched_off_still_answers_from_the_cache_even_if_stale(tmp_path, clock):
    geocoder, fake = make(tmp_path, clock)
    geocoder.lookup('Science Museum')
    geocoder.online = False
    clock.advance(200 * DAY)
    result = geocoder.lookup('Science Museum')
    assert result.status is Status.OK and result.from_cache and result.stale
    assert 'online search is off' in result.message
    assert len(fake.requests) == 1


def test_a_stale_entry_is_refreshed_when_online(tmp_path, clock):
    geocoder, fake = make(tmp_path, clock)
    geocoder.lookup('Science Museum')
    clock.advance(200 * DAY)
    result = geocoder.lookup('Science Museum')
    assert len(fake.requests) == 2
    assert result.status is Status.OK and not result.from_cache
    assert geocoder.cache.get('science museum', clock.wall).fresh


def test_a_stale_entry_is_used_when_the_refresh_fails(tmp_path, clock):
    geocoder, fake = make(tmp_path, clock)
    geocoder.lookup('Science Museum')
    clock.advance(200 * DAY)
    fake.error = FetchError('cannot reach nominatim.openstreetmap.org: timed out')
    result = geocoder.lookup('Science Museum')
    assert result.status is Status.OK and result.from_cache and result.stale
    assert 'refresh failed: cannot reach' in result.message


def test_offline_and_not_cached_is_offline(tmp_path, clock):
    fake = FakeNominatim(error=FetchError('cannot reach nominatim.openstreetmap.org'))
    geocoder, _ = make(tmp_path, clock, fake)
    result = geocoder.lookup('Science Museum')
    assert result.status is Status.OFFLINE and result.places == ()
    assert geocoder.cache.entry('science museum') is None   # failures are never cached


def test_a_refusal_pauses_requests_for_the_cooldown(tmp_path, clock):
    fake = FakeNominatim(error=FetchError('HTTP 429 from nominatim', kind='refused'))
    geocoder, _ = make(tmp_path, clock, fake)
    assert geocoder.lookup('Science Museum').status is Status.RATE_LIMITED
    fake.error = None
    clock.advance(60.0)
    paused = geocoder.lookup('museum')
    assert paused.status is Status.RATE_LIMITED and 'paused' in paused.message
    assert len(fake.requests) == 1                       # nothing sent while paused
    clock.advance(3600.0)
    assert geocoder.lookup('museum').status is Status.OK


def test_hourly_cap_refuses_without_a_request(tmp_path, clock):
    limiter = RateLimiter(RateLimitConfig(max_per_hour=2))
    geocoder, fake = make(tmp_path, clock, limiter=limiter)
    for name in ('one place', 'two places'):
        geocoder.lookup(name)
        clock.advance(2.0)
    capped = geocoder.lookup('three places')
    assert capped.status is Status.RATE_LIMITED and 'per hour' in capped.message
    assert len(fake.requests) == 2


def test_a_quick_second_request_waits_for_the_interval(tmp_path, clock):
    geocoder, fake = make(tmp_path, clock)
    geocoder.lookup('Science Museum')
    clock.advance(0.3)
    geocoder.lookup('museum')
    assert clock.slept == [pytest.approx(0.8)] and len(fake.requests) == 2


@pytest.mark.parametrize('answer, error', [
    ({'error': 'Unable to geocode'}, None),                      # 200 but not a list
    (None, FetchError('answer is not JSON', kind='bad_answer')),
])
def test_an_unreadable_answer_is_an_error_and_not_cached(tmp_path, clock, answer, error):
    fake = FakeNominatim(answers={'science museum': answer}, error=error)
    geocoder, _ = make(tmp_path, clock, fake)
    result = geocoder.lookup('Science Museum')
    assert result.status is Status.ERROR and geocoder.cache.entry('science museum') is None


@pytest.mark.parametrize('query', ['', 'ab', '  ?! '])
def test_a_too_short_query_is_never_sent(tmp_path, clock, query):
    geocoder, fake = make(tmp_path, clock)
    assert geocoder.lookup(query).status is Status.ERROR
    assert fake.requests == []


def test_a_failed_cache_save_still_returns_the_places(tmp_path, clock, monkeypatch):
    geocoder, _ = make(tmp_path, clock)

    def broken_save():
        raise OSError('disk full')

    monkeypatch.setattr(geocoder.cache, 'save', broken_save)
    result = geocoder.lookup('Science Museum')
    assert result.status is Status.OK and 'could not save the cache: disk full' in result.message


def test_several_results_keep_nominatim_order(tmp_path, clock):
    geocoder, _ = make(tmp_path, clock)
    result = geocoder.lookup('museum')
    assert [p.name for p in result.places][:3] == [
        'Natural History Museum', 'Victoria and Albert Museum', 'Science Museum']
    assert len(result.places) == 5


def test_fetch_error_rejects_unknown_kinds():
    with pytest.raises(ValueError):
        FetchError('x', kind='strange')
