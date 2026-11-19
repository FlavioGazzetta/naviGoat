"""Tests for guide_core.rate_limit: the request budget for Nominatim."""

from guide_core.rate_limit import RateLimitConfig, RateLimiter
import pytest

T0 = 1000.0   # any monotonic time


def test_first_request_may_go_at_once():
    decision = RateLimiter().check(T0)
    assert decision.send and decision.wait == 0.0


def test_a_quick_second_request_waits_for_the_rest_of_the_interval():
    limiter = RateLimiter()
    limiter.record_request(T0)
    decision = limiter.check(T0 + 0.3)
    assert decision.send and decision.wait == pytest.approx(0.8)


def test_after_the_interval_there_is_no_wait():
    limiter = RateLimiter()
    limiter.record_request(T0)
    assert limiter.check(T0 + 1.1).wait == 0.0
    assert limiter.check(T0 + 60.0).wait == 0.0


def test_check_alone_changes_nothing():
    limiter = RateLimiter()
    for _ in range(5):
        assert limiter.check(T0).wait == 0.0


def test_hourly_limit_refuses_until_the_oldest_request_is_an_hour_old():
    limiter = RateLimiter(RateLimitConfig(max_per_hour=3))
    for i in range(3):
        limiter.record_request(T0 + 10.0 * i)
    refused = limiter.check(T0 + 30.0)
    assert not refused.send and 'limit of 3' in refused.reason and '60 min' in refused.reason
    assert not limiter.check(T0 + 3599.0).send
    assert limiter.check(T0 + 3600.5).send


def test_a_refusal_from_the_server_pauses_everything_for_the_cooldown():
    limiter = RateLimiter()
    limiter.record_request(T0)
    limiter.record_refused(T0 + 0.5)
    paused = limiter.check(T0 + 600.5)
    assert not paused.send and '50 more min' in paused.reason
    assert not limiter.check(T0 + 3600.0).send
    assert limiter.check(T0 + 3600.6).send


@pytest.mark.parametrize('kwargs', [
    {'min_interval': 0.5},       # faster than the policy's 1 request per second
    {'max_per_hour': 0},
    {'block_cooldown': -1.0},
])
def test_config_rejects_bad_values(kwargs):
    with pytest.raises(ValueError):
        RateLimitConfig(**kwargs)
