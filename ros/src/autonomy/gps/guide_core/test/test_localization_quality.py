"""Tests for the GOOD / DEGRADED / LOST localisation quality monitor."""

import math

from guide_core.localization_quality import Quality, QualityConfig, QualityMonitor
import pytest


def feed(monitor, samples, start=0.0, dt=1.0):
    """Feed (sigma, since_fix) samples one second apart; return the qualities."""
    out = []
    for i, (sigma, since_fix) in enumerate(samples):
        out.append(monitor.update(start + i * dt, True, sigma, since_fix).quality)
    return out


def good_monitor():
    """Return a monitor that has become GOOD."""
    monitor = QualityMonitor()
    feed(monitor, [(2.0, 0.0)] * 6)
    assert monitor.state.quality is Quality.GOOD
    return monitor


def test_starts_lost_and_stays_lost_until_initialised():
    monitor = QualityMonitor()
    assert monitor.state.quality is Quality.LOST
    state = monitor.update(0.0, False, math.inf, math.inf)
    assert state.quality is Quality.LOST
    assert state.reason == 'not localised yet'


def test_good_after_hold_time_of_good_values():
    monitor = QualityMonitor()
    qualities = feed(monitor, [(2.0, 0.0)] * 6)
    # Conditions good from t=0: improves at t=5 (hold_time), not before.
    assert qualities == [Quality.LOST] * 5 + [Quality.GOOD]


def test_large_sigma_degrades_immediately():
    monitor = good_monitor()
    state = monitor.update(10.0, True, 5.5, 0.0)
    assert state.quality is Quality.DEGRADED
    assert state.changed
    assert state.since == 10.0
    assert state.reason == 'position uncertainty 5.5 m'


def test_long_silence_degrades_even_with_small_sigma():
    monitor = good_monitor()
    assert monitor.update(10.0, True, 2.0, 19.0).quality is Quality.GOOD
    state = monitor.update(11.0, True, 2.0, 21.0)
    assert state.quality is Quality.DEGRADED
    assert state.reason == 'no GPS fix for 21 s'


def test_sigma_between_thresholds_does_not_flicker():
    monitor = good_monitor()
    # 4 m: below the 5 m entry threshold, above the 3 m exit threshold.
    assert feed(monitor, [(4.0, 0.0)] * 10, start=10.0) == [Quality.GOOD] * 10
    monitor.update(20.0, True, 6.0, 0.0)
    assert feed(monitor, [(4.0, 0.0)] * 10, start=21.0) == [Quality.DEGRADED] * 10


def test_recovery_needs_hold_time_in_a_row():
    monitor = good_monitor()
    monitor.update(10.0, True, 6.0, 0.0)
    # Good for 4 s, one bad second, then good again: the clock restarts.
    samples = [(2.0, 0.0)] * 4 + [(3.5, 0.0)] + [(2.0, 0.0)] * 6
    qualities = feed(monitor, samples, start=11.0)
    assert qualities[:10] == [Quality.DEGRADED] * 10
    assert qualities[10] is Quality.GOOD


def test_recovery_needs_a_recent_fix():
    monitor = good_monitor()
    monitor.update(10.0, True, 6.0, 0.0)
    assert feed(monitor, [(2.0, 8.0)] * 10, start=11.0) == [Quality.DEGRADED] * 10


def test_lost_and_back_through_degraded():
    monitor = good_monitor()
    assert monitor.update(10.0, True, 16.0, 30.0).quality is Quality.LOST
    # 12 m: under the 15 m entry but over the 10 m exit: stays LOST.
    assert feed(monitor, [(12.0, 0.0)] * 6, start=11.0) == [Quality.LOST] * 6
    # 8 m with fixes: LOST -> DEGRADED after 5 s; still above good_sigma.
    qualities = feed(monitor, [(8.0, 0.0)] * 6, start=17.0)
    assert qualities == [Quality.LOST] * 5 + [Quality.DEGRADED]


def test_lost_can_recover_straight_to_good():
    monitor = good_monitor()
    monitor.update(10.0, True, 20.0, 0.0)
    qualities = feed(monitor, [(2.0, 0.0)] * 6, start=11.0)
    assert qualities == [Quality.LOST] * 5 + [Quality.GOOD]


def test_infinite_or_nan_sigma_is_lost():
    monitor = good_monitor()
    assert monitor.update(10.0, True, math.inf, 0.0).quality is Quality.LOST
    monitor = good_monitor()
    assert monitor.update(10.0, True, math.nan, 0.0).quality is Quality.LOST


def test_changed_only_on_the_transition():
    monitor = good_monitor()
    assert monitor.update(10.0, True, 6.0, 0.0).changed
    assert not monitor.update(11.0, True, 6.0, 0.0).changed
    assert monitor.state.since == 10.0


def test_degraded_reason_names_both_problems():
    monitor = good_monitor()
    state = monitor.update(10.0, True, 6.2, 25.0)
    assert state.reason == 'position uncertainty 6.2 m, no GPS fix for 25 s'


@pytest.mark.parametrize('kwargs', [
    {'good_sigma': 6.0},
    {'lost_exit_sigma': 16.0},
    {'recent_fix_time': 30.0},
    {'hold_time': -1.0},
])
def test_invalid_config_raises(kwargs):
    with pytest.raises(ValueError):
        QualityConfig(**kwargs)
