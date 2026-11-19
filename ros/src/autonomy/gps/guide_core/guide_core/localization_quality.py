"""Localisation quality GOOD / DEGRADED / LOST with hysteresis (Step 8).

Turns the alignment filter's numbers (is it initialised, position sigma,
seconds since the last accepted GPS fix) into one of three levels:

* GOOD: normal guiding.
* DEGRADED: keep going with larger margins (GPS weak or absent, e.g. a tunnel).
* LOST: position too uncertain to guide; the robot must stop.

Getting worse is immediate: a level is entered as soon as its entry condition
holds. Getting better is slow: the level improves only after the (stricter)
exit conditions have held for hold_time seconds in a row, so the state does
not flicker when a value hovers around a threshold.
"""

from dataclasses import dataclass
import enum
import math


class Quality(enum.IntEnum):
    """Localisation quality; larger is worse (values used in LocalizationStatus.msg)."""

    GOOD = 0
    DEGRADED = 1
    LOST = 2


@dataclass(frozen=True)
class QualityConfig:
    """Thresholds; each 'exit' threshold is stricter than its 'enter' threshold."""

    # DEGRADED is entered above degraded_sigma, left below good_sigma (m).
    degraded_sigma: float = 5.0
    good_sigma: float = 3.0
    # LOST is entered above lost_sigma, left below lost_exit_sigma (m).
    lost_sigma: float = 15.0
    lost_exit_sigma: float = 10.0
    # DEGRADED after this long without an accepted fix (s) ...
    no_fix_time: float = 20.0
    # ... and GOOD again only with an accepted fix within recent_fix_time (s).
    recent_fix_time: float = 5.0
    # Exit conditions must hold this long before the level improves (s).
    hold_time: float = 5.0

    def __post_init__(self):
        if not 0.0 < self.good_sigma < self.degraded_sigma <= self.lost_exit_sigma \
                < self.lost_sigma:
            raise ValueError('need 0 < good_sigma < degraded_sigma <= lost_exit_sigma'
                             ' < lost_sigma')
        if not 0.0 < self.recent_fix_time < self.no_fix_time:
            raise ValueError('need 0 < recent_fix_time < no_fix_time')
        if self.hold_time < 0.0:
            raise ValueError('hold_time must be >= 0')


@dataclass(frozen=True)
class QualityState:
    """The current level, why, and since when."""

    quality: Quality
    reason: str
    since: float
    changed: bool = False


def _describe(quality, initialised, sigma, since_fix, config):
    """Return a short reason for the level (for logs and the status topic)."""
    if not initialised:
        return 'not localised yet'
    if quality is Quality.GOOD:
        return f'position uncertainty {sigma:.1f} m'
    parts = []
    if sigma > config.good_sigma:
        parts.append(f'position uncertainty {sigma:.1f} m')
    if since_fix > config.recent_fix_time:
        parts.append('no GPS fix' if math.isinf(since_fix)
                     else f'no GPS fix for {since_fix:.0f} s')
    return ', '.join(parts) or f'position uncertainty {sigma:.1f} m'


class QualityMonitor:
    """Track the localisation quality level over time."""

    def __init__(self, config=None):
        self.config = config or QualityConfig()
        self.state = QualityState(Quality.LOST, 'not localised yet', -math.inf)
        self._better_since = None

    def entry_level(self, initialised, sigma, since_fix):
        """Return the level the enter thresholds call for."""
        c = self.config
        if not initialised or not sigma <= c.lost_sigma:
            return Quality.LOST
        if sigma > c.degraded_sigma or since_fix > c.no_fix_time:
            return Quality.DEGRADED
        return Quality.GOOD

    def exit_level(self, initialised, sigma, since_fix):
        """Return the best level the (stricter) exit thresholds allow."""
        c = self.config
        if not initialised or not sigma < c.lost_exit_sigma:
            return Quality.LOST
        if not sigma < c.good_sigma or since_fix > c.recent_fix_time:
            return Quality.DEGRADED
        return Quality.GOOD

    def update(self, now, initialised, sigma, since_fix):
        """Return the new state from the filter's numbers at time now (s)."""
        current = self.state.quality
        worse = self.entry_level(initialised, sigma, since_fix)
        new = current
        if worse > current:
            new = worse
            self._better_since = None
        else:
            better = self.exit_level(initialised, sigma, since_fix)
            if better < current:
                if self._better_since is None:
                    self._better_since = now
                if now - self._better_since >= self.config.hold_time:
                    new = better
                    self._better_since = None
            else:
                self._better_since = None
        changed = new is not current
        since = now if changed else self.state.since
        self.state = QualityState(
            new, _describe(new, initialised, sigma, since_fix, self.config), since, changed)
        return self.state
