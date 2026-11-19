"""GPS error model for the simulator (Step 8).

A real receiver's error is a mix of effects with different time scales:

* bias: a slowly wandering offset (first-order Gauss-Markov process per axis),
* jitter: fresh white noise on every fix,
* multipath jumps: sudden offsets of 10-30 m held for a few seconds, during
  which the receiver still reports its normal (too small) uncertainty,
* dropouts: short periods with no fix,
* no-fix zones (tunnels): no fix at all for as long as the robot is inside.

All randomness comes from one seeded ``random.Random``, so a run can be
repeated exactly. Positions are metres in the local ENU frame (x = east,
y = north). The model never imports rclpy; the sim node copies readings into
NavSatFix messages.
"""

from dataclasses import dataclass, fields, replace
import math
import random


@dataclass(frozen=True)
class GpsConditions:
    """Error parameters for one kind of environment (open sky, canyon, tunnel)."""

    # Standard deviation of the slowly wandering bias, per axis (m).
    bias_sigma: float = 0.0
    # How long the bias takes to wander (s); larger = slower.
    bias_time_constant: float = 120.0
    # Standard deviation of the per-fix white noise, per axis (m).
    jitter_sigma: float = 0.0
    # Average number of multipath jumps starting per minute.
    jumps_per_minute: float = 0.0
    jump_size_min: float = 10.0
    jump_size_max: float = 30.0
    jump_duration_min: float = 2.0
    jump_duration_max: float = 10.0
    # Average number of dropouts (no fix) starting per minute.
    dropouts_per_minute: float = 0.0
    dropout_duration_min: float = 2.0
    dropout_duration_max: float = 8.0
    # Horizontal sigma the receiver reports (m); unaware of jumps.
    reported_sigma: float = 2.5
    # False inside tunnels: no fix at all.
    fix_available: bool = True

    def __post_init__(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if f.name != 'fix_available' and value < 0.0:
                raise ValueError(f'{f.name} must be >= 0, got {value}')
        if self.bias_time_constant <= 0.0:
            raise ValueError('bias_time_constant must be > 0')
        if self.reported_sigma <= 0.0:
            raise ValueError('reported_sigma must be > 0')
        for name in ('jump_size', 'jump_duration', 'dropout_duration'):
            low = getattr(self, name + '_min')
            high = getattr(self, name + '_max')
            if low > high:
                raise ValueError(f'{name}_min ({low}) > {name}_max ({high})')


# Named starting values; tune once the team knows the receiver.
PROFILES = {
    'perfect': GpsConditions(reported_sigma=0.5),
    'open': GpsConditions(
        bias_sigma=1.5, bias_time_constant=120.0, jitter_sigma=0.5,
        dropouts_per_minute=0.05, reported_sigma=2.5),
    'urban_canyon': GpsConditions(
        bias_sigma=3.0, bias_time_constant=60.0, jitter_sigma=1.0,
        jumps_per_minute=0.3, dropouts_per_minute=0.5, reported_sigma=5.0),
    'no_fix': GpsConditions(fix_available=False),
}


def conditions_from_dict(values, base=None):
    """Return ``base`` (default: the 'perfect' profile) with fields overridden.

    ``values`` may contain ``profile: <name>`` to start from a named profile.
    Unknown keys raise ValueError so typos in YAML files are caught.
    """
    values = dict(values)
    if 'profile' in values:
        name = values.pop('profile')
        if name not in PROFILES:
            raise ValueError(f'unknown GPS profile {name!r}; known: {sorted(PROFILES)}')
        base = PROFILES[name]
    if base is None:
        base = PROFILES['perfect']
    known = {f.name for f in fields(GpsConditions)}
    unknown = sorted(set(values) - known)
    if unknown:
        raise ValueError(f'unknown GPS condition fields: {unknown}')
    return replace(base, **values)


@dataclass(frozen=True)
class GpsReading:
    """One simulated receiver output (plus truth flags for tests and evaluation)."""

    available: bool
    east: float | None
    north: float | None
    reported_sigma: float
    # True while a multipath jump is active (the receiver does not know this).
    jumping: bool = False
    # True while a random dropout is active (not set inside no-fix zones).
    dropout: bool = False


def _event_probability(per_minute, dt):
    """Return the chance that a Poisson event with this rate starts within dt."""
    if per_minute <= 0.0 or dt <= 0.0:
        return 0.0
    return 1.0 - math.exp(-per_minute / 60.0 * dt)


class GpsErrorModel:
    """Stateful GPS error generator; call ``read`` once per GPS fix."""

    def __init__(self, seed=0):
        self._rng = random.Random(seed)
        self._last_time = None
        self._bias_e = None
        self._bias_n = None
        self._jump_until = -math.inf
        self._jump_e = 0.0
        self._jump_n = 0.0
        self._dropout_until = -math.inf

    @property
    def bias(self):
        """Return the current bias as (east, north) in metres, for tests."""
        return (self._bias_e or 0.0, self._bias_n or 0.0)

    def read(self, true_east, true_north, t, conditions):
        """Return the receiver output at sim time ``t`` (s) for the true position."""
        if self._last_time is not None and t < self._last_time:
            raise ValueError(f'time went backwards: {t} < {self._last_time}')
        dt = 0.0 if self._last_time is None else t - self._last_time
        self._last_time = t
        c = conditions

        self._update_bias(dt, c)

        if not c.fix_available:
            # Tunnel: no fix; any jump in progress ends with the signal.
            self._jump_until = -math.inf
            return GpsReading(False, None, None, c.reported_sigma)

        if t >= self._dropout_until and \
                self._rng.random() < _event_probability(c.dropouts_per_minute, dt):
            self._dropout_until = t + self._rng.uniform(
                c.dropout_duration_min, c.dropout_duration_max)
        if t < self._dropout_until:
            return GpsReading(False, None, None, c.reported_sigma, dropout=True)

        if t >= self._jump_until and \
                self._rng.random() < _event_probability(c.jumps_per_minute, dt):
            size = self._rng.uniform(c.jump_size_min, c.jump_size_max)
            direction = self._rng.uniform(-math.pi, math.pi)
            self._jump_e = size * math.cos(direction)
            self._jump_n = size * math.sin(direction)
            self._jump_until = t + self._rng.uniform(
                c.jump_duration_min, c.jump_duration_max)
        jumping = t < self._jump_until

        east = true_east + self._bias_e + self._rng.gauss(0.0, 1.0) * c.jitter_sigma
        north = true_north + self._bias_n + self._rng.gauss(0.0, 1.0) * c.jitter_sigma
        if jumping:
            east += self._jump_e
            north += self._jump_n
        return GpsReading(True, east, north, c.reported_sigma, jumping=jumping)

    def _update_bias(self, dt, c):
        """Advance the Gauss-Markov bias by dt (exact discretisation)."""
        if self._bias_e is None:
            self._bias_e = self._rng.gauss(0.0, 1.0) * c.bias_sigma
            self._bias_n = self._rng.gauss(0.0, 1.0) * c.bias_sigma
            return
        if dt <= 0.0:
            return
        decay = math.exp(-dt / c.bias_time_constant)
        step = c.bias_sigma * math.sqrt(1.0 - decay * decay)
        self._bias_e = self._bias_e * decay + self._rng.gauss(0.0, 1.0) * step
        self._bias_n = self._bias_n * decay + self._rng.gauss(0.0, 1.0) * step
