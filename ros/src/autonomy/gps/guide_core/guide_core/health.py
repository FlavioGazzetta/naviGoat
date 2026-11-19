"""Health of the GPS layer: can a GPS trip start now, and if not, why (Step 10).

Pure Python (no ROS). The gps_bridge node collects what it can see (when the
last GPS message and fix arrived, the localizer's latest status, which of our
servers and Nav2's are up) into HealthInputs once a second; evaluate() turns
that into a Health: one Part per piece, whether a trip could start, and one
sentence saying why not. The node publishes it as JSON on /gps/health.

Parts that block a start: the mission manager, the position, the route
planner and Nav2. The GPS itself never blocks once a position exists: in a
tunnel SLAM keeps the position. The geocoder never blocks either: names in
the curated list still work without it.
"""

from dataclasses import dataclass
import json
import math

from guide_core.localization_quality import Quality

# Parts in the order they are reported, and in which a blocking part explains
# the refusal (the first one that blocks wins).
PART_NAMES = ('mission', 'position', 'planner', 'nav2', 'gps', 'geocoder')

READY_TEXT = 'Ready.'


@dataclass(frozen=True)
class HealthConfig:
    """Timeouts (s) after which a silent input counts as gone."""

    gps_timeout: float = 2.0      # receivers send 1-10 fixes a second
    status_timeout: float = 2.0   # the localizer sends its status at 2 Hz

    def __post_init__(self):
        for name in ('gps_timeout', 'status_timeout'):
            if not getattr(self, name) > 0.0:
                raise ValueError(f'{name} must be > 0')


@dataclass(frozen=True)
class HealthInputs:
    """What the node knows at time now (all times in seconds, same clock).

    gps_message_time / gps_fix_time: when the last NavSatFix arrived, and the
    last one with a fix (status >= STATUS_FIX); None = never.
    status_time: when the last guide/localization_status arrived (None =
    never), with its quality and initialised flag.
    nav2_required: False when the route is only published as waypoints for
    someone else to drive: Nav2 missing is then reported but blocks nothing.
    ever_localised: the position has been usable (GOOD or DEGRADED) at least
    once. The localizer starts at LOST and improves only after a hold time,
    so LOST before that means "still settling", not "lost".
    """

    now: float
    gps_topic: str = '/gps/fix'
    gps_message_time: float = None
    gps_fix_time: float = None
    status_time: float = None
    quality: int = Quality.LOST
    initialised: bool = False
    ever_localised: bool = False
    mission_ready: bool = False
    planner_ready: bool = False
    nav2_ready: bool = False
    nav2_required: bool = True   # False: a trip may start without Nav2 (waypoints only)
    geocoder_ready: bool = False


@dataclass(frozen=True)
class Part:
    """One piece of the GPS layer: ready or not, a short detail, and whether it blocks."""

    name: str
    ok: bool
    detail: str
    blocks: bool = False


@dataclass(frozen=True)
class Health:
    """The whole picture: ready, the sentence for the user, and every part."""

    ready: bool
    summary: str
    parts: tuple

    def part(self, name):
        """Return the Part called name."""
        return next(p for p in self.parts if p.name == name)

    def to_json(self):
        """Return the /gps/health message text (keys in a fixed order)."""
        return json.dumps({
            'ready': self.ready,
            'summary': self.summary,
            'parts': {p.name: {'ok': p.ok, 'detail': p.detail} for p in self.parts},
        })


def _age(now, then):
    return math.inf if then is None else now - then


def gps_part(inputs, config):
    """Return the gps Part: whether the receiver is sending fixes."""
    topic = inputs.gps_topic
    message_age = _age(inputs.now, inputs.gps_message_time)
    if math.isinf(message_age):
        return Part('gps', False, f'no messages on {topic}')
    if message_age > config.gps_timeout:
        return Part('gps', False, f'no recent messages on {topic}')
    if _age(inputs.now, inputs.gps_fix_time) > config.gps_timeout:
        return Part('gps', False, 'GPS has no fix')
    return Part('gps', True, 'receiving fixes')


def position_part(inputs, config):
    """Return the position Part: whether the localizer has a usable position.

    The details hold no numbers that change by themselves (the localizer's
    reason does: "position uncertainty 0.6 m"), so the topic stays quiet
    until the situation changes; guide/localization_status has the numbers.
    """
    if _age(inputs.now, inputs.status_time) > config.status_timeout:
        return Part('position', False, 'localizer is not running', blocks=True)
    if not inputs.initialised or (inputs.quality == Quality.LOST
                                  and not inputs.ever_localised):
        return Part('position', False, 'not localised yet', blocks=True)
    if inputs.quality == Quality.LOST:
        return Part('position', False, 'position lost', blocks=True)
    if inputs.quality == Quality.DEGRADED:
        return Part('position', True, 'GPS weak, position less certain')
    return Part('position', True, 'good')


def server_part(name, ready, what, blocks=True):
    """Return the Part for one of the servers (an action or a service)."""
    if ready:
        return Part(name, True, 'running')
    return Part(name, False, f'{what} is not running', blocks=blocks)


def refusal(part, gps):
    """Return the spoken sentence for a blocking part (gps: the gps Part)."""
    if part.name == 'mission':
        return "GPS navigation isn't running."
    if part.name == 'planner':
        return "I can't plan a route: the route planner isn't running."
    if part.name == 'nav2':
        return "I can't drive: Nav2 isn't running."
    # position
    if part.detail == 'localizer is not running':
        return "I can't navigate by GPS: the localizer isn't running."
    if part.detail == 'not localised yet':
        if not gps.ok:
            return "I can't navigate by GPS yet: there is no GPS signal."
        return "I can't navigate by GPS yet: I'm still working out where we are."
    return "I can't navigate by GPS: I've lost our position."


def evaluate(inputs, config=None):
    """Return the Health for these inputs."""
    config = config or HealthConfig()
    gps = gps_part(inputs, config)
    parts = {
        'mission': server_part('mission', inputs.mission_ready, 'mission manager'),
        'position': position_part(inputs, config),
        'planner': server_part('planner', inputs.planner_ready, 'route planner'),
        'nav2': server_part('nav2', inputs.nav2_ready, 'Nav2', blocks=inputs.nav2_required),
        'gps': gps,
        'geocoder': server_part('geocoder', inputs.geocoder_ready, 'geocoder', blocks=False),
    }
    ordered = tuple(parts[name] for name in PART_NAMES)
    blocking = [p for p in ordered if p.blocks]
    if blocking:
        return Health(False, refusal(blocking[0], gps), ordered)
    return Health(True, READY_TEXT, ordered)
