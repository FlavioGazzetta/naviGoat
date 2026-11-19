"""Road crossings along a route: data, validation, route-file I/O, stop points.

Pure Python, no ROS. All positions are arc length s (m) along a Route,
measured from the route's first point.

Shared by the route planner (which finds crossings), plan_route --save (which
writes them to a route file), the route follower (which stops before them)
and, from Step 7, the mission manager (which gets them from PlanRoute).
"""

from dataclasses import dataclass
import math

from guide_core.route import InvalidRouteError
from guide_core.walk_costs import CrossingType

#: Allowed values of RouteCrossing.crossed_road.
CROSSED_ROAD_VALUES = ('major', 'minor', 'none', 'unknown')

#: Distance (m) a crossing may stick out past either end of the route and
#: still be accepted (it is clamped). Covers float rounding and the 1 mm
#: rounding of route files.
END_TOLERANCE = 1e-3

# Keys of one entry in a route file's 'crossings' list.
_REQUIRED_KEYS = ('start_s', 'end_s', 'type', 'crossed_road', 'tactile_paving')
_OPTIONAL_KEYS = ('has_island',)

# When crossings are merged, the merged one takes the "worst" of each value.
_TYPE_RANK = {CrossingType.SIGNALLED: 0, CrossingType.UNKNOWN: 1,
              CrossingType.UNSIGNALLED: 2}
_ROAD_RANK = {'none': 0, 'minor': 1, 'unknown': 2, 'major': 3}


class CrossingError(InvalidRouteError):
    """Crossing data is invalid.

    A subclass of InvalidRouteError, so code that already handles bad route
    files also handles bad crossings.
    """


@dataclass(frozen=True)
class RouteCrossing:
    """One road crossing along a route.

    start_s / end_s: distance (m) along the route of the near and far kerb.
    crossing_type: a guide_core.walk_costs.CrossingType.
    crossed_road: 'major', 'minor', 'none' (no public road, e.g. a cycle
    track) or 'unknown'.
    tactile_paving: tactile paving is mapped at the crossing.
    has_island: the crossing is (part of) a crossing with a pedestrian island.
    """

    start_s: float
    end_s: float
    crossing_type: object  # guide_core.walk_costs.CrossingType
    crossed_road: str
    tactile_paving: bool
    has_island: bool = False


def _is_finite_number(value):
    """Return True for a finite int/float that is not a bool."""
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def _check_crossing(crossing, total_length):
    """Validate one crossing; return it with s values clamped to the route."""
    if not isinstance(crossing, RouteCrossing):
        raise CrossingError(
            f'expected a RouteCrossing, got {type(crossing).__name__}')
    if not (_is_finite_number(crossing.start_s) and _is_finite_number(crossing.end_s)):
        raise CrossingError(
            f'crossing start_s/end_s must be finite numbers, got '
            f'{crossing.start_s!r} / {crossing.end_s!r}')
    if crossing.start_s >= crossing.end_s:
        raise CrossingError(
            f'crossing start_s ({crossing.start_s}) must be before end_s '
            f'({crossing.end_s})')
    if crossing.start_s < -END_TOLERANCE or crossing.end_s > total_length + END_TOLERANCE:
        raise CrossingError(
            f'crossing {crossing.start_s}-{crossing.end_s} m is outside the '
            f'route (0-{total_length} m)')
    if not isinstance(crossing.crossing_type, CrossingType):
        raise CrossingError(
            f'crossing_type must be a CrossingType, got {crossing.crossing_type!r}')
    if crossing.crossed_road not in CROSSED_ROAD_VALUES:
        raise CrossingError(
            f'crossed_road must be one of {CROSSED_ROAD_VALUES}, '
            f'got {crossing.crossed_road!r}')
    if not isinstance(crossing.tactile_paving, bool):
        raise CrossingError(
            f'tactile_paving must be true/false, got {crossing.tactile_paving!r}')
    if not isinstance(crossing.has_island, bool):
        raise CrossingError(
            f'has_island must be true/false, got {crossing.has_island!r}')
    start_s = max(0.0, float(crossing.start_s))
    end_s = min(float(total_length), float(crossing.end_s))
    return RouteCrossing(start_s, end_s, crossing.crossing_type, crossing.crossed_road,
                         crossing.tactile_paving, crossing.has_island)


def _merge(a, b):
    """Merge two overlapping crossings into one (worst of each value)."""
    crossing_type = max(a.crossing_type, b.crossing_type, key=_TYPE_RANK.__getitem__)
    crossed_road = max(a.crossed_road, b.crossed_road, key=_ROAD_RANK.__getitem__)
    return RouteCrossing(
        start_s=a.start_s, end_s=max(a.end_s, b.end_s),
        crossing_type=crossing_type, crossed_road=crossed_road,
        tactile_paving=a.tactile_paving and b.tactile_paving,
        has_island=a.has_island or b.has_island)


def prepare_crossings(crossings, total_length):
    """Validate, sort and merge crossings for a route of the given length.

    Every consumer calls this, whatever the crossings came from (route file,
    PlanRoute response), so the follower can rely on the result:
    sorted by start_s, 0 <= start_s < end_s <= total_length, and no two
    crossings overlapping (those are merged: there is nowhere to stop
    between them). Crossings that only touch stay separate: the point where
    they meet is somewhere to stop (e.g. a pedestrian island mapped as a
    single node), and stop_points puts the stop exactly there.

    Args:
        crossings: Iterable of RouteCrossing.
        total_length: Route length (m).

    Returns:
        Tuple of RouteCrossing.

    Raises:
        CrossingError: If any crossing is invalid.
    """
    checked = sorted((_check_crossing(c, total_length) for c in crossings),
                     key=lambda c: (c.start_s, c.end_s))
    merged = []
    for crossing in checked:
        if merged and crossing.start_s < merged[-1].end_s:
            merged[-1] = _merge(merged[-1], crossing)
        else:
            merged.append(crossing)
    return tuple(merged)


def crossings_from_dict(data, route):
    """Read the optional 'crossings' list of parsed route-file data.

    Layout of one entry (has_island is optional, default false)::

        {start_s: 390.2, end_s: 398.5, type: signalled, crossed_road: major,
         tactile_paving: true, has_island: true}

    Args:
        data: The parsed file contents (as given to route_from_dict).
        route: The Route built from the same data (for its length).

    Returns:
        Tuple of RouteCrossing, prepared (see prepare_crossings). Empty if the
        file has no 'crossings' key or it is empty.

    Raises:
        CrossingError: If the list or any entry is invalid.
    """
    if not isinstance(data, dict):
        raise CrossingError(f'route data must be a mapping, got {type(data).__name__}')
    raw = data.get('crossings')
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise CrossingError("route 'crossings' must be a list")
    crossings = []
    for index, entry in enumerate(raw):
        where = f'crossings[{index}]'
        if not isinstance(entry, dict):
            raise CrossingError(f'{where} must be a mapping')
        missing = [k for k in _REQUIRED_KEYS if k not in entry]
        if missing:
            raise CrossingError(f'{where} is missing {missing}')
        unknown = sorted(set(entry) - set(_REQUIRED_KEYS) - set(_OPTIONAL_KEYS))
        if unknown:
            raise CrossingError(f'{where} has unknown keys {unknown}')
        try:
            crossing_type = CrossingType(entry['type'])
        except ValueError:
            names = [t.value for t in CrossingType]
            raise CrossingError(
                f"{where}: type must be one of {names}, got {entry['type']!r}") from None
        try:
            crossings.append(RouteCrossing(
                start_s=entry['start_s'], end_s=entry['end_s'],
                crossing_type=crossing_type, crossed_road=entry['crossed_road'],
                tactile_paving=entry['tactile_paving'],
                has_island=entry.get('has_island', False)))
        except TypeError as error:  # cannot happen with the key checks; be safe
            raise CrossingError(f'{where}: {error}') from None
    try:
        return prepare_crossings(crossings, route.total_length)
    except CrossingError as error:
        raise CrossingError(f'route crossings: {error}') from None


def crossings_to_dicts(crossings):
    """Turn crossings into plain dicts for a route file (inverse of crossings_from_dict).

    s values are rounded to the millimetre to keep files readable.
    """
    return [{'start_s': round(float(c.start_s), 3),
             'end_s': round(float(c.end_s), 3),
             'type': c.crossing_type.value,
             'crossed_road': c.crossed_road,
             'tactile_paving': bool(c.tactile_paving),
             'has_island': bool(c.has_island)}
            for c in crossings]


def stop_points(crossings, margin):
    """Return the s (m) where the robot stops before each crossing.

    stop = start_s - margin, but never below 0 and never in the first half of
    the gap after the previous crossing: on a pedestrian island narrower than
    2 * margin the robot stops in the middle of the island, never closer to
    the road it has just crossed than to the one ahead.

    Args:
        crossings: Prepared crossings (see prepare_crossings).
        margin: Stop distance (m) before the near kerb, >= 0.

    Returns:
        Tuple of floats, one per crossing, in the same order.

    Raises:
        ValueError: If margin is negative or not finite.
    """
    if not _is_finite_number(margin) or margin < 0.0:
        raise ValueError(f'margin must be finite and >= 0, got {margin!r}')
    stops = []
    previous = None
    for crossing in crossings:
        stop = crossing.start_s - margin
        if previous is not None:
            stop = max(stop, (previous.end_s + crossing.start_s) / 2.0)
        stops.append(max(0.0, stop))
        previous = crossing
    return tuple(stops)
