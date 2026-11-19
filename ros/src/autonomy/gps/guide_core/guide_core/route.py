"""Route geometry: a polyline of waypoints in the local map frame (metres).

A route is an ordered list of (x, y) waypoints, x = east and y = north,
joined by straight segments. Distances "along the route" are measured from
the first waypoint and are called s throughout guide_core.
"""

import bisect
from dataclasses import dataclass
import math
from numbers import Real
from typing import Any, Iterable, Optional, Sequence

from guide_core.angles import wrap_angle

#: Consecutive waypoints closer than this (m) are treated as duplicates.
MIN_SEGMENT_LENGTH = 1e-6

Point = tuple[float, float]


class InvalidRouteError(ValueError):
    """Raised when route data cannot form a usable route."""


@dataclass(frozen=True)
class Projection:
    """The closest point on a route to a query position.

    Attributes:
        s: Distance along the route from its start to the closest point (m).
        segment_index: Index of the segment containing the closest point.
        point: The closest point (x, y) on the route.
        cross_track_error: Signed distance from the query position to the
            closest point (m). Positive = left of the route in the direction
            of travel, negative = right (counter-clockwise positive, REP 103).
    """

    s: float
    segment_index: int
    point: Point
    cross_track_error: float


def _to_point(raw: Any, index: int) -> Point:
    """Validate one raw waypoint and return it as an (x, y) tuple of floats.

    Args:
        raw: The waypoint as given, expected to be a 2-element sequence.
        index: Position of the waypoint in the input, used in error messages.

    Raises:
        InvalidRouteError: If the waypoint is not two finite real numbers.
    """
    # Strings are sequences in Python, so exclude them explicitly.
    if isinstance(raw, (str, bytes)) or not isinstance(raw, Sequence) or len(raw) != 2:
        raise InvalidRouteError(f'waypoint {index} must be [x, y], got {raw!r}')
    coords = []
    for value in raw:
        # bool is a subclass of int, but True/False is never a coordinate.
        if isinstance(value, bool) or not isinstance(value, Real):
            raise InvalidRouteError(
                f'waypoint {index} has a non-numeric coordinate: {raw!r}')
        value = float(value)
        if not math.isfinite(value):
            raise InvalidRouteError(
                f'waypoint {index} has a non-finite coordinate: {raw!r}')
        coords.append(value)
    return (coords[0], coords[1])


class Route:
    """An immutable polyline route in the map frame.

    Attributes are exposed as read-only properties backed by tuples, so a
    Route cannot change after construction.

    Args:
        waypoints: Iterable of [x, y] pairs in metres. Consecutive duplicates
            are removed; at least two distinct waypoints must remain.

    Raises:
        InvalidRouteError: If any waypoint is invalid or fewer than two
            distinct waypoints remain.
    """

    def __init__(self, waypoints: Iterable[Sequence[float]]):
        points: list[Point] = []
        for index, raw in enumerate(waypoints):
            point = _to_point(raw, index)
            # Skip a point that repeats the previous one: it would create a
            # zero-length segment with no direction.
            if points and math.dist(points[-1], point) < MIN_SEGMENT_LENGTH:
                continue
            points.append(point)

        if len(points) < 2:
            raise InvalidRouteError(
                f'a route needs at least 2 distinct waypoints, got {len(points)}')

        segment_lengths = tuple(
            math.dist(start, end) for start, end in zip(points, points[1:]))

        cumulative = [0.0]
        for length in segment_lengths:
            cumulative.append(cumulative[-1] + length)

        # Unit direction vector of each segment, computed once.
        directions = tuple(
            ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
            for start, end, length in zip(points, points[1:], segment_lengths))

        # Heading of each segment: 0 = east, counter-clockwise positive (REP 103).
        headings = tuple(math.atan2(dy, dx) for dx, dy in directions)

        self._points = tuple(points)
        self._segment_lengths = segment_lengths
        self._cumulative = tuple(cumulative)
        self._directions = directions
        self._headings = headings

    @property
    def points(self) -> tuple[Point, ...]:
        """Waypoints as (x, y) tuples, after duplicate removal."""
        return self._points

    @property
    def segment_lengths(self) -> tuple[float, ...]:
        """Length of each segment; segment i joins points[i] and points[i + 1]."""
        return self._segment_lengths

    @property
    def cumulative(self) -> tuple[float, ...]:
        """Distance along the route (s) at each waypoint; starts at 0."""
        return self._cumulative

    @property
    def total_length(self) -> float:
        """Total length of the route in metres."""
        return self._cumulative[-1]

    @property
    def num_segments(self) -> int:
        """Return the number of segments (one fewer than the number of points)."""
        return len(self._segment_lengths)

    def project(self, x: float, y: float,
                s_min: Optional[float] = None,
                s_max: Optional[float] = None) -> Projection:
        """Find the closest point on the route to (x, y).

        Only the part of the route with s in [s_min, s_max] is searched; this
        stops the projection jumping to a different part of a route that
        passes close to itself. The window is clamped to [0, total_length].
        When two segments are equally close, the earlier one wins.

        Args:
            x: Query position east (m).
            y: Query position north (m).
            s_min: Start of the search window (m); None = route start.
            s_max: End of the search window (m); None = route end.

        Raises:
            ValueError: If both window limits are given and s_min > s_max.
        """
        if s_min is not None and s_max is not None and s_min > s_max:
            raise ValueError(f's_min ({s_min}) must not exceed s_max ({s_max})')
        total = self.total_length
        lo = 0.0 if s_min is None else min(max(s_min, 0.0), total)
        hi = total if s_max is None else min(max(s_max, 0.0), total)

        best: Optional[Projection] = None
        best_distance = math.inf
        for i, (ax, ay) in enumerate(self._points[:-1]):
            seg_start = self._cumulative[i]
            seg_end = self._cumulative[i + 1]
            if seg_end < lo or seg_start > hi:
                continue  # segment lies entirely outside the window

            dx, dy = self._directions[i]
            # Dot product: how far along this segment the query point lies,
            # clamped to the part of the segment inside the window.
            along = (x - ax) * dx + (y - ay) * dy
            along_min = max(0.0, lo - seg_start)
            along_max = min(self._segment_lengths[i], hi - seg_start)
            along = min(max(along, along_min), along_max)

            px = ax + dx * along
            py = ay + dy * along
            distance = math.hypot(x - px, y - py)
            if distance < best_distance:  # strict: ties keep the earlier segment
                # Cross product: positive when the query point is left of d.
                cross = dx * (y - ay) - dy * (x - ax)
                sign = -1.0 if cross < 0.0 else 1.0
                best_distance = distance
                best = Projection(
                    s=seg_start + along,
                    segment_index=i,
                    point=(px, py),
                    cross_track_error=sign * distance,
                )

        # The clamped window always overlaps at least one segment.
        assert best is not None
        return best

    def segment_index_at(self, s: float) -> int:
        """Return the index of the segment containing distance s.

        At a waypoint, the segment that starts there (outgoing) is returned,
        except at the route end, which belongs to the last segment. s outside
        [0, total_length] is clamped.
        """
        # bisect_right finds where s would be inserted into the sorted
        # cumulative distances; one less is the segment that starts at or before s.
        index = bisect.bisect_right(self._cumulative, s) - 1
        return min(max(index, 0), self.num_segments - 1)

    def point_at(self, s: float) -> Point:
        """Return the point at distance s along the route (s is clamped)."""
        s = min(max(s, 0.0), self.total_length)
        i = self.segment_index_at(s)
        along = s - self._cumulative[i]
        ax, ay = self._points[i]
        dx, dy = self._directions[i]
        return (ax + dx * along, ay + dy * along)

    def heading_at(self, s: float) -> float:
        """Return the route heading (rad, REP 103) at distance s.

        At a waypoint this is the heading of the outgoing segment.
        """
        return self._headings[self.segment_index_at(s)]

    def turn_angle(self, index: int) -> float:
        """Return the signed turn at interior waypoint index, in (-pi, pi].

        Positive = left turn, negative = right turn.

        Raises:
            IndexError: If index is not an interior waypoint (the first and
                last waypoints have no turn).
        """
        if not 1 <= index <= len(self._points) - 2:
            raise IndexError(
                f'turn_angle needs an interior waypoint index '
                f'(1..{len(self._points) - 2}), got {index}')
        return wrap_angle(self._headings[index] - self._headings[index - 1])

    def corners(self, min_turn_angle: float) -> tuple[int, ...]:
        """Return the indices of waypoints that turn by more than min_turn_angle.

        Args:
            min_turn_angle: Threshold in radians (>= 0); the absolute turn
                must be strictly greater than this to count as a corner.

        Raises:
            ValueError: If min_turn_angle is negative.
        """
        if min_turn_angle < 0.0:
            raise ValueError(f'min_turn_angle must be >= 0, got {min_turn_angle}')
        return tuple(
            i for i in range(1, len(self._points) - 1)
            if abs(self.turn_angle(i)) > min_turn_angle)


def route_from_dict(data: Any, expected_frame: str = 'map') -> Route:
    """Build a Route from parsed route-file data.

    Expected layout (as produced by yaml.safe_load)::

        {'frame_id': 'map', 'waypoints': [[0.0, 0.0], [40.0, 0.0], ...]}

    Args:
        data: The parsed file contents.
        expected_frame: The frame the waypoints must be expressed in.

    Raises:
        InvalidRouteError: If the layout is wrong, the frame does not match,
            or the waypoints are invalid.
    """
    if not isinstance(data, dict):
        raise InvalidRouteError(
            f'route data must be a mapping, got {type(data).__name__}')
    frame_id = data.get('frame_id')
    if frame_id != expected_frame:
        raise InvalidRouteError(
            f'route frame_id is {frame_id!r}, expected {expected_frame!r}')
    waypoints = data.get('waypoints')
    if not isinstance(waypoints, list):
        raise InvalidRouteError("route data needs a 'waypoints' list")
    return Route(waypoints)
