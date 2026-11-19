"""Route following: track progress along a Route and choose goals for Nav2.

A RouteFollower is created for one route. Each pose update projects the
robot onto the route, reports progress, and decides whether Nav2 should be
sent a new goal. Goals lie on the route a lookahead distance ahead, but are
held at sharp corners so the local planner never cuts across them, and at a
stop point before every road crossing, where the robot waits for the user.
Only the user releases a crossing (confirm_crossing); the goal is then held
at the far kerb, so Nav2 crosses straight over, before normal goals resume.

Under position uncertainty (Step 8), update() can also be given the robot's
position sigma and the time:
* the stop before the next crossing moves earlier by about 3 sigma, so an
  uncertain position can never carry the robot onto the road unconfirmed;
* the arrival tolerance grows a little with sigma;
* off-route detection: off_route becomes True once the robot has been more
  than off_route_distance from the route for off_route_time seconds, and False
  again below back_on_route_distance (never judged on or at a crossing, or
  while sigma is above off_route_max_sigma).
Without sigma and time the follower behaves exactly as before Step 8.
"""

from dataclasses import dataclass
import math
from typing import Optional

from guide_core.crossings import prepare_crossings, RouteCrossing, stop_points
from guide_core.route import Route

#: Goals whose distance along the route differs by less than this (m) are the same goal.
_SAME_S = 1e-9
#: Goal positions closer than this (m) are the same goal. Positions are copied
#: exactly through ROS messages, so this only absorbs floating-point rounding.
_SAME_POINT = 1e-3
#: How far short of a stop point (m) the robot may halt and still count as
#: waiting there: Nav2 stops ~0.25 m short of a goal, the sim 0.3 m.
_STOP_SHORT = 0.5


@dataclass(frozen=True)
class FollowerConfig:
    """Tuning values for RouteFollower (all distances in metres).

    Attributes:
        lookahead: How far ahead of the robot, along the route, goals are placed.
        corner_angle: A waypoint turning by more than this (rad) is a corner.
        corner_release_distance: How close (along the route) the robot must get
            to a corner before the goal may move past it.
        goal_update_distance: Minimum movement of the goal along the route before
            a new goal is sent (corner and final goals are sent regardless).
        arrival_tolerance: The robot has arrived when both the distance remaining
            along the route and the straight-line distance to the end are within this.
        window_behind: The projection search window starts this far behind the last s.
        window_ahead: The projection search window ends this far ahead of the last s.
        crossing_stop_margin: The robot stops this far before a crossing's near
            kerb (but never in the first half of a pedestrian island, see
            guide_core.crossings.stop_points). May be 0.
        crossing_wait_distance: The robot is waiting at a crossing once it is
            within this distance (along the route) of the stop point.
        stop_sigma_factor, stop_sigma_included, stop_sigma_step: the stop point
            moves earlier by stop_sigma_factor * (sigma - stop_sigma_included),
            rounded up to a multiple of stop_sigma_step (none if negative).
        stop_latch_distance: Within this distance before the (static) stop point
            the shift only ever grows, so the stop goal cannot creep back and forth.
        arrival_sigma_factor: The arrival tolerance is
            max(arrival_tolerance, arrival_sigma_factor * sigma).
        off_route_distance, back_on_route_distance: Cross-track error (m) above
            which the robot counts as off the route (after off_route_time
            seconds), and below which it is back on it.
        off_route_time: Seconds the robot must stay beyond off_route_distance.
        off_route_max_sigma: Above this sigma (m) off-route is not judged (the
            state is kept as it was): the error is then more likely ours.

    Raises:
        ValueError: If a distance is not positive and finite (crossing_stop_margin:
            not >= 0 and finite), or corner_angle is outside [0, pi].
    """

    lookahead: float = 25.0
    corner_angle: float = math.radians(30.0)
    corner_release_distance: float = 1.5
    goal_update_distance: float = 5.0
    arrival_tolerance: float = 1.0
    window_behind: float = 5.0
    window_ahead: float = 30.0
    crossing_stop_margin: float = 1.5
    crossing_wait_distance: float = 1.5
    # 3: the along-route error of an honest filter exceeds 3 sigma only ~0.1 %
    # of the time (2 sigma: ~2 %, too often for "never"; Step 8 M4 runs).
    stop_sigma_factor: float = 3.0
    stop_sigma_included: float = 0.5
    stop_sigma_step: float = 0.5
    stop_latch_distance: float = 15.0
    arrival_sigma_factor: float = 0.5
    off_route_distance: float = 10.0
    back_on_route_distance: float = 5.0
    off_route_time: float = 5.0
    off_route_max_sigma: float = 5.0

    def __post_init__(self):
        for name in ('lookahead', 'corner_release_distance', 'goal_update_distance',
                     'arrival_tolerance', 'window_behind', 'window_ahead',
                     'crossing_wait_distance'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be a positive finite number, got {value}')
        if not (math.isfinite(self.crossing_stop_margin) and self.crossing_stop_margin >= 0.0):
            raise ValueError(f'crossing_stop_margin must be finite and >= 0, '
                             f'got {self.crossing_stop_margin}')
        if not 0.0 <= self.corner_angle <= math.pi:  # also rejects NaN
            raise ValueError(f'corner_angle must be in [0, pi], got {self.corner_angle}')
        for name in ('stop_sigma_step', 'off_route_distance', 'back_on_route_distance',
                     'off_route_max_sigma'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be a positive finite number, got {value}')
        for name in ('stop_sigma_factor', 'stop_sigma_included', 'stop_latch_distance',
                     'arrival_sigma_factor', 'off_route_time'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f'{name} must be finite and >= 0, got {value}')
        if not self.back_on_route_distance < self.off_route_distance:
            raise ValueError('back_on_route_distance must be < off_route_distance')


@dataclass(frozen=True)
class Goal:
    """A goal pose on the route, in the map frame.

    Attributes:
        x: East (m).
        y: North (m).
        yaw: Heading of the route at the goal (rad, REP 103); at a corner, the
            outgoing direction; at a crossing stop, the direction across the
            crossing (near kerb to far kerb).
        s: Distance along the route of the goal (m).
        is_corner: True if the goal is being held at a corner.
        is_final: True if the goal is the end of the route.
        is_crossing_stop: True if the goal is the stop point before a crossing.
        is_crossing_exit: True if the goal is held at the far kerb of a crossing
            the user has confirmed.
    """

    x: float
    y: float
    yaw: float
    s: float
    is_corner: bool
    is_final: bool
    is_crossing_stop: bool = False
    is_crossing_exit: bool = False


@dataclass(frozen=True)
class FollowerOutput:
    """Result of one RouteFollower.update() call.

    The progress fields match guide_interfaces/msg/RouteProgress.

    Attributes:
        s: Distance along the route of the robot's projected position (m).
        segment_index: Segment the robot is projected onto.
        cross_track_error: Signed distance from the route (m), + = left.
        distance_remaining: Distance along the route still to go (m).
        total_length: Total route length (m).
        arrived: True once the robot has reached the end (latched).
        new_goal: A goal to send to Nav2 now, or None if the current one stands.
        waiting_at_crossing: True while the robot is at the stop point of the
            next crossing, waiting for the user to confirm.
        on_crossing: True while the robot is between the near and far kerb of
            a crossing the user has confirmed.
        next_crossing_index: Index (in RouteFollower.crossings) of the first
            crossing not yet confirmed, or -1 if none are left.
        distance_to_next_crossing: That crossing's start_s (near kerb) minus the
            robot's s (m); negative once past the near kerb; 0.0 if none left.
        off_route: True while the robot counts as off the route (see the module
            docstring); always False when update() is not given the time.
    """

    s: float
    segment_index: int
    cross_track_error: float
    distance_remaining: float
    total_length: float
    arrived: bool
    new_goal: Optional[Goal]
    waiting_at_crossing: bool = False
    on_crossing: bool = False
    next_crossing_index: int = -1
    distance_to_next_crossing: float = 0.0
    off_route: bool = False


class RouteFollower:
    """Follows one route: reports progress and chooses goals.

    Create a new RouteFollower for each new route; there is no reset.

    Args:
        route: The route to follow.
        config: Tuning values; defaults to FollowerConfig().
        crossings: Road crossings along the route (RouteCrossing). They are
            validated, sorted and merged with prepare_crossings. Empty (the
            default): the follower behaves exactly as without crossings.

    Raises:
        guide_core.crossings.CrossingError: If a crossing is invalid.
    """

    def __init__(self, route: Route, config: FollowerConfig = FollowerConfig(),
                 crossings=()):
        self._route = route
        self._config = config
        # Distance along the route of every corner, in order.
        self._corner_s = tuple(
            route.cumulative[i] for i in route.corners(config.corner_angle))
        self._crossings = prepare_crossings(crossings, route.total_length)
        # Distance along the route of the stop point before each crossing.
        self._stop_s = stop_points(self._crossings, config.crossing_stop_margin)
        self._last_s: Optional[float] = None   # None until the first update
        self._released_s = -math.inf           # corners at or before this s are released
        self._goal: Optional[Goal] = None      # last goal handed out
        self._arrived = False
        self._next_crossing = 0                # index of the first crossing not yet released
        self._stop_reached = False             # Nav2 reported the current stop goal reached
        self._exit_s: Optional[float] = None   # far kerb of a confirmed crossing, until released
        # Uncertainty handling (Step 8) for the next crossing's stop:
        # _stop_offset: how far (m) the stop has been moved earlier;
        # _stop_floor: s where the robot already was when a larger offset put the
        #   stop behind it (it then stops there instead of being sent back).
        self._stop_offset = 0.0
        self._stop_floor: Optional[float] = None
        self._off_route = False
        self._off_since: Optional[float] = None   # time the error first exceeded the limit

    @property
    def route(self) -> Route:
        """Return the route being followed."""
        return self._route

    @property
    def config(self) -> FollowerConfig:
        """Return the tuning values in use."""
        return self._config

    @property
    def goal(self) -> Optional[Goal]:
        """Return the most recent goal handed out, or None before the first one."""
        return self._goal

    @property
    def arrived(self) -> bool:
        """Return True once the robot has reached the end of the route."""
        return self._arrived

    @property
    def crossings(self) -> tuple:
        """Return the crossings in use (validated, sorted, merged)."""
        return self._crossings

    @property
    def stop_points(self) -> tuple:
        """Distance along the route (m) of the stop point before each crossing."""
        return self._stop_s

    @property
    def next_crossing_index(self) -> int:
        """Index of the first crossing not yet confirmed, or -1 if none are left."""
        return self._next_crossing if self._next_crossing < len(self._crossings) else -1

    @property
    def stop_offset(self) -> float:
        """How far (m) the next crossing's stop is moved earlier for uncertainty."""
        return self._stop_offset

    @property
    def active_stop_s(self) -> Optional[float]:
        """Stop point (s) in use for the next crossing (after any shift), or None."""
        return self._active_stop_s()

    @property
    def off_route(self) -> bool:
        """Return True while the robot counts as off the route (last update)."""
        return self._off_route

    @property
    def waiting_at_crossing(self) -> bool:
        """Return True if the robot is waiting at a crossing (last update)."""
        return self._last_s is not None and self._is_waiting(self._last_s)

    def update(self, x: float, y: float, sigma: float = 0.0,
               now: Optional[float] = None) -> FollowerOutput:
        """Process one robot position (route frame, metres).

        Args:
            x, y: The robot's position.
            sigma: Uncertainty (1 sigma, m) of that position; 0 = exact.
            now: Time (s) of the position, for off-route detection; None
                disables it.

        Raises:
            ValueError: If x or y is not finite, or sigma is negative / NaN.
        """
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f'robot position must be finite, got ({x}, {y})')
        if not sigma >= 0.0:
            raise ValueError(f'sigma must be >= 0, got {sigma}')
        cfg = self._config
        route = self._route

        # 1. Project onto the route; after the first update, only search a
        #    window around the last known progress (see Route.project).
        if self._last_s is None:
            projection = route.project(x, y)
        else:
            projection = route.project(
                x, y,
                s_min=self._last_s - cfg.window_behind,
                s_max=self._last_s + cfg.window_ahead)
        s = projection.s
        self._last_s = s
        remaining = route.total_length - s

        # 2. Arrival needs both conditions, so a route that loops back to its
        #    start is not "arrived" at the beginning. Latched once true.
        if not self._arrived:
            tolerance = max(cfg.arrival_tolerance, cfg.arrival_sigma_factor * sigma)
            near_end = math.dist((x, y), route.points[-1]) <= tolerance
            if remaining <= tolerance and near_end:
                self._arrived = True

        # 2b. Move the next crossing's stop earlier if the position is uncertain.
        self._update_stop_offset(s, sigma)

        # 3-5. Choose a goal and decide whether to hand it out.
        new_goal = None
        if not self._arrived:
            self._release_corners(s)
            candidate = self._candidate_goal(s)
            if self._should_send(candidate):
                self._goal = candidate
                new_goal = candidate

        # 6. Crossing state for the progress report.
        next_index = self.next_crossing_index
        last = self._next_crossing - 1       # most recently confirmed crossing
        on_crossing = (last >= 0 and
                       self._crossings[last].start_s <= s <= self._crossings[last].end_s)
        waiting = self._is_waiting(s)

        # 7. Off-route detection (only with a time, see the module docstring).
        self._update_off_route(abs(projection.cross_track_error), sigma, now,
                               on_crossing or waiting)

        return FollowerOutput(
            s=s,
            segment_index=projection.segment_index,
            cross_track_error=projection.cross_track_error,
            distance_remaining=remaining,
            total_length=route.total_length,
            arrived=self._arrived,
            new_goal=new_goal,
            waiting_at_crossing=waiting,
            on_crossing=on_crossing,
            next_crossing_index=next_index,
            distance_to_next_crossing=(
                self._crossings[next_index].start_s - s if next_index >= 0 else 0.0),
            off_route=self._off_route,
        )

    def _update_stop_offset(self, s: float, sigma: float) -> None:
        """Move the next crossing's stop earlier by about 3 sigma (see FollowerConfig).

        The shift grows as soon as sigma calls for more (rounded up to
        stop_sigma_step). It shrinks only when sigma calls for at least two steps
        less, and never within stop_latch_distance of the static stop: a sigma
        hovering at a rounding boundary must not make the stop goal (and Nav2's
        goal) flip back and forth. If a larger shift puts the stop behind the
        robot, the robot stops where it is.
        """
        cfg = self._config
        index = self._next_crossing
        if index >= len(self._stop_s):
            return
        step = cfg.stop_sigma_step
        excess = max(0.0, cfg.stop_sigma_factor * (sigma - cfg.stop_sigma_included))
        wanted = math.ceil(excess / step - 1e-9) * step
        offset = self._stop_offset
        if excess > offset:
            offset = wanted                                  # more caution: at once
        elif (excess <= offset - 2.0 * step
              and self._stop_s[index] - s > cfg.stop_latch_distance):
            offset = wanted                                  # clearly less, and far away
        grew = offset > self._stop_offset
        self._stop_offset = offset
        if grew and s > self._shifted_stop(index):
            self._stop_floor = s if self._stop_floor is None else min(self._stop_floor, s)

    def _shifted_stop(self, index: int) -> float:
        """Return the static stop minus the offset, never before the last far kerb."""
        static = self._stop_s[index]
        stop = static - self._stop_offset
        if index > 0:
            # min(..., static): a static stop already before that far kerb (islands,
            # see stop_points) stays exactly where it is.
            stop = max(stop, min(self._crossings[index - 1].end_s, static))
        return min(stop, static)

    def _update_off_route(self, error: float, sigma: float, now: Optional[float],
                          at_crossing: bool) -> None:
        """Update the off-route state from the cross-track error (with hysteresis)."""
        cfg = self._config
        if now is None or at_crossing or sigma > cfg.off_route_max_sigma:
            self._off_since = None
            return            # not judged: keep the current state
        if error < cfg.back_on_route_distance:
            self._off_route = False
            self._off_since = None
        elif error > cfg.off_route_distance:
            if self._off_since is None:
                self._off_since = now
            if now - self._off_since >= cfg.off_route_time:
                self._off_route = True
        else:
            self._off_since = None   # in between: no change, and the timer restarts

    def confirm_crossing(self) -> bool:
        """Release the crossing the robot is waiting at (the user said go).

        Accepted only while waiting (see FollowerOutput.waiting_at_crossing,
        as of the last update). A go given anywhere else is rejected, never
        remembered: the user's decision is only valid at the kerb, at that
        moment. After a release the goal is held at the far kerb (a corner-like
        hold), then normal goals resume; the next update() sends the new goal.

        Returns:
            True if a crossing was released, False if the go was rejected.
        """
        if self._last_s is None or not self._is_waiting(self._last_s):
            return False
        crossing = self._crossings[self._next_crossing]
        self._next_crossing += 1
        self._stop_reached = False
        self._exit_s = crossing.end_s
        self._stop_offset = 0.0        # the next crossing starts with its own shift
        self._stop_floor = None
        return True

    def confirm_goal_reached(self, x: float, y: float) -> bool:
        """Tell the follower that Nav2 reached the goal at (x, y) (map frame, metres).

        This is the corner safety net. Nav2 stops a little short of a goal
        (~0.25 m) and reports success. If a corner goal is reached like that
        before the robot is within corner_release_distance of it, the goal
        would stay held at the corner forever and the robot would never move
        again. So a confirmed corner goal is released here; the next update()
        then chooses a goal past it.

        Only the current goal counts: a position that is not the current goal
        (a stale or unrelated success) changes nothing. Lookahead and final
        goals change nothing either; arrival is decided by update() alone.

        A crossing stop goal is NEVER released here: only the user may release
        a crossing. Nav2's success only tells the follower the robot is at the
        stop, so it counts as waiting even if GPS puts it a little further back.
        A far-kerb goal (after the user's go) is released like a corner.

        Returns:
            True if a corner or far-kerb hold was released, False otherwise
            (always False for a crossing stop).
        """
        goal = self._goal
        if self._arrived or goal is None:
            return False
        if math.dist((x, y), (goal.x, goal.y)) > _SAME_POINT:
            return False  # not our current goal
        if goal.is_crossing_stop:
            self._stop_reached = True
            return False
        released = False
        if goal.is_crossing_exit and self._exit_s is not None:
            self._exit_s = None
            released = True
        if goal.is_corner and goal.s > self._released_s:
            self._released_s = goal.s
            released = True
        return released

    def _release_corners(self, s: float) -> None:
        """Release every corner the robot is within corner_release_distance of.

        Why the release distance is 1.5 m (see the Step 4 design notes):
        once a corner is released, Nav2 aims at a goal ~lookahead metres down
        the next leg and plans roughly straight to it, cutting inside the
        corner by about corner_release_distance (r * L / sqrt(r^2 + L^2) ~= r
        for a 90 degree turn). So it must be small, or the robot leaves the
        pavement. But it must stay comfortably above Nav2's goal tolerance
        (~0.25 m; the sim stops 0.3 m short), otherwise the robot parks just
        before the corner and it is never released (deadlock), and the
        controller slows to a near-stop at every corner. GPS noise (Step 8)
        also needs margin, so revisit this value then. Distances are measured
        along the route so sideways drift does not affect the release.
        Step 5 adds a safety net: Nav2 reporting SUCCEEDED for a corner goal
        also releases it. If corner cutting is still a problem on the real
        robot, Nav2's NavigateThroughPoses action is the alternative (Step 9).

        Releases are latched (they never un-release), so a pose wobbling
        across the threshold cannot make the goal flip back and forth.

        Corners beyond the stop point of an unreleased crossing are not
        released while the crossing is held, however close the robot is (it
        may wait past the stop point); after the user's go they follow the
        normal rule. Likewise corners beyond the far kerb of a crossing being
        crossed. The far-kerb hold itself is released by the same distance rule.
        """
        release = self._config.corner_release_distance
        limit = self._active_stop_s()
        if self._exit_s is not None:
            if s >= self._exit_s - release:
                self._exit_s = None
            else:
                limit = self._exit_s if limit is None else min(limit, self._exit_s)
        for corner_s in self._corner_s:
            if limit is not None and corner_s > limit:
                break  # corners are in route order: the rest are beyond it too
            if corner_s > self._released_s and s >= corner_s - release:
                self._released_s = corner_s

    def _is_waiting(self, s: float) -> bool:
        """Return True if the robot at progress s waits at the next crossing's stop.

        Waiting starts crossing_wait_distance before the stop point, but never
        before the far kerb of the crossing just crossed: on a narrow island
        the robot must be off the first carriageway before a go is accepted.
        (If the far kerb is the stop point itself, an island mapped as a single
        node, waiting starts _STOP_SHORT before it, where Nav2 halts.)
        Nav2 reporting the stop goal reached also counts as waiting.
        """
        stop = self._active_stop_s()
        if self._arrived or stop is None:
            return False
        if self._stop_reached:
            return True
        threshold = stop - self._config.crossing_wait_distance
        if self._next_crossing > 0:
            far_kerb = self._crossings[self._next_crossing - 1].end_s
            threshold = max(threshold, min(far_kerb, stop - _STOP_SHORT))
        return s >= threshold

    def _active_stop_s(self) -> Optional[float]:
        """Return the stop point (s) in use for the first unreleased crossing, or None.

        That is the static stop point, moved earlier for position uncertainty
        (_update_stop_offset), but never behind where the robot already was.
        """
        index = self._next_crossing
        if index >= len(self._stop_s):
            return None
        stop = self._shifted_stop(index)
        if self._stop_floor is not None:
            stop = min(self._stop_s[index], max(stop, self._stop_floor))
        return stop

    def _crossing_heading(self, crossing: RouteCrossing) -> float:
        """Direction (rad) from the near kerb to the far kerb of a crossing."""
        x0, y0 = self._route.point_at(crossing.start_s)
        x1, y1 = self._route.point_at(crossing.end_s)
        return math.atan2(y1 - y0, x1 - x0)

    def _candidate_goal(self, s: float) -> Goal:
        """Return the goal for progress s: lookahead ahead, held at the first hold.

        Holds are the first unreleased corner, the far kerb of a crossing being
        crossed, and the stop point of the first unreleased crossing; whichever
        comes first along the route wins (a stop at the same s as a corner wins
        too). The goal never passes an unreleased crossing stop.
        """
        route = self._route
        total = route.total_length
        target_s = min(s + self._config.lookahead, total)

        # The first corner not yet released blocks the goal from passing it.
        blocking = next((c for c in self._corner_s if c > self._released_s), None)
        is_corner = blocking is not None and target_s >= blocking
        if is_corner:
            target_s = blocking

        # The far kerb of the crossing being crossed blocks it.
        is_crossing_exit = self._exit_s is not None and target_s >= self._exit_s
        if is_crossing_exit:
            is_corner = is_corner and blocking <= self._exit_s  # a corner exactly there
            target_s = self._exit_s

        # The next crossing's stop point blocks it too.
        stop = self._active_stop_s()
        is_crossing_stop = stop is not None and target_s >= stop
        if is_crossing_stop:
            target_s = stop
            is_corner = False
            is_crossing_exit = False
            yaw = self._crossing_heading(self._crossings[self._next_crossing])
        else:
            yaw = route.heading_at(target_s)

        x, y = route.point_at(target_s)
        return Goal(
            x=x,
            y=y,
            yaw=yaw,
            s=target_s,
            is_corner=is_corner,
            is_final=target_s >= total - _SAME_S,
            is_crossing_stop=is_crossing_stop,
            is_crossing_exit=is_crossing_exit,
        )

    def _should_send(self, candidate: Goal) -> bool:
        """Decide whether candidate replaces the current goal.

        Hold goals (corner, crossing stop, far kerb) and final goals are always
        sent, and so is any goal that moves off a hold: otherwise, with a
        lookahead shorter than goal_update_distance, a released hold could stay
        Nav2's goal and the robot would park there.
        """
        if self._goal is None:
            return True
        change = abs(candidate.s - self._goal.s)
        if change < _SAME_S:
            return False  # same goal: never resend
        if (candidate.is_corner or candidate.is_crossing_stop or candidate.is_crossing_exit
                or candidate.is_final or self._goal.is_corner
                or self._goal.is_crossing_stop or self._goal.is_crossing_exit):
            return True   # Nav2 must aim exactly at holds and the end, and leave holds promptly
        return change >= self._config.goal_update_distance
