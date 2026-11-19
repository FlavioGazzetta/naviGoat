r"""Mission state machine: one journey from "take me to X" to "you have arrived".

Pure Python (no ROS). The mission manager node feeds events in (a goal,
the planner's answer, route progress, the paused flag, Nav2 statuses, the
user's go / pause, cancel) and carries out the commands that come back
(call the planner, send the route to the follower, pause, resume, confirm a
crossing, stop the robot, finish the action).

Who owns what:
- the route follower decides when the robot waits at a crossing;
- the goal sender owns pause / resume;
- this machine owns the mission and what the user's "go" means right now.
While a route is being followed, WAITING_AT_CROSSING and PAUSED are COPIED
from the follower (guide/route_progress) and the goal sender (guide/paused),
never decided here, so the nodes cannot disagree.

    IDLE -> PLANNING -> READY --go--> NAVIGATING <-> WAITING_AT_CROSSING
                                          ^  \--------> PAUSED (pause / Nav2 failure)
                                          +--------------/ (go)
    NAVIGATING -> ARRIVED;  any failure -> FAILED;  cancel -> IDLE

Destinations by name: the curated list first. Only if it has no match, the
geocoder is asked (LookUpOnline -> on_lookup_result); the state stays
PLANNING meanwhile ("Searching OpenStreetMap for ..."). An online result is
always read back in READY and waits for go, even with skip_confirmation.

Localisation and rerouting (Step 8). The localizer's quality (GOOD /
DEGRADED / LOST, on_localization) and the follower's off_route flag (in
Progress) are inputs too:
- DEGRADED while navigating is reported as GPS_DEGRADED: the robot keeps going
  (the follower already stops earlier); waiting / paused states take priority.
- LOST pauses the robot (route kept) and reports POSITION_LOST; never in the
  middle of a crossing (it pauses once off the road). When the position is
  back the robot stays paused: the user's go resumes, as after any pause.
- off_route while navigating with GOOD quality: the robot is stopped, the
  route cleared, a new route planned from where it is (REROUTING) and read
  back in READY; it starts on the user's go, like any route. At most
  max_reroutes reroutes without reroute_reset_distance of progress between.
Without any on_localization call the quality counts as GOOD, so the machine
behaves exactly as before Step 8.
"""

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
from typing import Optional, Tuple

from guide_core.destinations import Destination, format_distance, MatchConfig, Outcome, resolve
from guide_core.localization_quality import Quality
from guide_core.online_resolve import FOUND_IN_CACHE, FOUND_ONLINE, LARGE_AREA_NOTE, resolve_online


class MissionState(IntEnum):
    """States, numbered as in guide_interfaces/msg/MissionState.msg."""

    IDLE = 0
    PLANNING = 1
    READY = 2
    NAVIGATING = 3
    WAITING_AT_CROSSING = 4
    PAUSED = 5
    ARRIVED = 6
    FAILED = 7
    GPS_DEGRADED = 8     # navigating, but the position is uncertain (Step 8)
    POSITION_LOST = 9    # stopped: position too uncertain to guide (Step 8)
    REROUTING = 10       # stopped off the route, planning a new one (Step 8)


class MissionError(IntEnum):
    """Result codes, numbered as in NavigateTo.action's result."""

    NONE = 0
    NO_POSITION = 1
    UNKNOWN_DESTINATION = 2
    AMBIGUOUS_DESTINATION = 3
    PLANNING_FAILED = 4
    NAVIGATION_FAILED = 5
    CANCELED = 6
    BUSY = 7
    INVALID_GOAL = 8


class NavStatus(IntEnum):
    """Nav2 goal statuses, numbered as in guide_interfaces/msg/NavGoalStatus.msg."""

    PENDING = 0
    ACTIVE = 1
    SUCCEEDED = 2
    ABORTED = 3
    CANCELED = 4
    REJECTED = 5


# --- Inputs ---

@dataclass(frozen=True)
class MissionGoal:
    """What the user asked for: a destination name, or a lat/lon (name empty)."""

    destination: str = ''
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    skip_confirmation: bool = False

    @property
    def by_name(self):
        return self.latitude is None and self.longitude is None


@dataclass(frozen=True)
class CrossingInfo:
    """What the user is told about one crossing of the planned route."""

    kind: str               # 'signalled', 'unsignalled' or 'unknown'
    road: str               # 'major', 'minor', 'none' or 'unknown'
    has_island: bool = False


@dataclass(frozen=True)
class PlanSummary:
    """The parts of a successful PlanRoute answer this machine needs."""

    length: float
    goal_snap_distance: float
    crossings: Tuple[CrossingInfo, ...] = ()
    warnings: Tuple[str, ...] = ()


@dataclass(frozen=True)
class Progress:
    """The parts of guide/route_progress this machine needs."""

    route_id: int
    distance_remaining: float
    arrived: bool = False
    waiting_at_crossing: bool = False
    on_crossing: bool = False
    next_crossing_index: int = -1
    off_route: bool = False


@dataclass(frozen=True)
class MissionConfig:
    """Tuning of the mission behaviour.

    max_nav_failures: Nav2 failures in a row (robot not moving on) before the
        mission fails; each earlier failure pauses and waits for go (retry).
    failure_reset_distance: metres the robot must move along the route after
        a failure for the failure count to start again from zero.
    min_go_interval: seconds; a go within this time of the previous accepted
        go is ignored, so a double press cannot resume AND start a crossing.
    snap_report_distance: if the route ends further than this (m) from the
        requested point, the read-back says so.
    online_lookup: ask the geocoder (LookUpOnline) when the curated list has
        no match; False = fail with UNKNOWN_DESTINATION straight away. Off
        here; the mission manager node turns it on (its parameter
        online_lookup, default true).
    pick_nearest: when several places match a name ("pret"), go to the one
        nearest to the robot and say so, instead of failing with the list of
        options. False = fail with AMBIGUOUS_DESTINATION (the user chooses).
    max_reroutes: new routes allowed after leaving the route before the
        mission fails ("I keep losing the route"); the count starts again after
        reroute_reset_distance metres of progress.
    """

    max_nav_failures: int = 3
    failure_reset_distance: float = 5.0
    min_go_interval: float = 1.0
    snap_report_distance: float = 5.0
    online_lookup: bool = False
    pick_nearest: bool = False
    max_reroutes: int = 3
    reroute_reset_distance: float = 100.0

    def __post_init__(self):
        if self.max_nav_failures < 1:
            raise ValueError('max_nav_failures must be >= 1')
        if self.max_reroutes < 0:
            raise ValueError('max_reroutes must be >= 0')
        for name in ('failure_reset_distance', 'min_go_interval', 'snap_report_distance',
                     'reroute_reset_distance'):
            if not getattr(self, name) >= 0.0:
                raise ValueError(f'{name} must be >= 0')


# --- Outputs (commands for the node) ---

@dataclass(frozen=True)
class CallPlanner:
    """Call guide/plan_route; answer with on_plan_result(plan_id, ...)."""

    plan_id: int
    start_lat: float
    start_lon: float
    goal_lat: float
    goal_lon: float


@dataclass(frozen=True)
class LookUpOnline:
    """Call guide/geocode for query; answer with on_lookup_result(lookup_id, ...)."""

    lookup_id: int
    query: str


@dataclass(frozen=True)
class SendRoute:
    """Call guide/set_route with the last plan; answer with on_route_set(route_id, ...)."""

    route_id: int


@dataclass(frozen=True)
class ClearRoute:
    """Call guide/clear_route: the follower stops publishing goals."""


@dataclass(frozen=True)
class StopRobot:
    """Call the goal sender's guide/cancel: cancel the goal, forget it, unpause."""


@dataclass(frozen=True)
class Pause:
    """Call the goal sender's guide/pause."""


@dataclass(frozen=True)
class Resume:
    """Call the goal sender's guide/resume."""


@dataclass(frozen=True)
class ConfirmCrossing:
    """Call the follower's guide/confirm_crossing; answer with on_confirm_result."""


@dataclass(frozen=True)
class Finish:
    """End the NavigateTo goal with this result."""

    error_code: MissionError
    message: str
    distance_remaining: float = 0.0


@dataclass(frozen=True)
class Reply:
    """Answer to the user's go / pause, plus the commands it causes."""

    accepted: bool
    text: str
    commands: Tuple[object, ...] = ()


@dataclass(frozen=True)
class Snapshot:
    """Everything guide/mission_state publishes."""

    state: MissionState
    route_id: int
    destination_name: str
    destination_note: str
    route_length: float
    num_crossings: int
    summary: str
    warnings: Tuple[str, ...]
    message: str


ROAD_WORDS = {'major': 'major road', 'minor': 'minor road',
              'none': 'not a mapped road', 'unknown': 'road type unknown'}


def _plural(n, word):
    return f'{n} {word}' + ('' if n == 1 else 's')


def describe_crossing(info):
    """'signalled, major road, with an island'."""
    text = f'{info.kind}, {ROAD_WORDS.get(info.road, info.road)}'
    return text + (', with an island' if info.has_island else '')


def nearest_destination(candidates):
    """Return (the destination to go to, how many it was chosen from).

    The only candidate, or of several the nearest by straight-line distance
    (the first one if no distance is known).
    """
    best = min(candidates, key=lambda c: c.distance if c.distance is not None else math.inf)
    return best.destination, len(candidates)


def build_summary(name, note, plan, snap_report_distance, skip_confirmation=False,
                  found_in='', large_area=False):
    """Return the read-back sentence for READY (pure text; tested on its own).

    "South Kensington Station, end of the subway. About 650 m walk, 6 road
    crossings, 2 with an island. Warning: route uses 193 m of tunnel (no GPS
    there). Press go to start."
    found_in: for online results, where they came from ('OpenStreetMap'):
    "Found in OpenStreetMap: Royal Albert Hall. ..."
    large_area: the destination is a large building or area whose centre
    was searched for: the read-back says the route goes to its nearest path.
    """
    parts = [(f'Found in {found_in}: ' if found_in else '') + name
             + (f', {note}' if note else '') + '.']
    n = len(plan.crossings)
    islands = sum(1 for c in plan.crossings if c.has_island)
    walk = f'{format_distance(plan.length)} walk'
    walk = walk[0].upper() + walk[1:]
    crossings = _plural(n, 'road crossing') if n else 'no road crossings'
    if islands:
        crossings += f', {islands} with an island'
    parts.append(f'{walk}, {crossings}.')
    if large_area:
        parts.append(f'{name} is a large area: the route goes to its nearest path, '
                     'not to an entrance.')
    for warning in plan.warnings:
        parts.append(f'Warning: {warning}.')
    if plan.goal_snap_distance > snap_report_distance:
        parts.append(f'The route ends {plan.goal_snap_distance:.0f} m from {name}.')
    parts.append('Setting off.' if skip_confirmation else 'Press go to start.')
    return ' '.join(parts)


class _Phase(Enum):
    """Where the mission is; while ACTIVE the reported state is derived."""

    IDLE = 'idle'
    LOOKING_UP = 'looking_up'  # waiting for the geocoder (reported as PLANNING)
    PLANNING = 'planning'
    READY = 'ready'           # planned; waiting for go (or for the follower to accept)
    REROUTING = 'rerouting'   # left the route: robot stopped, planning a new route
    ACTIVE = 'active'         # the follower has the route
    ARRIVED = 'arrived'
    FAILED = 'failed'


class Mission:
    """The mission manager's logic. One instance for the node's lifetime.

    Every event method returns a tuple of commands (user inputs return a
    Reply with commands inside). Times are seconds from any steady clock.
    """

    def __init__(self, destinations=(), config=None, match_config=None):
        self._destinations = tuple(destinations)
        self._config = config or MissionConfig()
        self._match_config = match_config or MatchConfig()
        self._plan_id = 0
        self._lookup_id = 0
        self._route_id = 0
        self._phase = _Phase.IDLE
        self._paused = False          # copied from guide/paused (any time)
        self._quality = Quality.GOOD  # copied from guide/localization_status (any time)
        self._version = 0
        self._reset_mission()
        self._message = ''

    # --- read-only views ---

    @property
    def config(self):
        return self._config

    @property
    def route_id(self):
        """Route id handed to the follower for the current mission (0 = none)."""
        return self._active_route_id

    @property
    def version(self):
        """Increases on every change of the snapshot; the node republishes then."""
        return self._version

    @property
    def state(self):
        if self._phase is _Phase.ACTIVE:
            if self._lost_hold:
                return MissionState.POSITION_LOST
            if self._paused:
                return MissionState.PAUSED
            if self._waiting:
                return MissionState.WAITING_AT_CROSSING
            if self._quality is not Quality.GOOD:
                return MissionState.GPS_DEGRADED
            return MissionState.NAVIGATING
        return {
            _Phase.IDLE: MissionState.IDLE,
            _Phase.LOOKING_UP: MissionState.PLANNING,
            _Phase.PLANNING: MissionState.PLANNING,
            _Phase.READY: MissionState.READY,
            _Phase.REROUTING: MissionState.REROUTING,
            _Phase.ARRIVED: MissionState.ARRIVED,
            _Phase.FAILED: MissionState.FAILED,
        }[self._phase]

    def can_accept(self):
        """Return True if a new goal may start now (otherwise reject it: BUSY)."""
        return self._phase in (_Phase.IDLE, _Phase.ARRIVED, _Phase.FAILED)

    def snapshot(self):
        plan = self._plan
        warnings = plan.warnings if plan else ()
        if self._phase is _Phase.ACTIVE and self._quality is not Quality.GOOD:
            warnings += ('position lost' if self._quality is Quality.LOST
                         else 'GPS signal weak',)
        return Snapshot(
            state=self.state, route_id=self._active_route_id,
            destination_name=self._dest_name, destination_note=self._dest_note,
            route_length=plan.length if plan else 0.0,
            num_crossings=len(plan.crossings) if plan else 0,
            summary=self._summary, warnings=warnings,
            message=self._message)

    # --- goal lifecycle ---

    def start(self, goal, robot_latlon):
        """Start the mission for an accepted NavigateTo goal.

        robot_latlon: (lat, lon), or None if no recent position. Returns
        commands (CallPlanner, LookUpOnline when the curated list has no
        match, or Finish on failure).
        """
        if not self.can_accept():
            raise RuntimeError('start() while a mission is running; check can_accept()')
        self._reset_mission()

        if goal.by_name:
            if not goal.destination.strip():
                return self._fail(MissionError.INVALID_GOAL, 'No destination was given.')
        elif not (goal.latitude is not None and goal.longitude is not None
                  and math.isfinite(goal.latitude) and math.isfinite(goal.longitude)
                  and -90.0 <= goal.latitude <= 90.0
                  and -180.0 <= goal.longitude <= 180.0):
            return self._fail(MissionError.INVALID_GOAL,
                              'The destination coordinates are not valid.')
        if robot_latlon is None or self._quality is Quality.LOST:
            return self._fail(MissionError.NO_POSITION,
                              "I don't know where we are yet, so I can't plan a route.")

        if goal.by_name:
            resolution = resolve(goal.destination, self._destinations,
                                 robot_latlon=robot_latlon, config=self._match_config)
            if resolution.outcome is Outcome.NOT_FOUND:
                if not self._config.online_lookup or not resolution.query:
                    return self._fail(MissionError.UNKNOWN_DESTINATION, resolution.message)
                # Not in the curated list: ask the geocoder, plan when it answers.
                self._lookup_id += 1
                self._query = resolution.query
                self._robot = robot_latlon
                self._skip_requested = goal.skip_confirmation
                self._phase = _Phase.LOOKING_UP
                self._set_message(f"Searching OpenStreetMap for '{resolution.query}'.")
                return (LookUpOnline(self._lookup_id, resolution.query),)
            if resolution.outcome is Outcome.AMBIGUOUS and not self._config.pick_nearest:
                return self._fail(MissionError.AMBIGUOUS_DESTINATION, resolution.message)
            target, self._picked_from = nearest_destination(resolution.candidates)
        else:
            target = Destination('the requested point', goal.latitude, goal.longitude)
        return self._plan_to(target, robot_latlon, goal.skip_confirmation)

    def on_lookup_result(self, lookup_id, status, destinations=(), from_cache=False):
        """Take the geocoder's answer (or its failure: pass status OFFLINE).

        status: the Geocode response status; destinations: its candidates as
        Destinations (source 'nominatim'), in its order. One result -> plan
        to it (read back, always waits for go); several -> AMBIGUOUS; none or
        a failure -> UNKNOWN_DESTINATION with the reason.
        """
        if self._phase is not _Phase.LOOKING_UP or lookup_id != self._lookup_id:
            return ()  # answer to a cancelled or older lookup
        resolution = resolve_online(self._query, status, destinations,
                                    robot_latlon=self._robot, from_cache=from_cache,
                                    config=self._match_config)
        if resolution.outcome is Outcome.NOT_FOUND:
            return self._fail(MissionError.UNKNOWN_DESTINATION, resolution.message)
        if resolution.outcome is Outcome.AMBIGUOUS and not self._config.pick_nearest:
            return self._fail(MissionError.AMBIGUOUS_DESTINATION, resolution.message)
        target, self._picked_from = nearest_destination(resolution.candidates)
        return self._plan_to(target, self._robot, self._skip_requested,
                             FOUND_IN_CACHE if from_cache else FOUND_ONLINE)

    def on_plan_result(self, plan_id, success, message, plan=None):
        """Take the planner's answer (plan is a PlanSummary when success is True)."""
        if (self._phase not in (_Phase.PLANNING, _Phase.REROUTING)
                or plan_id != self._plan_id):
            return ()  # answer to a cancelled or older request
        if not success:
            return self._fail(MissionError.PLANNING_FAILED,
                              f"I couldn't plan a route: {message}")
        self._plan = plan
        if self._phase is _Phase.REROUTING:
            # A new route mid-trip: always read back, always waits for go.
            summary = build_summary(self._dest_name, self._dest_note, plan,
                                    self._config.snap_report_distance)
            self._summary = 'New route: ' + summary.replace('Press go to start.',
                                                            'Press go to continue.')
            self._phase = _Phase.READY
            self._set_message(self._summary)
            return ()
        self._summary = build_summary(self._dest_name, self._dest_note, plan,
                                      self._config.snap_report_distance,
                                      self._skip_confirmation, self._found_in,
                                      self._large_area)
        self._phase = _Phase.READY
        self._set_message(self._summary)
        if self._skip_confirmation:
            return self._send_route()
        return ()

    def on_route_set(self, route_id, success, message, sent_stamp=-math.inf):
        """Take the follower's answer to guide/set_route.

        sent_stamp: time (s) the request was sent; goal statuses stamped at or
        before it belong to an earlier route (the topic is latched) and are ignored.
        """
        if self._phase is not _Phase.READY or route_id != self._pending_route_id:
            return ()
        self._pending_route_id = 0
        self._last_status_stamp = max(self._last_status_stamp, sent_stamp)
        if not success:
            return self._fail(MissionError.PLANNING_FAILED,
                              f'The route could not be loaded: {message}')
        self._active_route_id = route_id
        self._phase = _Phase.ACTIVE
        self._last_remaining = None   # progress is measured afresh on each route
        self._failures = 0
        self._last_state = MissionState.NAVIGATING
        self._set_message('Setting off.')
        return ()

    def cancel(self):
        """Cancel the NavigateTo goal: stop everything, back to IDLE."""
        if self._phase in (_Phase.ARRIVED, _Phase.FAILED, _Phase.IDLE):
            return ()
        commands = []
        if self._phase is _Phase.ACTIVE or self._pending_route_id:
            # The follower has (or may be about to get) the route: stop the robot.
            commands += [StopRobot(), ClearRoute()]
        commands.append(Finish(MissionError.CANCELED, 'Cancelled. The robot has stopped.',
                               self._distance_remaining))
        self._phase = _Phase.IDLE
        self._active_route_id = 0
        self._pending_route_id = 0
        self._set_message('Cancelled. The robot has stopped.')
        return tuple(commands)

    # --- what the other nodes report ---

    def on_paused(self, paused):
        """guide/paused changed (latched, from the goal sender)."""
        self._paused = bool(paused)
        if self._paused:
            self._pause_pending = False
        else:
            self._resume_pending = False
        return self._after_change()

    def on_progress(self, progress, robot_latlon=None):
        """Take a guide/route_progress message (ignored unless it is for our route).

        robot_latlon: the robot's (lat, lon) now, needed only to plan a new
        route when progress.off_route says the robot has left this one.
        """
        if self._phase is not _Phase.ACTIVE or progress.route_id != self._active_route_id:
            return ()
        if self._last_remaining is not None and progress.distance_remaining < self._last_remaining:
            self._travelled += self._last_remaining - progress.distance_remaining
        self._last_remaining = progress.distance_remaining
        if self._reroutes and self._travelled - self._reroute_travel \
                >= self._config.reroute_reset_distance:
            self._reroutes = 0
        self._distance_remaining = progress.distance_remaining
        self._waiting = progress.waiting_at_crossing
        self._on_crossing = progress.on_crossing
        self._next_crossing = progress.next_crossing_index
        moved = self._failure_at - progress.distance_remaining  # metres since last failure
        if self._failures and moved >= self._config.failure_reset_distance:
            self._failures = 0
        if progress.arrived:
            name = self._dest_name
            self._phase = _Phase.ARRIVED
            self._active_route_id = 0
            message = f'You have arrived at {name}.'
            self._set_message(message)
            return (ClearRoute(), Finish(MissionError.NONE, message, 0.0))
        commands = self._check_localization()
        if progress.off_route and self._can_reroute():
            return commands + self._reroute(robot_latlon)
        return commands + self._after_change()

    def on_localization(self, quality):
        """Take the localizer's quality (GOOD / DEGRADED / LOST) from its status topic."""
        self._quality = Quality(quality)
        if self._phase is not _Phase.ACTIVE:
            return ()
        return self._check_localization() + self._after_change()

    def on_nav_status(self, status, stamp):
        """Take a guide/goal_status message.

        stamp: its header time (seconds), used to skip repeats; only ABORTED
        and REJECTED matter (CANCELED = our own pause).
        """
        if self._phase is not _Phase.ACTIVE or stamp <= self._last_status_stamp:
            return ()
        self._last_status_stamp = stamp
        if status not in (NavStatus.ABORTED, NavStatus.REJECTED):
            return ()
        self._failures += 1
        self._failure_at = self._distance_remaining
        if self._failures >= self._config.max_nav_failures:
            return self._fail(
                MissionError.NAVIGATION_FAILED,
                f'Navigation failed {self._failures} times in a row. '
                'The robot has stopped.')
        self._failure_pause = True
        self._set_message(f'The robot could not continue ({self._failures} of '
                          f'{self._config.max_nav_failures}). Press go to try again.')
        if self._paused or self._pause_pending:
            return ()
        self._pause_pending = True
        return (Pause(),)

    def on_confirm_result(self, success, text):
        """Take the follower's answer to guide/confirm_crossing."""
        if not success:
            self._set_message(f'Could not start the crossing: {text}')
        return ()

    def on_service_failed(self, command, text):
        """Handle a failed Pause / Resume / ConfirmCrossing / StopRobot / ClearRoute call.

        The service was missing or answered with an error. Clears the
        'pending' flag so the user can try again, and tells the user.
        (Planner and set_route failures go through on_plan_result /
        on_route_set with success=False instead.)
        """
        if isinstance(command, Pause):
            self._pause_pending = False
            self._pause_text = ''
        elif isinstance(command, Resume):
            self._resume_pending = False
        self._set_message(f'Something went wrong: {text}')
        return ()

    # --- the user's inputs ---

    def on_go(self, now):
        """Handle the user's go: start / cross / resume, depending on the state."""
        state = self.state
        if state not in (MissionState.READY, MissionState.WAITING_AT_CROSSING,
                         MissionState.PAUSED):
            texts = {MissionState.NAVIGATING: 'Already moving.',
                     MissionState.GPS_DEGRADED: 'Already moving.',
                     MissionState.PLANNING: 'Still planning, please wait.',
                     MissionState.REROUTING: 'Finding a new route, please wait.',
                     MissionState.POSITION_LOST: "Waiting for GPS: I can't continue yet."}
            return Reply(False, texts.get(state, 'Nothing to start.'))
        if state is MissionState.READY and self._quality is Quality.LOST:
            return Reply(False, "I'm not sure where we are yet. Please wait.")
        if now - self._last_go < self._config.min_go_interval:
            return Reply(False, 'Go pressed twice; the second press was ignored.')
        if state is MissionState.READY:
            if self._pending_route_id:
                return Reply(False, 'Starting, please wait.')
            self._last_go = now
            return Reply(True, 'Starting.', self._send_route())
        if state is MissionState.PAUSED:
            if self._resume_pending:
                return Reply(False, 'Resuming, please wait.')
            self._last_go = now
            self._resume_pending = True
            self._failure_pause = False
            return Reply(True, 'Resuming.', (Resume(),))
        self._last_go = now  # waiting at a crossing
        return Reply(True, 'Crossing.', (ConfirmCrossing(),))

    def on_pause(self):
        """Handle the user's pause. Always obeyed while moving, even on a crossing."""
        if self._phase is not _Phase.ACTIVE:
            return Reply(False, 'Not moving.')
        if self._paused or self._pause_pending:
            return Reply(False, 'Already paused.')
        self._pause_pending = True
        self._failure_pause = False
        if self._on_crossing:
            text = 'Paused in the road. Press go to continue crossing.'
        else:
            text = 'Paused.'
        # Not shown yet: the message changes once, when guide/paused confirms
        # the pause (see _after_change), so the user hears it once, with the
        # state PAUSED. The Reply tells the caller straight away.
        self._pause_text = text
        return Reply(True, text, (Pause(),))

    # --- internals ---

    def _reset_mission(self):
        self._dest_name = ''
        self._dest_note = ''
        self._picked_from = 1         # how many matching places the destination was chosen from
        self._found_in = ''           # '' = curated / lat-lon; else where the result came from
        self._large_area = False
        self._query = ''              # name being looked up online
        self._robot = None            # robot (lat, lon) when the lookup started
        self._skip_requested = False
        self._plan = None
        self._summary = ''
        self._skip_confirmation = False
        self._pending_route_id = 0
        self._active_route_id = 0
        self._distance_remaining = 0.0
        self._waiting = False
        self._on_crossing = False
        self._next_crossing = -1
        self._failures = 0
        self._failure_at = 0.0
        self._failure_pause = False
        self._pause_pending = False
        self._pause_text = ''         # message to show once the pause is confirmed
        self._resume_pending = False
        self._last_go = -math.inf
        self._last_status_stamp = -math.inf
        self._last_state = None
        self._target = None           # the Destination being travelled to
        self._lost_hold = False       # paused because the position is lost
        self._reroutes = 0            # new routes since the count last restarted
        self._travelled = 0.0         # metres of progress on this mission's routes
        self._reroute_travel = 0.0    # _travelled at the last reroute
        self._last_remaining = None   # distance_remaining of the previous progress

    def _plan_to(self, target, robot_latlon, skip_confirmation, found_in=''):
        """Start planning to target. Online results never skip the read-back."""
        online = target.source != 'curated'
        self._target = target
        self._dest_name = target.name
        self._dest_note = '' if online else target.note
        if self._picked_from > 1:   # read back, so the user knows a choice was made
            chosen = f'the nearest of {self._picked_from} places with that name'
            self._dest_note = f'{self._dest_note}, {chosen}' if self._dest_note else chosen
        self._found_in = found_in
        self._large_area = online and target.note == LARGE_AREA_NOTE
        self._skip_confirmation = skip_confirmation and not online
        self._plan_id += 1
        self._phase = _Phase.PLANNING
        self._set_message(f'Planning a route to {target.name}.')
        return (CallPlanner(self._plan_id, robot_latlon[0], robot_latlon[1],
                            target.lat, target.lon),)

    def _check_localization(self):
        """Pause when the position is lost (never on a crossing); note when it is back."""
        if self._quality is Quality.LOST:
            if self._lost_hold or self._on_crossing:
                return ()   # already holding, or finish the crossing first
            self._lost_hold = True
            self._failure_pause = False
            if self._paused or self._pause_pending:
                return ()
            self._pause_pending = True
            self._pause_text = ''
            return (Pause(),)
        self._lost_hold = False   # position back: stay paused until the user's go
        return ()

    def _can_reroute(self):
        """Return True if leaving the route should trigger a new route now."""
        return (self.state is MissionState.NAVIGATING and not self._on_crossing
                and not self._pause_pending and self._quality is Quality.GOOD)

    def _reroute(self, robot_latlon):
        """Stop, clear the route and plan a new one from where the robot is."""
        if robot_latlon is None:
            return self._fail(MissionError.NAVIGATION_FAILED,
                              "We've left the route and I don't know where we are. "
                              'The robot has stopped.')
        if self._reroutes >= self._config.max_reroutes:
            return self._fail(MissionError.NAVIGATION_FAILED,
                              'I keep losing the route. The robot has stopped.')
        self._reroutes += 1
        self._reroute_travel = self._travelled
        self._plan_id += 1
        self._phase = _Phase.REROUTING
        self._active_route_id = 0
        self._waiting = False
        self._on_crossing = False
        self._set_message("We've left the route. Stopping to find a new one.")
        target = self._target
        return (StopRobot(), ClearRoute(),
                CallPlanner(self._plan_id, robot_latlon[0], robot_latlon[1],
                            target.lat, target.lon))

    def _send_route(self):
        self._route_id += 1
        self._pending_route_id = self._route_id
        return (SendRoute(self._route_id),)

    def _fail(self, code, message):
        commands = []
        if self._phase is _Phase.ACTIVE:
            commands += [StopRobot(), ClearRoute()]
        remaining = self._distance_remaining if self._phase is _Phase.ACTIVE else 0.0
        self._phase = _Phase.FAILED
        self._active_route_id = 0
        self._pending_route_id = 0
        self._set_message(message)
        commands.append(Finish(code, message, remaining))
        return tuple(commands)

    def _set_message(self, message):
        self._message = message
        self._version += 1

    def _after_change(self):
        """Update the message when the derived state changes."""
        if self._phase is not _Phase.ACTIVE:
            return ()
        state = self.state
        if state is self._last_state:
            return ()
        old, self._last_state = self._last_state, state
        if state is MissionState.POSITION_LOST:
            self._set_message("I'm not sure where we are. Stopped, waiting for GPS.")
        elif state is MissionState.WAITING_AT_CROSSING:
            n = len(self._plan.crossings) if self._plan else 0
            k = self._next_crossing
            what = ''
            if self._plan and 0 <= k < n:
                what = f' ({describe_crossing(self._plan.crossings[k])})'
            self._set_message(f'At crossing {k + 1} of {n}{what}. '
                              'Press go when it is safe to cross.')
        elif state is MissionState.PAUSED:
            if old is MissionState.POSITION_LOST:   # position back, still stopped
                self._set_message('Position found again. Press go to continue.')
            elif self._pause_text:          # the user's pause, now confirmed
                self._set_message(self._pause_text)
                self._pause_text = ''
            elif self._failure_pause:       # message already set by on_nav_status
                self._version += 1
            else:                           # paused from elsewhere (e.g. the CLI)
                self._set_message('Paused. Press go to continue.')
        elif old is MissionState.WAITING_AT_CROSSING:
            self._set_message('Crossing now.')
        elif old is MissionState.PAUSED:
            self._set_message('Continuing.')
        elif state is MissionState.GPS_DEGRADED:
            self._set_message('GPS signal is weak. Continuing carefully.')
        elif old is MissionState.GPS_DEGRADED:
            self._set_message('GPS signal is back.')
        else:
            self._version += 1
        return ()
