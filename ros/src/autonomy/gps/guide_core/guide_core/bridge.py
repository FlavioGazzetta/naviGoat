"""GPS bridge logic: the team's topics in, our NavigateTo mission out.

Pure Python (no ROS). The gps_bridge node feeds in what arrives on the
team's topics (/middleman/mode JSON, /gps/go) and what our mission manager
reports about the goal the bridge sent (state updates, the answer to a go,
the result), and carries out the commands that come back.

What the team's middleman already says is never repeated here: "GPS to X."
when a request starts, "Arrived." after /middleman/done, and its own reply
to "stop". A failure is not spoken here either: its sentence goes on
/middleman/failed and the middleman speaks it. The bridge speaks what lies
in between (the planned route, crossings, pauses, GPS warnings), each line
once, when it changes.

Every goal sent gets a new goal_id. Anything that arrives for an older goal
(a late result after a cancel, feedback still in flight) is ignored.

While something needed for a trip is missing (guide_core.health: no position,
Nav2 or our planner not running...), the node sets a blocker sentence and a
GPS request is refused at once with it, without sending a goal.

With auto_go (for a robot with no go button) the bridge gives the go itself:
the trip starts as soon as the route is planned and carries on at every road
crossing, and "Press go ..." is left out of what is spoken. A pause still waits
for a real go, and stop still stops.
"""

from dataclasses import dataclass
from enum import Enum
import json
import re

from guide_core.mission import MissionError, MissionState

GPS_MODE = 'gps'   # the middleman's mode name for "GPS take me to ..."

# /gps/status words, in the style of the team's /navigation/status ('going:x').
STATUS_WORDS = {
    MissionState.PLANNING: 'planning',
    MissionState.READY: 'ready',
    MissionState.NAVIGATING: 'going',
    MissionState.PAUSED: 'paused',
    MissionState.GPS_DEGRADED: 'gps_weak',
    MissionState.POSITION_LOST: 'position_lost',
    MissionState.REROUTING: 'rerouting',
}

# States whose message is not spoken by the bridge (see the module docstring).
SILENT_STATES = (MissionState.IDLE, MissionState.ARRIVED, MissionState.FAILED)

# Spoken when a result carries no message of its own.
FAILURE_TEXTS = {
    MissionError.NO_POSITION: "I don't know where we are yet, so I can't plan a route.",
    MissionError.UNKNOWN_DESTINATION: "I couldn't find that place.",
    MissionError.AMBIGUOUS_DESTINATION: 'There are several places with that name.',
    MissionError.PLANNING_FAILED: "I couldn't plan a route.",
    MissionError.NAVIGATION_FAILED: 'Navigation failed. The robot has stopped.',
    MissionError.CANCELED: 'The GPS trip was cancelled.',
    MissionError.BUSY: 'GPS navigation is busy with another trip.',
    MissionError.INVALID_GOAL: 'That destination is not valid.',
}
# With auto_go the bridge gives the go in these states, and does not say "Press go ...".
AUTO_GO_STATES = (MissionState.READY, MissionState.WAITING_AT_CROSSING)
PRESS_GO_SENTENCE = re.compile(r'\s*Press go\b[^.!?]*[.!?]')
DEFAULT_FAILURE_TEXT = 'GPS navigation failed.'
UNAVAILABLE_TEXT = "GPS navigation isn't running."
REJECTED_TEXT = 'GPS navigation refused the request.'


# --- What the middleman asked for ---

class Request(Enum):
    """What a /middleman/mode message means for the GPS layer."""

    START = 'start'     # a GPS trip to target
    CANCEL = 'cancel'   # stop, or any other mode: end our trip if one is running


@dataclass(frozen=True)
class ModeRequest:
    """A parsed /middleman/mode message."""

    request: Request
    target: str = ''


def clean_target(value):
    """Return the target as a string with single spaces ('' for None)."""
    if value is None:
        return ''
    return ' '.join(str(value).split())


def parse_mode(text, gps_mode=GPS_MODE):
    """Turn a /middleman/mode message into a ModeRequest.

    The message is JSON {"mode", "target", "intent", "stop"}. Our mode with
    a target starts a trip; a stop, another mode, or our mode without a
    target cancels. Return None for text that is not a JSON object.
    """
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    target = clean_target(data.get('target'))
    if data.get('mode') == gps_mode and not data.get('stop') and target:
        return ModeRequest(Request.START, target)
    return ModeRequest(Request.CANCEL)


def status_line(state, target):
    """Return the /gps/status line for a mission state ('' when there is none).

    IDLE, ARRIVED and FAILED give '': the result reports how a trip ended.
    A state number this code does not know gives 'state_<n>:<target>'.
    """
    if state in SILENT_STATES:
        return ''
    if state == MissionState.WAITING_AT_CROSSING:
        return 'waiting:crossing'
    word = STATUS_WORDS.get(state, f'state_{int(state)}')
    return f'{word}:{target}'


def failure_text(error_code, message=''):
    """Return the sentence for /middleman/failed: the result's message, or a fixed one."""
    message = (message or '').strip()
    if message:
        return message
    return FAILURE_TEXTS.get(error_code, DEFAULT_FAILURE_TEXT)


def failure_slug(error_code):
    """Return the short reason for 'failed:<reason>', e.g. 'unknown_destination'."""
    try:
        return MissionError(error_code).name.lower()
    except ValueError:
        return f'error_{int(error_code)}'


# --- Outputs (commands for the node) ---

@dataclass(frozen=True)
class StartMission:
    """Send a NavigateTo goal (TARGET_NAME) for target; tag what comes back with goal_id.

    skip_confirmation: set off as soon as the route is planned, without a go.
    """

    goal_id: int
    target: str
    skip_confirmation: bool = False


@dataclass(frozen=True)
class CancelMission:
    """Cancel the NavigateTo goal tagged goal_id."""

    goal_id: int


@dataclass(frozen=True)
class SendGo:
    """Call guide/user_go for goal goal_id; answer with on_go_reply."""

    goal_id: int


@dataclass(frozen=True)
class Say:
    """Publish text on /pete/response."""

    text: str


@dataclass(frozen=True)
class Status:
    """Publish text on /gps/status."""

    text: str


@dataclass(frozen=True)
class Done:
    """Publish target on /middleman/done (arrived)."""

    target: str


@dataclass(frozen=True)
class Failed:
    """Publish text on /middleman/failed (the trip was given up)."""

    text: str


@dataclass(frozen=True)
class CrossingWait:
    """Publish waiting on /gps/crossing_wait (latched; the node starts it at False)."""

    waiting: bool


# --- The bridge ---

class Bridge:
    """The gps_bridge node's logic. One instance for the node's lifetime.

    Every event method returns a tuple of commands, in the order they should
    be carried out.
    """

    def __init__(self, gps_mode=GPS_MODE, auto_go=False):
        self._gps_mode = gps_mode
        self._auto_go = bool(auto_go)
        self._auto_go_state = None  # the waiting state our own go was last sent for
        self._goal_id = 0
        self._active = False
        self._target = ''
        self._last_said = ''
        self._last_status = ''
        self._crossing_wait = False
        self._blocker = ''

    @property
    def active(self):
        """Return True while a GPS trip is running (sent and not yet ended)."""
        return self._active

    @property
    def goal_id(self):
        """Return the id of the latest goal sent (0 = none yet)."""
        return self._goal_id

    @property
    def target(self):
        """Return the target of the latest trip, as the middleman named it."""
        return self._target

    @property
    def blocker(self):
        """Return why a new trip would be refused now ('' = it would start)."""
        return self._blocker

    def set_blocker(self, text):
        """Set why a new trip cannot start ('' or None = nothing in the way).

        A trip already running is not affected: it handles its own problems.
        """
        self._blocker = (text or '').strip()

    # --- the team's inputs ---

    def on_mode(self, text):
        """Handle a /middleman/mode message: start, replace or cancel our trip."""
        request = parse_mode(text, self._gps_mode)
        if request is None:
            return ()
        commands = ()
        if self._active:
            commands = (CancelMission(self._goal_id),) + self._end(
                Status(f'cancelled:{self._target}'))
        if request.request is Request.START and self._blocker:
            return commands + (Status('failed:not_ready'), Failed(self._blocker))
        if request.request is Request.START:
            self._goal_id += 1
            self._active = True
            self._target = request.target
            self._last_said = ''
            self._last_status = ''
            self._auto_go_state = None
            commands += (StartMission(self._goal_id, request.target, self._auto_go),)
        return commands

    def on_go(self):
        """Handle the user's go (/gps/go); ignored when no GPS trip is running."""
        if not self._active:
            return ()
        return (SendGo(self._goal_id),)

    # --- what our mission manager reports ---

    def on_go_reply(self, goal_id, accepted, text):
        """Speak a refused go every time; an accepted one is told by the next message.

        With auto_go a refused go is not spoken: it was ours, and it is sent again
        with the next state update while the mission still waits.
        """
        text = (text or '').strip()
        if not self._is_current(goal_id) or accepted or not text:
            return ()
        if self._auto_go:
            self._auto_go_state = None
            return ()
        return (Say(text),)

    def on_state(self, goal_id, state, message):
        """Handle the mission's state and message (feedback or guide/mission_state)."""
        if not self._is_current(goal_id):
            return ()
        commands = self._set_crossing_wait(state == MissionState.WAITING_AT_CROSSING)
        line = status_line(state, self._target)
        if line and line != self._last_status:
            self._last_status = line
            commands += (Status(line),)
        if state not in SILENT_STATES:
            commands += self._say_once(message)
        if self._auto_go:
            if state not in AUTO_GO_STATES:
                self._auto_go_state = None
            elif state != self._auto_go_state:   # once each time the mission starts waiting
                self._auto_go_state = state
                commands += (SendGo(self._goal_id),)
        return commands

    def on_result(self, goal_id, error_code, message):
        """Handle the NavigateTo result: arrived, cancelled or failed."""
        if not self._is_current(goal_id):
            return ()
        if error_code == MissionError.NONE:
            return self._end(Status(f'arrived:{self._target}'), Done(self._target))
        if error_code == MissionError.CANCELED:
            # Not cancelled by us (we stop listening before our own cancel ends):
            # tell the middleman, or it would stay in GPS mode.
            return self._end(Status(f'cancelled:{self._target}'),
                             Failed(failure_text(error_code, message)))
        return self._end(Status(f'failed:{failure_slug(error_code)}'),
                         Failed(failure_text(error_code, message)))

    def on_goal_rejected(self, goal_id):
        """Handle the action server refusing the goal."""
        if not self._is_current(goal_id):
            return ()
        return self._end(Status('failed:rejected'), Failed(REJECTED_TEXT))

    def on_server_unavailable(self, goal_id):
        """Handle the action server not being there when the goal was to be sent."""
        if not self._is_current(goal_id):
            return ()
        return self._end(Status('failed:unavailable'), Failed(UNAVAILABLE_TEXT))

    # --- internals ---

    def _is_current(self, goal_id):
        return self._active and goal_id == self._goal_id

    def _end(self, *outputs):
        """End the current trip: crossing wait off, then the given outputs."""
        self._active = False
        return self._set_crossing_wait(False) + outputs

    def _set_crossing_wait(self, waiting):
        if waiting == self._crossing_wait:
            return ()
        self._crossing_wait = waiting
        return (CrossingWait(waiting),)

    def _say_once(self, text):
        text = (text or '').strip()
        if self._auto_go:
            text = PRESS_GO_SENTENCE.sub('', text).strip()
        if not text or text == self._last_said:
            return ()
        self._last_said = text
        return (Say(text),)
