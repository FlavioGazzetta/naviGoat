"""Controller: decides which mode a spoken request should start.

    "follow me"                -> FOLLOW
    "where is the cup"         -> PERCEPTION  (Pete: find / describe / guide to a visible object)
    "take me to the chair"     -> PERCEPTION  (guide)
    "take me to the bathroom"  -> NAVIGATION  (a known place inside the building)
    "take me to 12 Main St"    -> GPS         (an address or outdoor place)
    "stop"                     -> STOP        (from any mode, back to IDLE)

Priority: stop, help, a destination (even after "follow"), follow, find, describe, anything else.
A destination goes to the known places first (NAVIGATION), then campus words (NAVIGATION), then addresses (GPS),
then a visible object (PERCEPTION); anything left over is tried on GPS.

Text only, no ROS. Speech to text goes in, a Decision comes out:

    python3 controller.py        (type requests, decisions are printed)
"""

import re
import sys
from dataclasses import dataclass
from enum import Enum


class Mode(Enum):
    IDLE = 'idle'
    FOLLOW = 'follow'
    PERCEPTION = 'perception'
    NAVIGATION = 'navigation'
    GPS = 'gps'
    HELP = 'help'  # the user is in trouble: alert someone and stay put
    UNKNOWN = 'unknown'  # not understood: ask the user to rephrase


@dataclass
class Decision:
    mode: Mode
    intent: str = ''  # for PERCEPTION this is Pete's intent: find, guide, describe, question, ask
    target: str = ''
    text: str = ''
    stop: bool = False  # True for a stop request, mode is IDLE


# Keep in sync with VOCABULARY in ros/src/autonomy/perception/classification.py.
OBJECTS = {'person', 'chair', 'couch', 'bed', 'table', 'door', 'tv', 'remote', 'laptop', 'cell phone', 'bottle', 'cup', 'book', 'backpack', 'keys', 'wallet', 'glasses', 'shoe', 'refrigerator', 'sink', 'toilet', 'stairs', 'dog', 'cat'}
OBJECT_ALIASES = {'television': 'tv', 'remote control': 'remote', 'tv remote': 'remote', 'sofa': 'couch', 'phone': 'cell phone', 'fridge': 'refrigerator', 'desk': 'table', 'key': 'keys', 'steps': 'stairs'}

PLACE_ALIASES = {'bathroom': 'bathroom', 'restroom': 'bathroom', 'washroom': 'bathroom', 'kitchen': 'kitchen', 'exit': 'exit', 'entrance': 'entrance', 'lobby': 'lobby'}
OBJECT_FALLBACK_PLACES = {'toilet': 'bathroom', 'door': 'exit', 'refrigerator': 'kitchen', 'sink': 'kitchen'}

WAKE_WORDS = ('hey pete', 'ok pete', 'pete')

_STOP = re.compile(r"^((please|ok|okay|just) )*(stop|halt|cancel|wait|freeze|pause|never ?mind|hold on|enough)\b|\b(stop|quit) (following|moving|driving)\b|\b(don't|do not) (follow|move|go)\b|\bthat's enough\b")
_HELP = re.compile(r"^((please|somebody|someone) )?(help( me)?|i need help|emergency|call (for )?help)$|\b(i'm|i am) (lost|hurt)\b|\bi (fell|got lost)\b")
_FOLLOW = re.compile(r"\bfollow(ing)?\b|\b(come|walk|stay) with me\b")
_GUIDE = re.compile(r"\b(bring|take|lead|guide|walk|go|get|navigate|head)( me)? (to|towards)\b(?P<target>.*)")
# The robot leads the user ("lead me to the kitchen", handled by _GUIDE). If "follow" and a destination
# are both said, the destination wins rather than starting person following.
_FOLLOW_TO = re.compile(r"\b(follow(ing)?|(come|walk|stay) with me)\b.*?\b(to|towards)\b(?P<target>.*)")
_GO_HOME = re.compile(r"\b(bring|take|lead|guide|walk|go|get)( me)? (?P<target>home)\b")
_OUTSIDE = re.compile(r"\b(take|bring|get|lead|guide|walk)( me)? (outside|out)\b|\bget me out\b|\bi (want|need) to (leave|go outside)\b")
_NEED_PLACE = re.compile(r"\b(i need|i want|i have to|i'd like) (to )?(go to |get to |use |find )?(the )?(?P<target>bathroom|restroom|washroom|toilet|kitchen|exit|pee)\b")
_NEED_PLACE_ALIASES = {'toilet': 'bathroom', 'pee': 'bathroom'}  # "use the toilet" asks for the bathroom
# Asking to be taken to one of these means the room, not the fixture, when that room is a known place.
GUIDE_TO_PLACE = {'toilet': 'bathroom'}
_TWO_STEPS = re.compile(r"\b(and then|then)\b")
_FIND = re.compile(r"\b(where's|where is|where are|find|locate|look for|is there|do you see|i lost|i've lost|i misplaced)\b(?P<target>.*)")
_DESCRIBE = re.compile(r"\b(what's|what is|whats|describe)\b.*\b(in front|ahead|around|happening|room)\b|\bwhat (do|can) you see\b")
_NEED = re.compile(r"\b(i'd like|i would like|i need|i want|i'm looking for|somewhere to|i'm thirsty|i am thirsty)\b")
_QUESTION = re.compile(r"^(what|where|who|which|is|are|do|does|can|how)\b")
_FILLER = re.compile(r"^(the|a|an|my|any|nearest|closest) | (please|in front of me|around me|here|in the room|nearby|near me|close by)$")

_ADDRESS = re.compile(r"\b\d+\s+\w+(\s\w+)?\s(street|st|road|rd|avenue|ave|boulevard|blvd|lane|ln|drive|dr|way|court|ct|place|close|gardens|mews|row|terrace|crescent|square)\b|\b(street|avenue|boulevard|address|road|court|lane|square|crescent|terrace)\b")
_OUTDOOR = re.compile(r"\b(station|airport|hospital|mall|park|pharmacy|supermarket|bus stop|stop)\b")
_ROOM = re.compile(r"^(room|office|lab|floor) \w+$")
# Google Maps is poor inside the university, so these words mean NAVIGATION even when an address follows.
_CAMPUS = re.compile(r"\b(building|laboratory|lab|library|department(?! store)|lecture theatre|lecture hall|canteen|cafeteria|student union|school of|business school|faculty|level \d+|floor|room \d+|(?<!post )office|reception|front desk|main entrance|queen's tower)\b")

# Places the user sets once, e.g. {'home': (Mode.GPS, '12 main street')}. Asking for one that is not set needs an address.
SAVED_NAMES = ('home', 'work')


def normalise(text):
    text = re.sub(r"[^a-z0-9' ]+", ' ', text.lower().replace('’', "'"))
    return ' '.join(text.split())


def strip_wake_word(text, require):
    """Return the text after the wake word, or None if a wake word is required and missing."""
    for word in WAKE_WORDS:  # longest first
        match = re.search(rf'\b{word}\b', text)
        if match:
            return text[match.end():].strip()
    return None if require else text


def clean_target(target):
    target = target.strip()
    while (stripped := _FILLER.sub('', target).strip()) != target:
        target = stripped
    return target


def as_place(target, places):
    """The place name for target, or None if it is not a known place."""
    name = PLACE_ALIASES.get(target, target)
    return name if name in places else None


def as_object(target):
    target = OBJECT_ALIASES.get(target, target)
    if target.endswith('s') and target[:-1] in OBJECTS:
        target = target[:-1]
    return target if target in OBJECTS else None


def route_target(intent, target, text, places, saved):
    """Pick the mode for a request that names a destination or an object."""
    if target in SAVED_NAMES:
        if target not in saved:
            return Decision(Mode.UNKNOWN, 'need_address', target, text)  # ask the user where it is
        mode, destination = saved[target]
        return Decision(mode, 'goto', destination, text)
    if intent == 'guide' and GUIDE_TO_PLACE.get(target) in places:
        target = GUIDE_TO_PLACE[target]
    if place := as_place(target, places):
        return Decision(Mode.NAVIGATION, 'goto' if intent == 'guide' else 'locate', place, text)
    if _CAMPUS.search(target):
        return Decision(Mode.NAVIGATION, 'goto' if intent == 'guide' else 'locate', target, text)
    if _ADDRESS.search(target) or _OUTDOOR.search(target):
        return Decision(Mode.GPS, 'goto', target, text)
    if intent == 'guide' and _ROOM.match(target):
        return Decision(Mode.NAVIGATION, 'goto', target, text)  # a room that is not in the known places yet
    if intent == 'guide' and not as_object(target):
        return Decision(Mode.GPS, 'goto', target, text)  # not a known place or an object: let the geocoder decide if it is a real place
    # Pete can search for any name, not just the vocabulary, so a find request goes to him.
    return Decision(Mode.PERCEPTION, intent, as_object(target) or target, text)


def find_target(text, patterns):
    """The first (intent, target) any pattern finds in text, or None."""
    for intent, pattern in patterns:
        match = pattern.search(text)
        if match:
            target = clean_target(match.group('target'))
            if target and target not in ('it', 'that', 'this'):
                return intent, target
    return None


def classify(text, places=frozenset(PLACE_ALIASES.values()), require_wake_word=False, saved=None):
    """Turn one spoken request into a Decision. None if there is nothing to act on."""
    saved = saved or {}
    text = strip_wake_word(normalise(text), require_wake_word)
    if not text:
        return None
    if _STOP.search(text):
        return Decision(Mode.IDLE, text=text, stop=True)
    if _HELP.search(text):
        return Decision(Mode.HELP, text=text)
    if _TWO_STEPS.search(text):
        return Decision(Mode.UNKNOWN, 'one_at_a_time', text=text)  # ask for one request at a time
    if _OUTSIDE.search(text):
        return route_target('guide', 'exit', text, places, saved)
    if match := _NEED_PLACE.search(text):
        target = match.group('target')
        return route_target('guide', _NEED_PLACE_ALIASES.get(target, target), text, places, saved)
    if found := find_target(text, (('guide', _GUIDE), ('guide', _GO_HOME), ('guide', _FOLLOW_TO))):
        return route_target(*found, text, places, saved)  # a destination beats "follow"
    if _FOLLOW.search(text):
        return Decision(Mode.FOLLOW, text=text)
    if found := find_target(text, (('find', _FIND),)):
        return route_target(*found, text, places, saved)
    if _DESCRIBE.search(text):
        return Decision(Mode.PERCEPTION, 'describe', text=text)
    if _NEED.search(text):
        return Decision(Mode.PERCEPTION, 'ask', text=text)  # Pete works out what is wanted
    if _QUESTION.search(text):
        return Decision(Mode.PERCEPTION, 'question', text=text)
    return Decision(Mode.UNKNOWN, text=text)


class Controller:
    """Holds the current mode. Feed it requests with handle(), and report
    problems from the active mode with fail() and finish()."""

    def __init__(self, places=frozenset(PLACE_ALIASES.values()), require_wake_word=False, saved=None):
        self.places = set(places)
        self.saved = dict(saved or {})
        self.require_wake_word = require_wake_word
        self.mode = Mode.IDLE
        self.current = None

    def handle(self, text):
        """Return the Decision to act on, or None if the request was ignored."""
        decision = classify(text, self.places, self.require_wake_word, self.saved)
        if decision is None:
            return None
        if decision.mode is Mode.UNKNOWN:
            return decision  # stay in the current mode: the user just needs to rephrase
        self.mode, self.current = decision.mode, decision
        return decision

    def finish(self):
        """The active mode reached its goal."""
        self.mode, self.current = Mode.IDLE, None

    def fail(self):
        """The active mode gave up. Try a known place for the target if there is one, otherwise go idle.
        Returns the next Decision, or None."""
        current = self.current
        self.finish()
        if current and current.mode is Mode.PERCEPTION and current.intent == 'guide':
            place = OBJECT_FALLBACK_PLACES.get(current.target)
            if place in self.places:
                self.mode = Mode.NAVIGATION
                self.current = Decision(Mode.NAVIGATION, 'goto', place, current.text)
                return self.current
        return None


def format_decision(decision):
    shown = 'STOP' if decision.stop else decision.mode.value.upper()
    return f'{shown}  intent={decision.intent or "-"}  target={decision.target or "-"}'


def main():
    controller = Controller()
    print('Type a request (Ctrl+Z then Enter on Windows, Ctrl+D elsewhere, to quit).')
    for line in sys.stdin:
        decision = controller.handle(line)
        if decision is not None:
            print(f'  {format_decision(decision)}')


if __name__ == '__main__':
    main()
