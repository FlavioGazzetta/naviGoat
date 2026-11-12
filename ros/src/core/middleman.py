"""Middleman: turns what the user said into one mode and one target, and holds the current mode.

Modes (the middleman only handles going somewhere, following, and stopping):

    "follow Alice" / "follow him"    -> FOLLOW    Pete follows another person: a name, or '' for the one in the frame
    "take me to the chair"           -> OBJECT    (guide the user to a stored object)
    "take me to the bathroom"        -> LOCATION  (a stored location on the map)
    "GPS take me to 12 Main Street"  -> GPS       (only when the user says "GPS")
    "manual mode" / "let me drive"   -> MANUAL    the user drives the robot with the stick
    "stop" / "exit manual mode" / "stop following" -> IDLE
    "help"                           -> HELP

Going to a location, object or GPS destination ends by itself: the code that drives it calls finish() on arrival.
Following and manual mode have no goal, so they only end when the user says stop or exits the mode.

"Where is X" / "find X" is not handled for now (a spoken reply says so). Everything else is about what the camera
sees ("what's in front of me", "is the light on?") and is not the middleman's job: the text goes straight to perception over ROS (see middleman_node.py) and the
mode is unchanged. An unknown place or object gets a spoken reply ("Unknown location: X.").

Every Decision has a reply: what Pete says out loud, which is also the confirmation that the request was heard.
When there is no reply because the text goes to perception, which answers later, on_ack() is called so the user hears
a beep instead of silence.

The language model (see qwen_interpreter) only picks a mode and a target. The target is then checked against the
stored lists in catalog.py. If the model is slow or not reachable, simple rules decide, so a stop never waits.

Text only, no ROS. Speech to text goes in, a Decision comes out:

    python3 middleman.py        (type requests, decisions are printed)
    python3 middleman.py --qwen (the same, with Qwen reading the request; needs Ollama)
"""

import argparse
import difflib
import json
import re
import sys
import threading
import urllib.error
import urllib.request
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from catalog import GUIDE_TO_LOCATION, LOCATION_ALIASES, LOCATIONS, OBJECT_ALIASES, OBJECT_FALLBACK_LOCATIONS, OBJECTS


class Mode(Enum):
    IDLE = 'idle'
    FOLLOW = 'follow'
    OBJECT = 'object'
    LOCATION = 'location'
    GPS = 'gps'
    MANUAL = 'manual'  # the user drives the robot with the stick
    HELP = 'help'  # the user is in trouble: alert someone and stay put


@dataclass
class Decision:
    mode: Optional[Mode]  # None: no mode change (a reply to speak, or text for perception)
    intent: str = ''  # 'goto' for OBJECT, LOCATION and GPS; '' otherwise
    target: str = ''  # FOLLOW: the name of the person for Pete to follow, or '' for the person in the frame
    text: str = ''
    stop: bool = False  # True for a stop request, mode is IDLE
    reply: str = ''  # what Pete says out loud
    forward: str = ''  # the full text for perception (what the camera sees); mode is None


WAKE_WORDS = ('hey pete', 'ok pete', 'pete')

_STOP = re.compile(r"^((please|ok|okay|just) )*(stop|halt|cancel|wait|freeze|pause|never ?mind|hold on|enough)\b|\b(stop|quit) (following|moving|driving)\b|\b(don't|do not) (follow|move|go)\b|\bthat's enough\b")
_HELP = re.compile(r"^((please|somebody|someone) )?(help( me)?|i need help|emergency|call (for )?help)$|\b(i'm|i am) (lost|hurt)\b|\bi (fell|got lost)\b")
# Leaving an open-ended mode: "exit manual mode", "stop following", "leave follow mode". Checked before _MANUAL.
_EXIT = re.compile(r"\b(exit|leave|quit|end|disable|turn off|get out of)\b.*\b(manual|follow(ing)?|mode|control)\b")
_MANUAL = re.compile(r"\bmanual\b|\b(joystick|joy stick|stick) (mode|control)\b|\blet me (drive|steer|control)\b|\bi('ll| will) (drive|steer)\b")
_GPS = re.compile(r"\bgps\b")
_GPS_TARGET = re.compile(r"\bgps\b.*?\b(to|towards)\b(?P<target>.*)")
# Pete follows another person; the user follows Pete. Users are not expected to say "follow me".
_FOLLOW = re.compile(r"\bfollow(ing)?\b")
_FOLLOW_NAME = re.compile(r"\bfollow(ing)? (?!(me|him|her|them|us|that|this|the|a|my|it)\b)(?P<target>[a-z]+)")
_GUIDE = re.compile(r"\b(bring|take|lead|guide|walk|go|get|navigate|head)( me)? (to|towards)\b(?P<target>.*)")
_FOLLOW_TO = re.compile(r"\b(follow(ing)?|(come|walk|stay) with me)\b.*?\b(to|towards)\b(?P<target>.*)")
_GO_HOME = re.compile(r"\b(bring|take|lead|guide|walk|go|get)( me)? (?P<target>home)\b")
_OUTSIDE = re.compile(r"\b(take|bring|get|lead|guide|walk)( me)? (outside|out)\b|\bget me out\b|\bi (want|need) to (leave|go outside)\b")
_NEED_PLACE = re.compile(r"\b(i need|i want|i have to|i'd like) (to )?(go to |get to |use |find )?(the )?(?P<target>bathroom|restroom|washroom|toilet|kitchen|exit|pee)\b")
_NEED_PLACE_ALIASES = {'toilet': 'bathroom', 'pee': 'bathroom'}
_TWO_STEPS = re.compile(r"\b(and then|then)\b")
# "Where is X" / "find X": not handled for now, the middleman only takes the user to places.
_FIND = re.compile(r"\b(where's|where is|where are|find|locate|look for|is there|do you see|i lost|i've lost|i misplaced|i'm looking for)\b|^where\b")
# Anything about what is around or in view goes to perception.
_SCENE = re.compile(r"\b(what's|what is|whats|describe|what (do|can) you see|i'd like|i would like|i need|i want|somewhere to|i'm thirsty|i am thirsty)\b|^(what|who|which|is|are|do|does|can|how)\b")
_FILLER = re.compile(r"^(the|a|an|my|any|nearest|closest) | (please|in front of me|around me|here|in the room|nearby|near me|close by)$")

# Places the user sets once, e.g. {'home': (Mode.GPS, '12 main street')}.
SAVED_NAMES = ('home', 'work')

# The same local Ollama and model that Pete uses (ros/src/autonomy/perception/classification.py).
OLLAMA_URL = 'http://localhost:11434/api/chat'
QWEN_MODEL = 'qwen3-vl:2b-instruct'

QWEN_MODES = ('follow', 'object', 'location', 'gps', 'manual', 'perception', 'stop', 'help', 'unknown')
QWEN_SCHEMA = {'type': 'object', 'properties': {'mode': {'enum': list(QWEN_MODES)}, 'target': {'type': 'string'}, 'match': {'type': 'string'}, 'confidence': {'type': 'number'}}, 'required': ['mode', 'target', 'match', 'confidence']}
# The language model links what was said to the closest stored place or object. Below this confidence the
# answer is "not a known location/object": it need not be the exact word, but it has to be something similar.
CONFIDENCE_THRESHOLD = 0.7
# How close a mis-heard word must be to a stored name to count as it ("bathrom" -> bathroom). difflib ratio.
SPELLING_CUTOFF = 0.85
QWEN_PROMPT = """You turn what a blind person says to Pete, their guide robot, into one command.

mode is one of:
- follow: Pete follows ANOTHER person ("follow Alice", "follow that person"). target is the name, or "" for the person in front.- location: take them to a stored place. Places: {locations}
- object: take them to a stored object. Objects: {objects}
- gps: ONLY if they say "GPS" ("GPS take me to 12 Main Street"). target is the address or place.
- manual: they want to drive the robot themselves with the stick ("manual mode", "let me drive")
- perception: describing the scene or a question about what can be seen ("what's around me", "is the light on")
- stop: stop, cancel, never mind
- help: they are hurt, lost or in danger
- unknown: anything else

target is the place or object they want to go to, as they said it in a few lowercase words, or "" if there is none.
match is the ONE entry from the lists above that means the same thing or is the closest category ("sofa" -> couch, "loo" -> bathroom, "mug" -> cup, "canteen" -> kitchen). Use "" if nothing in the lists fits.
confidence is 0 to 1: how sure you are that match is what they want. Use a low number if you are guessing."""


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


def close_to(target, names):
    """The name that target is a near-spelling of (a mis-heard word), or None."""
    found = difflib.get_close_matches(target, names, n=1, cutoff=SPELLING_CUTOFF)
    return found[0] if found else None


def as_location(target, locations):
    name = LOCATION_ALIASES.get(target, target)
    return name if name in locations else close_to(name, locations)


def as_object(target):
    target = OBJECT_ALIASES.get(target, target)
    if target.endswith('s') and target[:-1] in OBJECTS:
        target = target[:-1]
    return target if target in OBJECTS else close_to(target, OBJECTS)


def say_unknown(target, hint=None):
    if hint == 'location':
        return f'Unknown location: {target}.'
    if hint == 'object':
        return f'Unknown object: {target}.'
    return f'Unknown: {target}.'


def resolve_known(target, text, locations, saved):
    """LOCATION or OBJECT if target is a stored name, an alias or a near-spelling of one, otherwise None."""
    if target in SAVED_NAMES and target in saved:
        mode, destination = saved[target]
        return Decision(mode, 'goto', destination, text, reply=f'Going {target}.')
    obj = as_object(target)
    place = as_location(GUIDE_TO_LOCATION.get(obj, target), locations)
    if place:
        return Decision(Mode.LOCATION, 'goto', place, text, reply=f'Going to the {place}.')
    if obj:
        return Decision(Mode.OBJECT, 'goto', obj, text, reply=f'Going to the {obj}.')
    return None


def resolve(target, text, locations, saved, hint=None, match='', confidence=0.0):
    """Pick LOCATION or OBJECT for something the user wants to go to, or reply that it is unknown.
    match is the stored name the language model thinks target means; it is used if it is confident enough."""
    if not target:
        return Decision(None, text=text, reply='Where to?')
    if decision := resolve_known(target, text, locations, saved):
        return decision
    match = clean_target(normalise(match or ''))
    if match and confidence >= CONFIDENCE_THRESHOLD and (decision := resolve_known(match, text, locations, saved)):
        return decision
    return Decision(None, text=text, reply=say_unknown(target, hint))


def gps_decision(target, text):
    if not target:
        return Decision(None, text=text, reply='GPS to where?')
    return Decision(Mode.GPS, 'goto', target, text, reply=f'GPS to {target}.')


def follow_decision(target, text):
    """Pete follows another person: the one named, or the one in the frame."""
    return Decision(Mode.FOLLOW, target=target, text=text, reply=f'Following {target}.' if target else 'Following.')


def manual_decision(text):
    return Decision(Mode.MANUAL, text=text, reply='Manual mode.')


def to_perception(text):
    return Decision(None, text=text, forward=text)


def decision_from_mode(mode, target, text, locations, saved, match='', confidence=0.0):
    """The Decision for what the language model understood, or None if it could not tell."""
    target = clean_target(normalise(target or ''))
    if mode == 'stop':
        return Decision(Mode.IDLE, text=text, stop=True, reply='Stopping.')
    if mode == 'help':
        return Decision(Mode.HELP, text=text, reply='Calling for help.')
    if mode == 'follow':
        return follow_decision(target, text)
    if mode == 'manual':
        return manual_decision(text)
    if mode == 'perception':
        return to_perception(text)
    if mode == 'gps':
        if not _GPS.search(text):  # the rule: GPS only when the user says GPS
            return Decision(None, text=text, reply='Say "GPS" first.')
        return gps_decision(target, text)
    if mode in ('object', 'location'):
        return resolve(target, text, locations, saved, hint=mode, match=match, confidence=confidence)
    return None


def qwen_interpreter(locations=LOCATIONS, saved=None, url=OLLAMA_URL, model=QWEN_MODEL, timeout=10):
    """A function that asks the local Qwen what a request means: Decision, or None if it is slow, unreachable or unsure."""
    saved = saved or {}
    prompt = QWEN_PROMPT.format(locations=', '.join(locations), objects=', '.join(OBJECTS))

    def interpret(text):
        messages = [{'role': 'system', 'content': prompt}, {'role': 'user', 'content': text}]
        body = {'model': model, 'stream': False, 'keep_alive': -1, 'format': QWEN_SCHEMA, 'options': {'num_predict': 60, 'temperature': 0}, 'messages': messages}
        try:
            request = urllib.request.Request(url, json.dumps(body).encode(), {'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=timeout) as response:
                result = json.loads(json.load(response)['message']['content'])
        except (urllib.error.URLError, TimeoutError, KeyError, ValueError):
            return None
        try:
            confidence = float(result.get('confidence') or 0)
        except (TypeError, ValueError):
            confidence = 0.0
        return decision_from_mode(result.get('mode'), result.get('target'), text, locations, saved, result.get('match'), confidence)

    interpret.warm_up = lambda: warm_up_qwen(url, model)
    return interpret


def warm_up_qwen(url=OLLAMA_URL, model=QWEN_MODEL):
    """Load the model now, so the first request is not slow. False if Ollama is not reachable."""
    try:
        request = urllib.request.Request(url, json.dumps({'model': model, 'messages': [], 'keep_alive': -1}).encode(), {'Content-Type': 'application/json'})
        urllib.request.urlopen(request, timeout=300).close()
        return True
    except (urllib.error.URLError, TimeoutError, ValueError):
        return False


def find_target(text, patterns):
    """The destination the first matching pattern finds in text, or None."""
    for pattern in patterns:
        match = pattern.search(text)
        if match:
            target = clean_target(match.group('target'))
            if target and target not in ('it', 'that', 'this', 'there'):
                return target
    return None


def classify(text, locations=LOCATIONS, require_wake_word=False, saved=None, interpreter=None):
    """Turn one spoken request into a Decision. None if there is nothing to act on.
    interpreter is an optional function (see qwen_interpreter) that reads what the rules leave to it."""
    saved = saved or {}
    text = strip_wake_word(normalise(text), require_wake_word)
    if not text:
        return None
    if _STOP.search(text) or _EXIT.search(text):
        return Decision(Mode.IDLE, text=text, stop=True, reply='Stopping.')
    if _HELP.search(text):
        return Decision(Mode.HELP, text=text, reply='Calling for help.')
    if _TWO_STEPS.search(text):
        return Decision(None, text=text, reply='One at a time.')
    if _MANUAL.search(text):
        return manual_decision(text)
    if _FIND.search(text) and not (_GUIDE.search(text) or _NEED_PLACE.search(text) or _GPS.search(text)):
        return Decision(None, text=text, reply="Can't look for things.")
    if interpreter and (decision := interpreter(text)):
        return decision
    if _GPS.search(text):
        match = _GPS_TARGET.search(text)
        return gps_decision(clean_target(match.group('target')) if match else '', text)
    if _OUTSIDE.search(text):
        return resolve('exit', text, locations, saved)
    if match := _NEED_PLACE.search(text):
        target = match.group('target')
        return resolve(_NEED_PLACE_ALIASES.get(target, target), text, locations, saved)
    if target := find_target(text, (_GUIDE, _GO_HOME, _FOLLOW_TO)):
        return resolve(target, text, locations, saved)  # a destination beats "follow"
    if _FOLLOW.search(text):
        name = _FOLLOW_NAME.search(text)
        return follow_decision(name.group('target') if name else '', text)
    if _SCENE.search(text):
        return to_perception(text)
    return Decision(None, text=text, reply="Say that again.")


# What Pete says when the user ends the mode they are in.
LEAVING_REPLIES = {Mode.MANUAL: 'Manual mode off.', Mode.FOLLOW: 'Stopped following.'}


class Middleman:
    """Holds the current mode. Feed it requests with handle(), and report
    problems from the active mode with fail() and finish().

    handle() may be called from several threads, so a "stop" is handled while a slow request is still with the
    language model. Once a stop or help has been heard, any earlier request still being read is dropped when it
    finishes, so it cannot start a mode after the stop."""

    def __init__(self, locations=LOCATIONS, require_wake_word=False, saved=None, interpreter=None, on_ack=None):
        self.locations = list(locations)
        self.saved = dict(saved or {})
        self.require_wake_word = require_wake_word
        self.interpreter = interpreter
        self.on_ack = on_ack
        self.mode = Mode.IDLE
        self.current = None
        self._lock = threading.RLock()
        self._epoch = 0  # raised by every stop or help, to cancel the requests heard before it

    def _interrupts(self, text):
        text = strip_wake_word(normalise(text), self.require_wake_word)
        return bool(text) and bool(_STOP.search(text) or _EXIT.search(text) or _HELP.search(text))

    def handle(self, text):
        """Return the Decision to act on, or None if the request was ignored."""
        with self._lock:
            if self._interrupts(text):
                self._epoch += 1
            epoch = self._epoch
        decision = classify(text, self.locations, self.require_wake_word, self.saved, self.interpreter)  # may be slow
        if decision is None:
            return None
        with self._lock:
            if epoch != self._epoch and not (decision.stop or decision.mode is Mode.HELP):
                return None  # a stop or help came in while this was being read
            if decision.forward and self.on_ack:
                self.on_ack()  # no spoken reply: perception answers later, so beep to say it was heard
            if decision.mode is None:
                return decision  # stay in the current mode: speak the reply, or send the text to perception
            if decision.stop:
                decision.reply = LEAVING_REPLIES.get(self.mode, decision.reply)
            self.mode, self.current = decision.mode, decision
            return decision

    def finish(self):
        """The active mode reached its goal. LOCATION, OBJECT and GPS have a goal, so whoever drives them calls this
        on arrival. FOLLOW and MANUAL have no goal, so they only end when the user says stop or exits the mode."""
        with self._lock:
            self.mode, self.current = Mode.IDLE, None

    def fail(self):
        """The active mode gave up. Try a stored location for the target if there is one, otherwise go idle.
        Returns the next Decision, or None."""
        with self._lock:
            current = self.current
            self.finish()
            if current and current.mode is Mode.OBJECT:
                place = OBJECT_FALLBACK_LOCATIONS.get(current.target)
                if place in self.locations:
                    self.mode = Mode.LOCATION
                    self.current = Decision(Mode.LOCATION, 'goto', place, current.text, reply=f'No {current.target}. Going to the {place}.')
                    return self.current
            return None


def format_decision(decision):
    shown = 'STOP' if decision.stop else decision.mode.value.upper() if decision.mode else ('PERCEPTION' if decision.forward else 'NO CHANGE')
    detail = f'  forward="{decision.forward}"' if decision.forward else ''
    return f'{shown}  target={decision.target or "-"}{detail}  reply="{decision.reply}"'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--qwen', action='store_true', help='let Qwen (through Ollama) read each request after the stop and help rules')
    args = parser.parse_args()
    interpreter = qwen_interpreter() if args.qwen else None
    if interpreter and not interpreter.warm_up():
        print('Cannot reach Ollama, using the rules only.')
        interpreter = None
    middleman = Middleman(interpreter=interpreter, on_ack=lambda: print('  (beep)'))
    print('Type a request (Ctrl+Z then Enter on Windows, Ctrl+D elsewhere, to quit).')
    for line in sys.stdin:
        decision = middleman.handle(line)
        if decision is not None:
            print(f'  {format_decision(decision)}')


if __name__ == '__main__':
    main()
