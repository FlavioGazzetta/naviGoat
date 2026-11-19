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
Following and manual mode have no arrival goal. Both end on user stop; following
also ends on correlated target-loss or input-failure feedback.

"Remember this place as X" saves where the robot is now: the node publishes X on /navigation/save_place (navigation's
places_node stores the position). places_node then sends all saved names back on /navigation/places, and
set_saved_places() adds them to the places "take me to X" understands, next to the ones in catalog.py, and tells the
language model about them too (a saved place has no purpose line like the catalog's, so the model goes by its name).

"Where is X" / "find X" takes the user to X, the same as "take me to X". Everything else is about what the camera
sees ("what's in front of me", "is the light on?") and is not the middleman's job: the text goes straight to perception over ROS (see middleman_node.py) and the
mode is unchanged. A place or object that is not in catalog.py gets a spoken reply ("I don't know where the piano is.");
perception is only asked about objects in the list, so "I couldn't find the chair." means it looked and did not see one.

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
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from catalog import (GUIDE_TO_LOCATION, LOCATION_ALIASES, LOCATION_DESCRIPTIONS, LOCATIONS, OBJECT_ALIASES,
                     OBJECT_FALLBACK_LOCATIONS, OBJECTS)


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
    gps_here: str = ''  # "GPS, I'm at the library": the place the user says they are at, for the GPS layer; mode is None
    save_place: str = ''  # "remember this place as the study": the name for navigation to save the robot's position under; mode is None
    request_id: str = field(default='', compare=False)  # correlate commands and following feedback


WAKE_WORDS = ('hey pete', 'ok pete', 'pete')

# "remember this place as the study", "save this spot as kitchen": navigation saves where the robot is now under that name.
# "store" is not a verb here: "take me to the store near here" is a request to go somewhere.
_SAVE_PLACE = re.compile(r"\b(remember|save|mark)\b (this|here|(my|the) (current )?(place|location|spot|position))\b")
_SAVE_NAME = re.compile(r"\b(as|called|named)\b(?P<name>.*)")
_STOP = re.compile(r"^((please|ok|okay|just) )*(stop|halt|cancel|wait|freeze|pause|never ?mind|hold on|enough)\b|\b(stop|quit) (following|moving|driving)\b|\b(don't|do not) (follow|move|go)\b|\bthat's enough\b")
_HELP = re.compile(r"^((please|somebody|someone) )?(help( me)?|i need help|emergency|call (for )?help)$|\b(i'm|i am) (lost|hurt)\b|\bi (fell|got lost)\b")
# Leaving an open-ended mode: "exit manual mode", "stop following", "leave follow mode". Checked before _MANUAL.
_EXIT = re.compile(r"\b(exit|leave|quit|end|disable|turn off|get out of)\b.*\b(manual|follow(ing)?|mode|control)\b")
_MANUAL = re.compile(r"\bmanual\b|\b(joystick|joy stick|stick) (mode|control)\b|\blet me (drive|steer|control)\b|\bi('ll| will) (drive|steer)\b")
_GPS = re.compile(r"\bgps\b")
_GPS_TARGET = re.compile(r"\bgps\b.*?\b(to|towards)\b(?P<target>.*)")
# "GPS, I'm at the library": where the user is now, for the GPS layer to start its routes from when it has no fix
_GPS_HERE = re.compile(r"\bgps\b.*?\b(?:(?:i'm|im|i am|we're|we are) (?:now |currently |standing |right now )*(?:at|in|inside|outside|near|next to|by)"
                       r"|(?:my|our) (?:location|position) is(?: at| in)?|start(?:ing)? from)\b(?P<target>.*)")
# Pete follows another person; the user follows Pete. Users are not expected to say "follow me".
_FOLLOW = re.compile(r"\bfollow(ing)?\b")
_FOLLOW_NAME = re.compile(r"\bfollow(ing)? (?!(me|him|her|them|us|that|this|the|a|my|it|follow|following|please|now|again|too)\b)(?P<target>[a-z]+)")
_GUIDE = re.compile(r"\b(bring|take|lead|guide|walk|go|get|navigate|head)( me)? (to|towards)\b(?P<target>.*)")
_FOLLOW_TO = re.compile(r"\b(follow(ing)?|(come|walk|stay) with me)\b.*?\b(to|towards)\b(?P<target>.*)")
_GO_HOME = re.compile(r"\b(bring|take|lead|guide|walk|go|get)( me)? (?P<target>home)\b")
_OUTSIDE = re.compile(r"\b(take|bring|get|lead|guide|walk)( me)? (outside|out)\b|\bget me out\b|\bi (want|need) to (leave|go outside)\b")
_NEED_PLACE = re.compile(r"\b(i need|i want|i have to|i'd like) (to )?(go to |get to |use |find )?(the )?(?P<target>bathroom|restroom|washroom|toilet|kitchen|exit|pee)\b")
_NEED_PLACE_ALIASES = {'toilet': 'bathroom', 'pee': 'bathroom'}
_TWO_STEPS = re.compile(r"\b(and then|then)\b")
# "Where is X" / "find X" is answered by taking the user to X, the same as "take me to X".
_FIND = re.compile(r"\b(where's|where is|where are|find|locate|look for|is there|do you see|i lost|i've lost|i misplaced|i'm looking for)\b|^where\b")
_FIND_TARGET = re.compile(r"\b(where's|where is|where are|find|locate|look for|is there|do you see|i lost|i've lost|i misplaced|i'm looking for)\b(?P<target>.*)")
# The questions perception can answer: it only describes the whole view. These are answered at once with DESCRIBING_REPLY.
DESCRIBE = re.compile(r"\b(what's|what is|whats|describe)\b.*\b(in front|ahead|around|nearby|near me|happening|room|scene|scenery|surroundings)\b|\bwhat (do|can) you see\b"
                      r"|\bdescribe\b|\bwhere am i\b|\blook around\b")
DESCRIBING_REPLY = 'Describing the scene.'
# Anything about what is around or in view goes to perception.
_SCENE =re.compile(r"\b(what's|what is|whats|describe|what (do|can) you see|i'd like|i would like|i need|i want|somewhere to|i'm thirsty|i am thirsty)\b|^(what|who|which|is|are|do|does|can|how)\b")
_FILLER = re.compile(r"^(the|a|an|my|any|nearest|closest) | (please|in front of me|around me|here|in the room|nearby|near me|close by)$")
# Questions about where the user is or what is around: the scene description answers them.
_LOOK = re.compile(r"\bwhere am i\b|\blook around\b|\bdescribe\b|\bwhat (do|can) you see\b|\bwhat'?s around\b")
# Going somewhere without saying where ("take me", "go there", "let's go"): ask, never guess.
_NO_DESTINATION = re.compile(r"^(please )?((let's|lets|let us) go|(take|bring|lead|guide|walk|get) me|go|come on|move|go on)( (there|then|now|please|ahead|on))*$")
# Words that carry no need or place. A request made only of these is not given to the language model, which would
# otherwise pick a place anyway ("go" -> exit, "okay" -> bathroom, with the same confidence as a real need).
_VAGUE_WORDS = {'pete', 'go', 'going', 'take', 'bring', 'lead', 'guide', 'walk', 'get', 'me', 'us', 'there', 'here', 'then', 'now',
                'please', 'come', 'on', 'okay', 'ok', 'yes', 'yeah', 'no', 'let', "let's", 'lets', 'next', 'leave', 'look', 'around',
                'the', 'a', 'to', 'and', 'so', 'um', 'uh', 'hmm', 'hi', 'hello', 'hey', 'thanks', 'thank', 'you', 'move', 'again', 'ahead'}

# Places the user sets once, e.g. {'home': (Mode.GPS, '12 main street')}.
SAVED_NAMES = ('home', 'work')

# The model the scene description also uses (core/llm.py): Claude with a key and a connection, otherwise the local one
import llm  # noqa: E402

OLLAMA_URL = llm.OLLAMA_URL
OLLAMA_OPTIONS = llm.OLLAMA_OPTIONS  # shared with the scene description, so Ollama never reloads the model
QWEN_MODEL = llm.LOCAL_MODEL  # the name is from when this was Qwen

QWEN_MODES = ('follow', 'object', 'location', 'gps', 'manual', 'perception', 'stop', 'help', 'unknown')
QWEN_SCHEMA = {'type': 'object', 'properties': {'mode': {'enum': list(QWEN_MODES)}, 'target': {'type': 'string'}, 'match': {'type': 'string'}, 'confidence': {'type': 'number'}}, 'required': ['mode', 'target', 'match', 'confidence']}
# The language model links what was said to the closest stored place or object. Below this confidence the
# answer is "not a known location/object": it need not be the exact word, but it has to be something similar.
CONFIDENCE_THRESHOLD = 0.7
# Claude says "" for anything that is not in the lists, and is modest about a need that several places meet: measured
# on the Jetson, "I'm hungry" -> pret at 0.5 to 0.7, "I need to study" at 0.6, "I'm thirsty" at 0.5. At 0.7 those were
# refused ("Say that again."). The local model keeps 0.7: it is sure of itself even when it is wrong.
CLOUD_CONFIDENCE_THRESHOLD = 0.5
# How close a mis-heard word must be to a stored name to count as it ("bathrom" -> bathroom). difflib ratio.
SPELLING_CUTOFF = 0.85
QWEN_PROMPT = """You help Pete, a guide robot at Imperial College London, work out where a person who cannot see wants to go.
They say what they need. Choose the ONE place or object below that meets the need, or "" if none does.
Places: {locations}
Objects: {objects}
Examples: "I'm hungry" -> pret. "I have to study" -> abdus salam library. "I want to work out" -> ethos sports centre. "I need to pray" -> prayer room. "I need to print something" -> computer room.
Answer with a name exactly as written in the lists, without the brackets. If what they ask for is not in the lists, answer "". A particular named place or person that is not listed (for example someone's office) is not a match: answer "".
confidence is 0 to 1: how sure you are that the place or object you chose meets the need. When several listed places would meet it equally well (several places sell food), choose one and stay sure: that is not guessing. Use a low number only if you doubt that anything in the lists is what they want."""


def normalise(text):
    text = re.sub(r"[^a-z0-9' ]+", ' ', text.lower().replace('’', "'"))
    return ' '.join(text.split())


_GREETING = re.compile(r"^((hello|hi|hey|ok|okay|yo)\b[ ,]*)+")


def strip_wake_word(text, require):
    """Return the request next to the wake word (after it, or before it: "take me to the bathroom, Pete"), or None
    if a wake word is required and missing."""
    for word in WAKE_WORDS:  # longest first
        match = re.search(rf'\b{word}\b', text)
        if match:
            return text[match.end():].strip() or _GREETING.sub('', text[:match.start()]).strip()
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
    if target in locations:
        return target  # a name that is itself a place wins over an alias, so a saved "lab" is not the laboratory
    name = LOCATION_ALIASES.get(target, target)
    return name if name in locations else close_to(name, locations)


def as_object(target):
    target = OBJECT_ALIASES.get(target, target)
    if target not in OBJECTS:
        for ending in ('es', 's'):  # 'buses' -> 'bus', 'chairs' -> 'chair', 'sofas' -> 'couch'
            if target.endswith(ending):
                singular = OBJECT_ALIASES.get(target[:-len(ending)], target[:-len(ending)])
                if singular in OBJECTS:
                    return singular
    return target if target in OBJECTS else close_to(target, OBJECTS)


def say_unknown(target):
    article = '' if target in SAVED_NAMES or re.search(r'\d', target) else 'the '  # "where home is", "where 12 main street is"
    return f"I don't know where {article}{target} is."


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
    return Decision(None, text=text, reply=say_unknown(target))


# "the natural history museum in south kensington": the map search wants the name alone, and only covers that area anyway
_AREA = re.compile(r"\s+(in|at|near|of)\s+(the\s+)?(south kensington|kensington|london|imperial college london|imperial college|imperial)$")


def strip_area(place):
    """A place name without the area said after it: 'abdus salam library in imperial college' -> 'abdus salam library'."""
    while (shorter := _AREA.sub('', place)) != place and shorter:
        place = shorter
    return place


def gps_decision(target, text):
    target = strip_area(target)
    if not target:
        return Decision(None, text=text, reply='GPS to where?')
    return Decision(Mode.GPS, 'goto', target, text, reply=f'GPS to {target}.')


def follow_decision(target, text):
    """Pete follows another person: the one named, or the one in the frame."""
    return Decision(Mode.FOLLOW, target=target, text=text, reply=f'Following {target}.' if target else 'Following.')


def manual_decision(text):
    return Decision(Mode.MANUAL, text=text, reply='Manual mode.')


def to_perception(text):
    """Send the question to perception. A scene description takes a few seconds, so Pete says he is doing it straight away."""
    return Decision(None, text=text, forward=text, reply=DESCRIBING_REPLY if DESCRIBE.search(text) else '')


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
    """A function that asks the local Qwen which stored place or object meets a need ("I'm hungry" -> pret): a Decision,
    or None if it is slow, unreachable or not sure. It can only answer with a name from the lists, so it cannot start
    follow, manual or any other mode: those stay with the rules."""
    saved = saved or {}
    current = {}  # what the model is told about: set_places() replaces all of it at once

    def set_places(places):
        """Tell the model about these places (the catalog's, and the ones saved by voice) instead of the ones it has now."""
        places = list(places)
        described = ', '.join(f'{p} ({LOCATION_DESCRIPTIONS[p]})' if p in LOCATION_DESCRIPTIONS else p for p in places)
        names = list(dict.fromkeys(places + list(OBJECTS))) + ['']
        schema = {'type': 'object', 'properties': {'match': {'type': 'string', 'enum': names}, 'confidence': {'type': 'number'}},
                  'required': ['match', 'confidence'], 'additionalProperties': False}
        current['told'] = (places, QWEN_PROMPT.format(locations=described, objects=', '.join(OBJECTS)), schema)  # one assignment: never half old, half new

    set_places(locations)

    def interpret(text, minimum=CONFIDENCE_THRESHOLD):
        locations, prompt, schema = current['told']
        messages = [{'role': 'system', 'content': prompt}, {'role': 'user', 'content': text}]
        body = {'model': model, 'stream': False, 'keep_alive': -1, 'format': schema, 'options': {**OLLAMA_OPTIONS, 'num_predict': 40, 'temperature': 0}, 'messages': messages}
        try:
            if (answer := llm.ask_cloud(prompt, text, schema=schema, max_tokens=512)) is not None:
                result = json.loads(answer)
                if minimum == CONFIDENCE_THRESHOLD:  # a need; an unknown name keeps its stricter minimum
                    minimum = CLOUD_CONFIDENCE_THRESHOLD
            elif minimum > CONFIDENCE_THRESHOLD:
                # An unknown name ("walk me to the bottom"): the local model claims to be sure of a wrong place every time
                # ("iphone" -> computer room, "dean office" -> office), so offline the user hears "I don't know where".
                return None
            else:  # no key, offline, or Claude was slow: the local model, for a need ("I'm hungry" -> pret)
                request = urllib.request.Request(url, json.dumps(body).encode(), {'Content-Type': 'application/json'})
                with urllib.request.urlopen(request, timeout=timeout) as response:
                    result = json.loads(json.load(response)['message']['content'])
        except (urllib.error.URLError, TimeoutError, KeyError, ValueError):
            return None
        try:
            confidence = float(result.get('confidence') or 0)
        except (TypeError, ValueError):
            confidence = 0.0
        match = result.get('match') or ''
        if not match or confidence < minimum:
            return None
        return resolve_known(match, text, locations, saved)

    interpret.warm_up = lambda: warm_up_qwen(url, model)
    interpret.set_places = set_places
    return interpret


def warm_up_qwen(url=OLLAMA_URL, model=QWEN_MODEL):
    """Load the model now, so the first request is not slow. False if Ollama is not reachable."""
    try:
        request = urllib.request.Request(url, json.dumps({'model': model, 'messages': [], 'keep_alive': -1, 'options': OLLAMA_OPTIONS}).encode(), {'Content-Type': 'application/json'})
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


# Something the person needs, as opposed to a question about what the camera sees.
_NEED = re.compile(r"\b(i'd like|i would like|i need|i want|somewhere to|something to (eat|drink)|i'm (thirsty|hungry)|i am (thirsty|hungry))\b")


def classify_by_rules(text, locations=LOCATIONS, require_wake_word=False, saved=None):
    """Turn one spoken request into a Decision with the rules alone. None if there is nothing to act on."""
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
    if _SAVE_PLACE.search(text):
        match = _SAVE_NAME.search(text)
        name = clean_target(match.group('name')) if match else ''
        if name in ('well', 'too', 'also'):  # "remember this as well" has no name
            name = ''
        if not name:
            return Decision(None, text=text, reply='What should I call this place? Say: remember this place as, and the name.')
        if _STOP.search(name) or _HELP.search(name) or _FOLLOW.search(name) or _MANUAL.search(name) or _GPS.search(name):
            return Decision(None, text=text, reply=f"I can't call a place {name}. Pick another name.")  # it would be heard as a command
        return Decision(None, text=text, save_place=name, reply=f'Remembering this place as {name}.')
    if _MANUAL.search(text):
        return manual_decision(text)
    if _LOOK.search(text):
        return to_perception(text)
    if _NO_DESTINATION.search(text):
        return Decision(None, text=text, reply='Where to?')
    if _FIND.search(text) and not (_GUIDE.search(text) or _NEED_PLACE.search(text) or _GPS.search(text)):
        match = _FIND_TARGET.search(text)
        return resolve(clean_target(match.group('target')) if match else '', text, locations, saved)  # "where is the cup" = "take me to the cup"
    if match := _GPS_HERE.search(text):
        place = strip_area(clean_target(match.group('target')))
        if not place:
            return Decision(None, text=text, reply="Where are you? Say: GPS, I'm at, and the place.")
        return Decision(None, text=text, gps_here=place, reply=f'Looking up {place}.')
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


def classify(text, locations=LOCATIONS, require_wake_word=False, saved=None, interpreter=None):
    """Turn one spoken request into a Decision. None if there is nothing to act on.

    The rules decide first: a stop, help or "take me to the bathroom" never waits for the language model. Only when the
    rules do not understand ("Say that again") or the place is not one they know, interpreter (see qwen_interpreter)
    is asked whether the request is a need that one of the stored places meets ("I'm hungry")."""
    decision = classify_by_rules(text, locations, require_wake_word, saved)
    if interpreter and decision is not None and decision.mode is None:
        request = strip_wake_word(normalise(text), require_wake_word)
        # A single unfamiliar word is nearly always a mis-heard name ("prud" for Pret), not a description: it gets the honest
        # refusal, not a guess by meaning (it was taken for the prayer room).
        unknown_place = decision.reply.startswith("I don't know where") and len((request or '').split()) >= 5
        # Nothing but filler words ("go", "okay", "come on"): there is no need to match, so do not let the model guess one
        has_content = any(word not in _VAGUE_WORDS for word in (request or '').split())
        not_understood = (decision.reply == 'Say that again.' and has_content) or unknown_place
        a_need = decision.forward and _NEED.search(request or '')  # "I want a coffee" is sent on to perception, which can only describe
        # Asked for a named place that is not stored ("the dean's office"): only a very sure answer may replace the refusal.
        sure = 0.9 if decision.reply.startswith("I don't know where") else CONFIDENCE_THRESHOLD
        if request and (not_understood or a_need) and (guess := interpreter(request, sure)):
            return guess
    return decision


# What Pete says when the user ends the mode they are in.
LEAVING_REPLIES = {Mode.MANUAL: 'Manual mode off.', Mode.FOLLOW: 'Stopped following.'}


class Middleman:
    """Holds the current mode. Feed it requests with handle(), and report
    problems from the active mode with fail() and finish().

    handle() may be called from several threads, so a "stop" is handled while a slow request is still with the
    language model. Once a stop or help has been heard, any earlier request still being read is dropped when it
    finishes, so it cannot start a mode after the stop."""

    def __init__(self, locations=LOCATIONS, require_wake_word=False, saved=None, interpreter=None, on_ack=None):
        self.catalog_locations = list(locations)
        self.locations = list(locations)  # the catalog's places, then the ones saved by voice (set_saved_places)
        self.saved = dict(saved or {})
        self.require_wake_word = require_wake_word
        self.interpreter = interpreter
        self.on_ack = on_ack
        self.mode = Mode.IDLE
        self.current = None
        self._lock = threading.RLock()
        self._epoch = 0  # raised by every stop or help, to cancel the requests heard before it

    def set_saved_places(self, names):
        """The places navigation has saved ("remember this place as the balcony"). They are understood like the catalog's
        places, and replace the earlier list, so a forgotten place goes away."""
        with self._lock:
            self.locations = self.catalog_locations + [name for name in names if name not in self.catalog_locations]
            if hasattr(self.interpreter, 'set_places'):
                self.interpreter.set_places(self.locations)  # the language model is told about them too

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
            if decision.forward and not decision.reply and self.on_ack:
                self.on_ack()  # no spoken reply: perception answers later, so beep to say it was heard
            if decision.mode is None:
                return decision  # stay in the current mode: speak the reply, or send the text to perception
            if decision.stop:
                decision.reply = LEAVING_REPLIES.get(self.mode, decision.reply)
            decision.request_id = uuid.uuid4().hex
            self.mode, self.current = decision.mode, decision
            return decision

    def following_status(self, payload):
        """End only the FOLLOW request named by terminal status feedback.

        Ignore progress, duplicates, standalone statuses and older requests.
        Return an idle Decision for the ROS wrapper to announce and speak.
        """
        replies = {
            'target_lost': 'I lost the person. Following stopped.',
            'input_stale': 'Person tracking stopped updating. Following stopped.',
            'input_fault': 'There is a problem with person tracking. Following stopped.',
            'unsupported_target': 'Following a named contact is not available yet.',
            'command_invalid': 'The following command was invalid. Following stopped.',
            'disabled': 'Stopped following.',
            'transform_unavailable': 'The navigation position is unavailable. Following stopped.',
            'navigation_unavailable': 'Navigation is not available. Following stopped.',
            'navigation_failed': 'Navigation failed. Following stopped.',
            'navigation_cancel_failed': "I couldn't confirm navigation stopped. Please stop the robot.",
        }
        if not isinstance(payload, dict):
            return None
        event = payload.get('event')
        request_id = payload.get('request_id')
        if (payload.get('state') != 'IDLE' or not isinstance(event, str)
                or event not in replies or not isinstance(request_id, str) or not request_id):
            return None
        with self._lock:
            if (self.mode is not Mode.FOLLOW or self.current is None
                    or request_id != self.current.request_id):
                return None
            self._epoch += 1  # invalidate an earlier slow interpretation
            decision = Decision(Mode.IDLE, reply=replies[event], request_id=uuid.uuid4().hex)
            self.mode, self.current = Mode.IDLE, decision
            return decision

    def finish(self):
        """The active mode reached its goal. LOCATION, OBJECT and GPS have a goal, so whoever drives them calls this
        on arrival. FOLLOW uses following_status() for failures; MANUAL ends on user stop."""
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

    def not_found(self, goal):
        """Perception searched for goal and did not find it. Returns the Decision to act on, or None if the report is
        not about the object being guided to (a late report, or one after a stop). The Decision's reply says what
        went wrong. Its mode is LOCATION if a stored place stands in for the object, otherwise None and the mode is IDLE."""
        with self._lock:
            current = self.current
            if not current or current.mode is not Mode.OBJECT or current.target != goal:
                return None
            return self.fail() or Decision(None, text=current.text, reply=f"I couldn't find the {goal}.")


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
