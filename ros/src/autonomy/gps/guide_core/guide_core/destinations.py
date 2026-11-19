"""Curated destination list: load it and match what the user asked for.

The user (blind) names a destination ("the library"); speech-to-text turns it
into text. resolve() finds the matching entries of a curated list, ranks
them and decides whether the answer is clear enough to plan a route:

- UNIQUE: one clear match (an exact name/alias, or one fuzzy match that is
  clearly better than the rest). It is still read back to the user before
  the robot moves (READY state of the mission manager).
- AMBIGUOUS: several plausible matches, or a single uncertain one; the
  user must choose ("Option 1: ... Which one?" / "Did you mean ...?").
- NOT_FOUND: nothing matched well enough.

Every Resolution carries a sentence ready to be spoken. Coordinates are for
the planner only and never appear in messages.

Pure Python (no ROS): the mission manager node loads the YAML and copies the
results into ROS messages.
"""

from dataclasses import dataclass
import difflib
from enum import Enum
import math
import re
from typing import Optional, Tuple
import unicodedata

from guide_core.geo import latlon_to_enu

DESTINATION_KEYS = {'name', 'lat', 'lon', 'aliases', 'note'}
SUBSET_SCORE = 0.9  # score when every word the user said appears in a name/alias


class DestinationError(ValueError):
    """The destination list is malformed."""


@dataclass(frozen=True)
class Destination:
    """One place the robot can take the user to.

    lat/lon are WGS84 degrees of the point to walk to: for curated entries
    the (preferably step-free) entrance, not the building's centre.
    source: where the entry comes from: 'curated' (the team's list, checked
    by hand) or 'nominatim' (an online search result: the centre of a
    building or area, less trusted, always read back before moving).
    """

    name: str
    lat: float
    lon: float
    aliases: Tuple[str, ...] = ()
    note: str = ''
    source: str = 'curated'


class MatchType(Enum):
    """How a candidate matched the query."""

    EXACT = 'exact'   # query equals the name or an alias (after normalising)
    FUZZY = 'fuzzy'   # close spelling or some of the words


class Outcome(Enum):
    """What the caller should do with a Resolution."""

    UNIQUE = 'unique'
    AMBIGUOUS = 'ambiguous'
    NOT_FOUND = 'not_found'


@dataclass(frozen=True)
class Candidate:
    """A destination that matched, with how well and how far away."""

    destination: Destination
    matched_text: str            # the name or alias that matched, as written in the list
    match_type: MatchType
    score: float                 # 0..1, 1.0 = exact
    distance: Optional[float]    # straight-line metres from the robot; None if unknown


@dataclass(frozen=True)
class Resolution:
    """Result of resolve(): the outcome, ranked candidates and a spoken sentence."""

    query: str
    outcome: Outcome
    candidates: Tuple[Candidate, ...]
    message: str


@dataclass(frozen=True)
class MatchConfig:
    """Thresholds for resolve().

    min_score: candidates scoring lower are dropped.
    unique_score / unique_margin: with no exact match, the best fuzzy match is
        UNIQUE only if it scores at least unique_score AND beats the second by
        at least unique_margin.
    max_candidates: how many candidates are returned at most.
    max_read_back: how many are read out in an AMBIGUOUS message (lists are
        hard to follow by ear).
    """

    min_score: float = 0.5
    unique_score: float = 0.85
    unique_margin: float = 0.15
    max_candidates: int = 5
    max_read_back: int = 3

    def __post_init__(self):
        if not 0.0 < self.min_score <= 1.0:
            raise ValueError(f'min_score must be in (0, 1], got {self.min_score}')
        if not self.min_score <= self.unique_score <= 1.0:
            raise ValueError('unique_score must be between min_score and 1, '
                             f'got {self.unique_score}')
        if self.unique_margin < 0.0:
            raise ValueError(f'unique_margin must be >= 0, got {self.unique_margin}')
        if self.max_candidates < 1 or self.max_read_back < 1:
            raise ValueError('max_candidates and max_read_back must be >= 1')


def normalise(text):
    """Return text in a canonical form for comparing names.

    Lower case, accents removed ("Café" -> "cafe"), apostrophes removed
    ("Queen's" -> "queens"), "&" -> "and", other punctuation -> spaces,
    repeated spaces collapsed, and a leading "the" dropped when other words
    follow ("The Library" -> "library").
    """
    text = unicodedata.normalize('NFKD', text)
    text = ''.join(c for c in text if not unicodedata.combining(c)).lower()
    text = re.sub(r"['\u2019]", '', text)  # straight and curly apostrophes
    text = text.replace('&', ' and ')
    words = re.sub(r'[^a-z0-9]+', ' ', text).split()
    if len(words) > 1 and words[0] == 'the':
        words = words[1:]
    return ' '.join(words)


def _number(entry, key, low, high, where):
    value = entry.get(key)
    # bool is a subclass of int in Python: reject it explicitly.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DestinationError(f'{where}: {key} must be a number, got {value!r}')
    value = float(value)
    if not (math.isfinite(value) and low <= value <= high):
        raise DestinationError(f'{where}: {key} must be in [{low}, {high}], got {value}')
    return value


def destinations_from_dict(data):
    """Build Destinations from a parsed YAML document {'destinations': [...]}.

    Each entry: name (required), lat, lon (required, the entrance), aliases
    (optional list of strings), note (optional string). Unknown keys are
    errors (a typo like 'alias' would otherwise be silently ignored). Two
    entries whose names are equal after normalising are an error; an alias
    shared by several entries is allowed (resolve() then reports AMBIGUOUS).
    """
    if not isinstance(data, dict) or 'destinations' not in data:
        raise DestinationError("Expected a mapping with a 'destinations' list")
    entries = data['destinations']
    if not isinstance(entries, list) or not entries:
        raise DestinationError("'destinations' must be a non-empty list")

    result = []
    seen_names = {}
    for i, entry in enumerate(entries):
        where = f'destination {i}'
        if not isinstance(entry, dict):
            raise DestinationError(f'{where}: must be a mapping, got {entry!r}')
        unknown = set(entry) - DESTINATION_KEYS
        if unknown:
            raise DestinationError(f'{where}: unknown keys {sorted(unknown)}')
        name = entry.get('name')
        if not isinstance(name, str) or not normalise(name):
            raise DestinationError(f'{where}: name must be a non-empty string, got {name!r}')
        where = f'destination {i} ({name})'
        aliases = entry.get('aliases', [])
        if not isinstance(aliases, list) or not all(
                isinstance(a, str) and normalise(a) for a in aliases):
            raise DestinationError(f'{where}: aliases must be a list of non-empty strings')
        note = entry.get('note', '')
        if not isinstance(note, str):
            raise DestinationError(f'{where}: note must be a string')
        key = normalise(name)
        if key in seen_names:
            raise DestinationError(f'{where}: same name as destination {seen_names[key]}')
        seen_names[key] = i
        result.append(Destination(
            name=name.strip(),
            lat=_number(entry, 'lat', -90.0, 90.0, where),
            lon=_number(entry, 'lon', -180.0, 180.0, where),
            aliases=tuple(a.strip() for a in aliases),
            note=note.strip()))
    return tuple(result)


def match_score(query_key, text_key):
    """Score how well a normalised query matches a normalised name or alias.

    1.0 if equal. Otherwise the larger of: difflib's similarity ratio
    (catches spelling mistakes: "libary" vs "library") and SUBSET_SCORE if
    every word of the query appears in the text ("kensington" in "south
    kensington station").
    """
    if not query_key or not text_key:
        return 0.0
    if query_key == text_key:
        return 1.0
    ratio = difflib.SequenceMatcher(None, query_key, text_key).ratio()
    subset = SUBSET_SCORE if set(query_key.split()) <= set(text_key.split()) else 0.0
    return max(ratio, subset)


def format_distance(metres):
    """Distance as spoken: 'about 300 m' (nearest 10 m, at least 10) or 'about 1.2 km'."""
    rounded = max(10, int(math.floor(metres / 10.0 + 0.5)) * 10)
    if rounded >= 1000:
        return f'about {metres / 1000.0:.1f} km'
    return f'about {rounded} m'


def describe(candidate):
    """Return 'Central Library, main entrance, step-free, about 300 m' (parts that exist)."""
    parts = [candidate.destination.name]
    if candidate.destination.note:
        parts.append(candidate.destination.note)
    if candidate.distance is not None:
        parts.append(format_distance(candidate.distance))
    return ', '.join(parts)


def _best_match(query_key, destination):
    """(score, matched_text) for the destination's best-matching name or alias."""
    best = (0.0, destination.name)
    for text in (destination.name,) + destination.aliases:
        score = match_score(query_key, normalise(text))
        if score > best[0]:  # strict: on a tie the name (listed first) wins
            best = (score, text)
    return best


def resolve(query, destinations, robot_latlon=None, config=None):
    """Find, rank and judge the destinations matching what the user said.

    robot_latlon: (lat, lon) of the robot, or None if unknown; used for the
    straight-line distance in each candidate and to break ties (nearer first).
    Returns a Resolution (see the module docstring for the outcomes).
    """
    config = config or MatchConfig()
    spoken = ' '.join(query.split())
    query_key = normalise(query)
    if not query_key:
        return Resolution(spoken, Outcome.NOT_FOUND, (), "I didn't catch a destination name.")

    candidates = []
    for destination in destinations:
        score, matched_text = _best_match(query_key, destination)
        if score < config.min_score:
            continue
        distance = None
        if robot_latlon is not None:
            x, y = latlon_to_enu(destination.lat, destination.lon, *robot_latlon)
            distance = math.hypot(x, y)
        candidates.append(Candidate(
            destination=destination, matched_text=matched_text,
            match_type=MatchType.EXACT if score == 1.0 else MatchType.FUZZY,
            score=score, distance=distance))

    if not candidates:
        return Resolution(spoken, Outcome.NOT_FOUND, (),
                          f"I don't know a place called '{spoken}'.")

    # Best score first; ties: nearer first (unknown distance last), then by name.
    candidates.sort(key=lambda c: (-c.score,
                                   c.distance if c.distance is not None else math.inf,
                                   c.destination.name))

    exact_count = sum(1 for c in candidates if c.match_type is MatchType.EXACT)
    if exact_count == 1:
        unique = True
    elif exact_count > 1:
        unique = False
    else:
        best = candidates[0].score
        second = candidates[1].score if len(candidates) > 1 else 0.0
        unique = best >= config.unique_score and best - second >= config.unique_margin

    shown = tuple(candidates[:config.max_candidates])
    if unique:
        return Resolution(spoken, Outcome.UNIQUE, shown, describe(shown[0]) +
                          (' away.' if shown[0].distance is not None else '.'))

    if len(candidates) == 1:
        # One match, but not a confident one (e.g. a bad misspelling): ask.
        return Resolution(spoken, Outcome.AMBIGUOUS, shown,
                          f'Did you mean {describe(shown[0])}?')
    read_back = candidates[:config.max_read_back]
    options = ' '.join(f'Option {i}: {describe(c)}.' for i, c in enumerate(read_back, start=1))
    more = len(candidates) - len(read_back)
    message = (f"I found {len(candidates)} places for '{spoken}'. {options}"
               + (f' And {more} more.' if more > 0 else '') + ' Which one?')
    return Resolution(spoken, Outcome.AMBIGUOUS, shown, message)
