"""Turn the geocoder's answer into a Resolution, like destinations.resolve() does.

Used when the curated list had no match (destinations.resolve() returned
NOT_FOUND) and the geocoder (guide/geocode) was asked instead. Online
results are less trusted than curated entries, so:

- one result -> UNIQUE, but the mission still reads it back in READY and
  waits for the user's go (skip_confirmation does not apply to it);
- several -> AMBIGUOUS ("I found 5 places in OpenStreetMap for 'museum'.
  Option 1: ... Which one?"), in the geocoder's order (most relevant first);
- none, or the lookup failed -> NOT_FOUND with a sentence saying why
  (nothing found / online search off / could not search / paused).

Separate from destinations.py because it needs the geocoder's status codes
(and the geocoder already imports destinations). Pure Python, no ROS.
"""

import math

from guide_core.destinations import (
    Candidate, describe, match_score, MatchConfig, MatchType, normalise, Outcome, Resolution)
from guide_core.geo import latlon_to_enu
from guide_core.geocoder import Status

LARGE_AREA_NOTE = 'large area'              # DestinationCandidate.note set by the geocoder
FOUND_ONLINE = 'OpenStreetMap'
FOUND_IN_CACHE = 'saved OpenStreetMap results'


def not_found_text(spoken, status):
    """Return the spoken sentence for "not in the list, and nothing usable online"."""
    base = f"I don't know a place called '{spoken}'"
    if status in (Status.OK, Status.NO_RESULTS):
        return f'{base}, and OpenStreetMap has nothing by that name near here.'
    if status is Status.DISABLED:
        return f'{base}. Online search is switched off.'
    if status is Status.RATE_LIMITED:
        return f'{base}, and online search is paused for now. Try another name.'
    return f"{base}, and I couldn't search online just now. Try another name."


def resolve_online(query, status, destinations, robot_latlon=None, from_cache=False,
                   config=None):
    """Judge the geocoder's answer for query.

    status: the Geocode response status (an int or geocoder.Status; unknown
    values count as ERROR). destinations: its candidates as Destinations
    (source 'nominatim'), in the geocoder's order. robot_latlon: for the
    distance in each candidate. from_cache: the answer came from the cache
    (the messages then say "saved OpenStreetMap results").
    """
    config = config or MatchConfig()
    spoken = ' '.join(query.split())
    try:
        status = Status(int(status))
    except ValueError:
        status = Status.ERROR
    destinations = tuple(destinations) if status is Status.OK else ()
    if not destinations:
        return Resolution(spoken, Outcome.NOT_FOUND, (), not_found_text(spoken, status))

    query_key = normalise(spoken)
    candidates = []
    for destination in destinations[:config.max_candidates]:
        distance = None
        if robot_latlon is not None:
            distance = math.hypot(*latlon_to_enu(destination.lat, destination.lon,
                                                 *robot_latlon))
        candidates.append(Candidate(
            destination=destination, matched_text=destination.name,
            match_type=MatchType.FUZZY, score=match_score(query_key, normalise(destination.name)),
            distance=distance))
    candidates = tuple(candidates)
    where = FOUND_IN_CACHE if from_cache else FOUND_ONLINE

    if len(candidates) == 1:
        (only,) = candidates
        return Resolution(spoken, Outcome.UNIQUE, candidates,
                          f'Found in {where}: {describe(only)}'
                          + (' away.' if only.distance is not None else '.'))
    read_back = candidates[:config.max_read_back]
    options = ' '.join(f'Option {i}: {describe(c)}.' for i, c in enumerate(read_back, start=1))
    more = len(candidates) - len(read_back)
    message = (f"I found {len(candidates)} places in {where} for '{spoken}'. {options}"
               + (f' And {more} more.' if more > 0 else '') + ' Which one?')
    return Resolution(spoken, Outcome.AMBIGUOUS, candidates, message)
