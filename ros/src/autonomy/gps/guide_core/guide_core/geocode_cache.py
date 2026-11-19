"""On-disk cache of online place searches, so places found once also work offline.

Every answer from Nominatim is saved (also "no results", because repeating
identical queries can get us blocked). The geocoder node looks here first:
a fresh entry is used without any request; a stale one is refreshed when
online, and still used when offline (an old answer beats none).

The file must survive software updates (there is only one robot):
- it lives outside the workspace (default ~/.local/share/navigoat/), so
  rebuilds and git never touch it;
- it carries a version number; an older file is migrated, a file written by
  a NEWER program is opened read-only (never overwritten by older code);
- unknown top-level and per-entry fields are kept, and entries this version
  cannot read are written back unchanged;
- every save first copies the previous file to <file>.bak, then writes a
  temporary file and renames it over the old one (a crash mid-write cannot
  corrupt the cache);
- a file that cannot be read at all is renamed to <file>.corrupt-<time>,
  never deleted, and the cache starts empty.

File format (version 1), JSON:
    {"version": 1, "provider": "nominatim", "attribution": "...",
     "entries": {"science museum": {"query": "Science Museum",
                                    "fetched_utc": "2026-11-15T13:51:00+00:00",
                                    "places": [{"name": ..., "lat": ..., ...}]}}}
Keys are the query after destinations.normalise ("The Science Museum" and
"science museum" share one entry). "places": [] means "no results".

No ROS. Times are Unix seconds (wall clock) passed in by the caller.
"""

from dataclasses import dataclass, fields
import datetime
import json
import math
import os
from pathlib import Path
import shutil
from typing import Optional, Tuple

from guide_core.destinations import normalise
from guide_core.geocoding import ATTRIBUTION, NOMINATIM_SOURCE, Place

CACHE_VERSION = 1
DEFAULT_CACHE_FILE = '~/.local/share/navigoat/geocode_cache.json'
DAY = 86400.0

# Migrations from older file versions: MIGRATIONS[n] turns a version-n
# document (a dict) into a version-(n + 1) one. Empty while only version 1
# exists; add one here whenever CACHE_VERSION goes up.
MIGRATIONS = {}

_PLACE_TYPES = {'name': str, 'lat': float, 'lon': float, 'road': str, 'osm_type': str,
                'osm_id': int, 'category': str, 'kind': str, 'size': float}
_ENTRY_KEYS = {'query', 'fetched_utc', 'places'}
_TOP_KEYS = {'version', 'provider', 'attribution', 'entries'}


@dataclass(frozen=True)
class CacheConfig:
    """How long entries stay fresh.

    max_age_days: places found. Buildings rarely move; refreshed only when a
        user asks for the place again while online (never in the background:
        periodic requests count as bulk use under the Nominatim policy).
    negative_max_age_days: "no results" answers; shorter, since OSM may add
        the place and the user may try the name again.
    """

    max_age_days: float = 180.0
    negative_max_age_days: float = 7.0

    def __post_init__(self):
        for name in ('max_age_days', 'negative_max_age_days'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be > 0, got {value!r}')


@dataclass(frozen=True)
class CacheEntry:
    """One saved answer. places == () means Nominatim found nothing."""

    query: str                 # as first asked (for people reading the file)
    fetched: float             # Unix seconds
    places: Tuple[Place, ...]
    extra: Tuple[Tuple[str, object], ...] = ()   # unknown fields, written back unchanged


@dataclass(frozen=True)
class CacheHit:
    """Result of GeocodeCache.get()."""

    query: str
    places: Tuple[Place, ...]  # after the area filter; () = "no results" was saved
    fetched: float
    age: float                 # seconds since fetched (0 if the clock says the future)
    fresh: bool                # False: refresh it if online, use it if not


@dataclass(frozen=True)
class ParsedCache:
    """A cache document read into memory (see parse_cache)."""

    entries: dict              # key -> CacheEntry
    unreadable: dict           # key -> raw entry this version cannot read (kept)
    extra: dict                # unknown top-level fields (kept)
    version: int               # version of the document as read
    read_only: bool            # True: never save over it
    warnings: Tuple[str, ...]


class CacheFormatError(ValueError):
    """The document as a whole cannot be read."""


def _utc_text(timestamp):
    return datetime.datetime.fromtimestamp(timestamp, datetime.timezone.utc) \
        .isoformat(timespec='seconds')


def _timestamp(text):
    stamp = datetime.datetime.fromisoformat(text)
    if stamp.tzinfo is None:
        raise ValueError('time has no time zone')
    return stamp.timestamp()


def place_to_dict(place):
    """Place -> JSON-ready dict."""
    return {f.name: getattr(place, f.name) for f in fields(Place)}


def place_from_dict(data):
    """Build a Place from a dict; ValueError if a field is missing or of the wrong type."""
    if not isinstance(data, dict):
        raise ValueError('place must be a mapping')
    values = {}
    for name, kind in _PLACE_TYPES.items():
        value = data.get(name)
        if isinstance(value, bool):
            raise ValueError(f'place {name} must be {kind.__name__}')
        if kind is float and isinstance(value, int):
            value = float(value)
        if not isinstance(value, kind) or (kind is float and not math.isfinite(value)):
            raise ValueError(f'place {name} must be {kind.__name__}, got {value!r}')
        values[name] = value
    return Place(**values)


def _entry_from_dict(raw):
    if not isinstance(raw, dict):
        raise ValueError('entry must be a mapping')
    query = raw.get('query')
    if not isinstance(query, str) or not normalise(query):
        raise ValueError('entry query must be a non-empty string')
    fetched = raw.get('fetched_utc')
    if not isinstance(fetched, str):
        raise ValueError('entry fetched_utc must be a string')
    places = raw.get('places')
    if not isinstance(places, list):
        raise ValueError('entry places must be a list')
    extra = tuple((k, v) for k, v in raw.items() if k not in _ENTRY_KEYS)
    return CacheEntry(query, _timestamp(fetched),
                      tuple(place_from_dict(p) for p in places), extra)


def _entry_to_dict(entry):
    raw = {'query': entry.query, 'fetched_utc': _utc_text(entry.fetched),
           'places': [place_to_dict(p) for p in entry.places]}
    raw.update(dict(entry.extra))
    return raw


def parse_cache(data, migrations=None):
    """Read a parsed JSON document into a ParsedCache.

    Raises CacheFormatError if the document as a whole is unusable (not a
    mapping, no 'entries' mapping, no integer version). Single entries that
    cannot be read are kept in 'unreadable' and reported in 'warnings'.
    """
    migrations = MIGRATIONS if migrations is None else migrations
    if not isinstance(data, dict):
        raise CacheFormatError(f'expected a JSON object, got {type(data).__name__}')
    version = data.get('version')
    if isinstance(version, bool) or not isinstance(version, int) or version < 0:
        raise CacheFormatError(f'version must be a non-negative integer, got {version!r}')
    warnings = []
    read_only = False
    file_version = version
    while version < CACHE_VERSION:
        if version not in migrations:
            warnings.append(f'cache file is version {file_version} and there is no migration '
                            f'from version {version}: using what can be read, not saving')
            read_only = True
            break
        data = migrations[version](dict(data))
        version += 1
    else:
        if file_version < CACHE_VERSION:
            warnings.append(f'cache file migrated from version {file_version} to '
                            f'{CACHE_VERSION} (the old file is kept as .bak on the next save)')
    if file_version > CACHE_VERSION:
        warnings.append(f'cache file is version {file_version}, newer than this program '
                        f'({CACHE_VERSION}): using what can be read, not saving')
        read_only = True
    entries = data.get('entries')
    if not isinstance(entries, dict):
        raise CacheFormatError("'entries' must be a JSON object")

    parsed, unreadable = {}, {}
    for key, raw in entries.items():
        try:
            entry = _entry_from_dict(raw)
        except (ValueError, TypeError) as e:
            unreadable[key] = raw
            warnings.append(f'cache entry {key!r} cannot be read ({e}); kept, not used')
            continue
        parsed[normalise(entry.query)] = entry
    extra = {k: v for k, v in data.items() if k not in _TOP_KEYS}
    return ParsedCache(parsed, unreadable, extra, file_version, read_only, tuple(warnings))


class GeocodeCache:
    """The cache file, loaded into memory. Call load() once, then get()/put()."""

    def __init__(self, path=DEFAULT_CACHE_FILE, config=None, migrations=None):
        self.path = Path(os.path.expanduser(str(path)))
        self.config = config or CacheConfig()
        self._migrations = migrations
        self._entries = {}
        self._unreadable = {}
        self._extra = {}
        self.read_only = False

    def __len__(self):
        return len(self._entries)

    def load(self):
        """Read the file (a missing file is an empty cache). Returns warnings to log.

        An unreadable file is renamed to <file>.corrupt-<UTC time> (kept for
        inspection) and the cache starts empty.
        """
        self._entries, self._unreadable, self._extra = {}, {}, {}
        self.read_only = False
        if not self.path.exists():
            return ()
        try:
            with open(self.path, encoding='utf-8') as f:
                parsed = parse_cache(json.load(f), self._migrations)
        except (OSError, ValueError) as e:   # JSONDecodeError and CacheFormatError too
            stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            moved = self.path.with_name(f'{self.path.name}.corrupt-{stamp}')
            try:
                os.replace(self.path, moved)
            except OSError as move_error:
                self.read_only = True   # cannot move it away: do not overwrite it either
                return (f'cache file {self.path} cannot be read ({e}) or moved '
                        f'({move_error}); starting empty, not saving',)
            return (f'cache file {self.path} cannot be read ({e}); moved to {moved.name}, '
                    'starting empty',)
        self._entries = dict(parsed.entries)
        self._unreadable = dict(parsed.unreadable)
        self._extra = dict(parsed.extra)
        self.read_only = parsed.read_only
        return parsed.warnings

    def get(self, query, now, area=None):
        """Return the saved answer for query, or None if never saved.

        area: a geocoding.BBox; saved places outside it are left out (the
        graph area may have changed since they were saved).
        """
        entry = self._entries.get(normalise(query))
        if entry is None:
            return None
        places = entry.places
        if area is not None:
            places = tuple(p for p in places if area.contains(p.lat, p.lon))
        age = max(0.0, now - entry.fetched)
        max_age = self.config.max_age_days if entry.places else self.config.negative_max_age_days
        return CacheHit(entry.query, places, entry.fetched, age, age <= max_age * DAY)

    def put(self, query, places, now):
        """Save an answer (places may be () for "no results") and write the file.

        Replaces an earlier entry for the same normalised query, keeping its
        unknown fields. Returns True if the file was written, False if the
        cache is read-only. OSError from writing is passed on.
        """
        key = normalise(query)
        if not key:
            raise ValueError(f'cannot cache an empty query {query!r}')
        old = self._entries.get(key)
        self._entries[key] = CacheEntry(' '.join(query.split()), float(now), tuple(places),
                                        old.extra if old else ())
        self._unreadable.pop(key, None)
        return self.save()

    def to_dict(self):
        """Return the whole cache as a JSON-ready document."""
        entries = dict(self._unreadable)
        entries.update({k: _entry_to_dict(e) for k, e in self._entries.items()})
        document = {'version': CACHE_VERSION, 'provider': NOMINATIM_SOURCE,
                    'attribution': ATTRIBUTION, 'entries': entries}
        document.update(self._extra)
        return document

    def save(self):
        """Write the file safely: backup, temporary file, rename. False if read-only."""
        if self.read_only:
            return False
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            shutil.copy2(self.path, self.path.with_name(self.path.name + '.bak'))
        tmp = self.path.with_name(self.path.name + '.tmp')
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=1, ensure_ascii=False)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)
        return True

    def entry(self, query) -> Optional[CacheEntry]:
        """Return the raw saved entry for query (no area filter, no freshness), or None."""
        return self._entries.get(normalise(query))
