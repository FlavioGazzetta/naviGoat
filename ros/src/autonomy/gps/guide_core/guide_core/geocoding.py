"""Online place search with Nominatim: request parameters and response parsing.

When a destination is not in the curated list, the geocoder node asks
Nominatim (OpenStreetMap's search engine) for it. This module holds the
parts that need no network, so they can be tested with saved responses:

- BBox: the area our walking graph covers. Every request is limited to it
  (viewbox + bounded=1) and every result outside it (minus a margin, since
  the graph is cut off at its edges) is dropped: a place we cannot route to
  is useless.
- search_params(): the query string of one /search request. It contains
  only the cleaned query text and the FIXED graph area, never the robot's
  position or anything about the user (Nominatim usage policy: no personal
  data).
- parse_results(): Nominatim's JSON (format=jsonv2) -> Places, in
  Nominatim's relevance order, filtered, de-duplicated and limited.

Nominatim returns the centre of a building or area, not its entrance. Each
Place therefore carries its size (larger side of its bounding box), so the
mission can warn when the route only goes to the nearest path of a large
area.

Usage policy of the public server (re-checked 15 Nov 2026):
https://operations.osmfoundation.org/policies/nominatim/
Search results (c) OpenStreetMap contributors, ODbL.

Pure Python (no ROS, no network).
"""

from dataclasses import dataclass
import math
from typing import Tuple

from guide_core.destinations import normalise
from guide_core.geo import enu_to_latlon, latlon_to_enu

NOMINATIM_SOURCE = 'nominatim'
ATTRIBUTION = 'Search results (c) OpenStreetMap contributors (ODbL), via Nominatim'


class GeocodeError(ValueError):
    """A request cannot be built or a response is not what Nominatim sends."""


def _finite(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not math.isfinite(value):
        raise GeocodeError(f'{what} must be a finite number, got {value!r}')
    return float(value)


@dataclass(frozen=True)
class BBox:
    """A latitude/longitude rectangle in WGS84 degrees (edges included)."""

    min_lat: float
    min_lon: float
    max_lat: float
    max_lon: float

    def __post_init__(self):
        for name in ('min_lat', 'min_lon', 'max_lat', 'max_lon'):
            _finite(getattr(self, name), name)
        if not -90.0 <= self.min_lat < self.max_lat <= 90.0:
            raise GeocodeError(f'need -90 <= min_lat < max_lat <= 90, got '
                               f'{self.min_lat}, {self.max_lat}')
        if not -180.0 <= self.min_lon < self.max_lon <= 180.0:
            raise GeocodeError(f'need -180 <= min_lon < max_lon <= 180, got '
                               f'{self.min_lon}, {self.max_lon}')

    @classmethod
    def around(cls, lat, lon, half_width):
        """Return the square reaching half_width metres east, west, north and south.

        It is centred on (lat, lon): the area osmnx downloads for
        graph_from_point(dist=half_width).
        """
        half_width = _finite(half_width, 'half_width')
        if half_width <= 0.0:
            raise GeocodeError(f'half_width must be > 0, got {half_width}')
        min_lat, min_lon = enu_to_latlon(-half_width, -half_width, lat, lon)
        max_lat, max_lon = enu_to_latlon(half_width, half_width, lat, lon)
        return cls(min_lat, min_lon, max_lat, max_lon)

    @property
    def centre(self):
        """(lat, lon) of the middle of the box."""
        return (0.5 * (self.min_lat + self.max_lat), 0.5 * (self.min_lon + self.max_lon))

    def contains(self, lat, lon):
        """Return True if (lat, lon) is inside the box or on its edge."""
        return self.min_lat <= lat <= self.max_lat and self.min_lon <= lon <= self.max_lon

    def shrunk(self, margin):
        """Return the box with every edge moved margin metres inwards."""
        margin = _finite(margin, 'margin')
        if margin < 0.0:
            raise GeocodeError(f'margin must be >= 0, got {margin}')
        lat0, lon0 = self.centre
        lat1, lon1 = enu_to_latlon(margin, margin, lat0, lon0)
        dlat, dlon = lat1 - lat0, lon1 - lon0
        if self.min_lat + dlat >= self.max_lat - dlat or \
                self.min_lon + dlon >= self.max_lon - dlon:
            raise GeocodeError(f'a {margin} m margin leaves nothing of the box')
        return BBox(self.min_lat + dlat, self.min_lon + dlon,
                    self.max_lat - dlat, self.max_lon - dlon)

    def viewbox(self):
        """Nominatim's viewbox parameter: 'left,top,right,bottom' (lon,lat,lon,lat)."""
        return (f'{self.min_lon:.7f},{self.max_lat:.7f},'
                f'{self.max_lon:.7f},{self.min_lat:.7f}')


def bbox_from_meta(meta):
    """Return the graph's area from its .meta.yaml (written by fetch_osm_graph).

    Uses an explicit 'bbox' mapping (min_lat, min_lon, max_lat, max_lon)
    if present; otherwise origin_lat/origin_lon/dist_m, the square osmnx
    downloaded (older meta files have only these).
    """
    if not isinstance(meta, dict):
        raise GeocodeError(f'graph metadata must be a mapping, got {type(meta).__name__}')
    box = meta.get('bbox')
    if box is not None:
        if not isinstance(box, dict):
            raise GeocodeError("'bbox' must be a mapping")
        try:
            return BBox(box['min_lat'], box['min_lon'], box['max_lat'], box['max_lon'])
        except KeyError as e:
            raise GeocodeError(f"'bbox' is missing {e}") from None
    missing = [k for k in ('origin_lat', 'origin_lon', 'dist_m') if k not in meta]
    if missing:
        raise GeocodeError(f'graph metadata has no bbox and is missing {missing}')
    return BBox.around(_finite(meta['origin_lat'], 'origin_lat'),
                       _finite(meta['origin_lon'], 'origin_lon'),
                       meta['dist_m'])


@dataclass(frozen=True)
class GeocodeConfig:
    """Settings for requests and parsing.

    max_results: Nominatim 'limit', and the most Places returned.
    edge_margin: metres; results closer than this to the graph's edge are
        dropped (the graph is cut off there, so routes would be poor).
    large_area_size: metres; a Place whose bounding box is at least this
        long on one side is a large area (centre far from any entrance).
    duplicate_distance: metres; two results with the same name this close
        are one place (Nominatim sometimes returns a building twice).
    language: accept-language for names ('en': names as English speakers say them).
    min_query_length: shorter queries (after normalising) are never sent;
        also a guard against accidental per-keystroke (autocomplete) use.
    """

    max_results: int = 5
    edge_margin: float = 30.0
    large_area_size: float = 100.0
    duplicate_distance: float = 30.0
    language: str = 'en'
    min_query_length: int = 3

    def __post_init__(self):
        if self.max_results < 1:
            raise GeocodeError(f'max_results must be >= 1, got {self.max_results}')
        for name in ('edge_margin', 'large_area_size', 'duplicate_distance'):
            if _finite(getattr(self, name), name) < 0.0:
                raise GeocodeError(f'{name} must be >= 0, got {getattr(self, name)}')
        if self.min_query_length < 1:
            raise GeocodeError(f'min_query_length must be >= 1, got {self.min_query_length}')


def searchable_query(query, config=None):
    """Return the query as it will be sent (spaces tidied), or '' if it must not be.

    Too short after normalising ("ab", "the", punctuation only) -> ''.
    """
    config = config or GeocodeConfig()
    text = ' '.join(str(query).split())
    return text if len(normalise(text)) >= config.min_query_length else ''


def search_params(query, bbox, config=None):
    """Query-string parameters of one Nominatim /search request.

    Only the query text and the fixed graph area are sent: no position,
    no email, nothing about the user.
    """
    config = config or GeocodeConfig()
    text = searchable_query(query, config)
    if not text:
        raise GeocodeError(f'query {query!r} is too short to search for')
    return {
        'q': text,
        'format': 'jsonv2',
        'limit': str(config.max_results),
        'addressdetails': '1',
        'accept-language': config.language,
        'viewbox': bbox.viewbox(),
        'bounded': '1',
    }


@dataclass(frozen=True)
class Place:
    """One search result, as we use it.

    name: what to say ("Science Museum"); road: the road Nominatim
    addresses it to ('' if none). Careful: that road is not necessarily
    where the entrance is (the Science Museum is addressed to Frankland or
    Armstrong Road; its entrance is on Exhibition Road).
    lat/lon: centre of the building or area (not an entrance).
    size: larger side of its bounding box in metres (0 if unknown).
    """

    name: str
    lat: float
    lon: float
    road: str
    osm_type: str
    osm_id: int
    category: str
    kind: str
    size: float

    @property
    def osm_ref(self):
        """'way/27765411': identifies the OSM object (logs, cache, debugging)."""
        return f'{self.osm_type}/{self.osm_id}'


@dataclass(frozen=True)
class ParseResult:
    """Places kept, and how many results were dropped and why (for logs)."""

    places: Tuple[Place, ...]
    outside: int = 0      # outside the graph area (or within edge_margin of its edge)
    invalid: int = 0      # malformed entries
    duplicates: int = 0   # same name within duplicate_distance of a kept place


def _text(value):
    return ' '.join(value.split()) if isinstance(value, str) else ''


def _coordinate(value, low, high):
    """Return Nominatim's string coordinate as a float, or None if unusable."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) and low <= number <= high else None


def _place_name(entry, address):
    """Spoken name: OSM name; else house number + road; else first part of display_name."""
    name = _text(entry.get('name'))
    if name:
        return name
    road = _text(address.get('road'))
    if road:
        number = _text(address.get('house_number'))
        return f'{number} {road}' if number else road
    return _text(_text(entry.get('display_name')).split(',')[0])


def _size(entry, lat, lon):
    """Larger side of the result's bounding box in metres, 0.0 if unknown."""
    box = entry.get('boundingbox')
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return 0.0
    south, north = (_coordinate(v, -90.0, 90.0) for v in box[:2])
    west, east = (_coordinate(v, -180.0, 180.0) for v in box[2:])
    if None in (south, north, west, east):
        return 0.0
    x0, y0 = latlon_to_enu(south, west, lat, lon)
    x1, y1 = latlon_to_enu(north, east, lat, lon)
    return max(abs(x1 - x0), abs(y1 - y0))


def _parse_entry(entry):
    """Place from one Nominatim result, or None if it is malformed."""
    if not isinstance(entry, dict):
        return None
    lat = _coordinate(entry.get('lat'), -90.0, 90.0)
    lon = _coordinate(entry.get('lon'), -180.0, 180.0)
    if lat is None or lon is None:
        return None
    address = entry.get('address')
    address = address if isinstance(address, dict) else {}
    name = _place_name(entry, address)
    if not normalise(name):
        return None
    osm_id = entry.get('osm_id')
    if isinstance(osm_id, bool) or not isinstance(osm_id, int):
        osm_id = 0
    return Place(name=name, lat=lat, lon=lon, road=_text(address.get('road')),
                 osm_type=_text(entry.get('osm_type')), osm_id=osm_id,
                 category=_text(entry.get('category')), kind=_text(entry.get('type')),
                 size=_size(entry, lat, lon))


def _distance(a, b):
    x, y = latlon_to_enu(b.lat, b.lon, a.lat, a.lon)
    return math.hypot(x, y)


def parse_results(data, bbox, config=None):
    """Turn a parsed /search response (format=jsonv2) into a ParseResult.

    Keeps Nominatim's order (most relevant first). Drops malformed entries,
    places outside bbox shrunk by config.edge_margin, and repeats of a kept
    place (same normalised name within config.duplicate_distance). Keeps at
    most config.max_results. A response that is not a list raises
    GeocodeError (Nominatim always answers a search with a list).
    """
    config = config or GeocodeConfig()
    if not isinstance(data, list):
        raise GeocodeError(f'expected a list of results, got {type(data).__name__}')
    area = bbox.shrunk(config.edge_margin)
    kept, outside, invalid, duplicates = [], 0, 0, 0
    for entry in data:
        place = _parse_entry(entry)
        if place is None:
            invalid += 1
        elif not area.contains(place.lat, place.lon):
            outside += 1
        elif any(normalise(p.name) == normalise(place.name)
                 and _distance(p, place) <= config.duplicate_distance for p in kept):
            duplicates += 1
        elif len(kept) < config.max_results:
            kept.append(place)
    return ParseResult(tuple(kept), outside, invalid, duplicates)


def is_large_area(place, config=None):
    """Return True if the place is so big that its centre may be far from any entrance."""
    config = config or GeocodeConfig()
    return place.size >= config.large_area_size


def labels(places):
    """Spoken label per place: the name, plus ', <road>' when names repeat.

    "Imperial College London" twice -> "Imperial College London, Exhibition
    Road" and "Imperial College London, Prince Consort Road", so the user
    can tell options apart by ear.
    """
    keys = [normalise(p.name) for p in places]
    result = []
    for place, key in zip(places, keys):
        if keys.count(key) > 1 and place.road:
            result.append(f'{place.name}, {place.road}')
        else:
            result.append(place.name)
    return tuple(result)
