"""Where the simulated GPS is good, poor or absent (Step 8).

A zone is a polyline widened by half_width metres on each side (a single
point gives a circle). Each zone carries the GpsConditions that apply inside
it; outside all zones the map's default conditions apply. If zones overlap,
the worst signal wins: no fix beats any fix, then the larger reported sigma.

Tunnel zones come from the routing graph's tunnel edges (walk_costs.is_tunnel,
the same test the route planner uses), so the sim loses GPS exactly where the
planner warns about tunnels. Zones are flat (2D), but a subway often runs
under a street: so a tunnel zone only applies where the nearest edge of the
graph IS a tunnel edge (surface check). On the pavement above a subway the
nearest edge is that pavement and GPS stays; inside the subway the nearest edge
is the subway. Extra zones (e.g. an urban canyon) come from a YAML dictionary
with points in lat/lon. Only the simulator uses this module:
the real robot decides that GPS is poor from the fixes it receives.

Works on any networkx-style graph (osmnx or plain networkx.read_graphml):
node coordinates 'x' (lon) / 'y' (lat) may be numbers or strings, and edge
'geometry' may be a shapely LineString, a WKT string or missing.
"""

from dataclasses import dataclass
import math

from guide_core.geo import latlon_to_enu
from guide_core.gps_error import conditions_from_dict, GpsConditions, PROFILES
from guide_core.walk_costs import is_tunnel

TUNNEL_ZONE_NAME = 'tunnel'
DEFAULT_TUNNEL_HALF_WIDTH = 5.0
# Grid cell size (m) of the surface edge index.
SURFACE_CELL = 20.0


@dataclass(frozen=True)
class GpsZone:
    """A widened polyline (ENU metres) with the GPS conditions inside it."""

    name: str
    conditions: GpsConditions
    points: tuple
    half_width: float
    # True for tunnels: the zone does not apply where a surface edge is nearer.
    yields_to_surface: bool = False

    def __post_init__(self):
        if not self.points:
            raise ValueError(f'zone {self.name!r} has no points')
        if not (math.isfinite(self.half_width) and self.half_width >= 0.0):
            raise ValueError(f'zone {self.name!r}: half_width must be >= 0')
        object.__setattr__(self, 'points',
                           tuple((float(x), float(y)) for x, y in self.points))

    def bounds(self):
        """Return (min_x, min_y, max_x, max_y), widened by half_width."""
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        w = self.half_width
        return (min(xs) - w, min(ys) - w, max(xs) + w, max(ys) + w)

    def distance(self, x, y):
        """Return the distance from (x, y) to the zone's centre line."""
        if len(self.points) == 1:
            px, py = self.points[0]
            return math.hypot(x - px, y - py)
        return min(_segment_distance(x, y, a, b)
                   for a, b in zip(self.points, self.points[1:]))

    def contains(self, x, y):
        """Return True if (x, y) lies inside the zone."""
        return self.distance(x, y) <= self.half_width


def _segment_distance(x, y, a, b):
    """Return the distance from (x, y) to the segment a-b."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0.0 else ((x - ax) * dx + (y - ay) * dy) / length_sq
    t = min(1.0, max(0.0, t))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def severity(conditions):
    """Return a sort key: larger means a worse GPS signal."""
    return (not conditions.fix_available, conditions.reported_sigma,
            conditions.bias_sigma, conditions.jumps_per_minute)


class SegmentIndex:
    """Grid index of line segments, for "nearest segment within r" queries."""

    def __init__(self, segments, cell=SURFACE_CELL):
        self.cell = float(cell)
        self._cells = {}
        self.count = 0
        for a, b in segments:
            a = (float(a[0]), float(a[1]))
            b = (float(b[0]), float(b[1]))
            self.count += 1
            for key in self._keys(min(a[0], b[0]), min(a[1], b[1]),
                                  max(a[0], b[0]), max(a[1], b[1])):
                self._cells.setdefault(key, []).append((a, b))

    def _keys(self, x0, y0, x1, y1):
        """Return the grid cells overlapping a bounding box."""
        c = self.cell
        return [(i, j)
                for i in range(math.floor(x0 / c), math.floor(x1 / c) + 1)
                for j in range(math.floor(y0 / c), math.floor(y1 / c) + 1)]

    def nearest_distance(self, x, y, max_distance):
        """Return the distance to the nearest segment, or inf if none within max_distance."""
        best = math.inf
        r = max_distance
        for key in self._keys(x - r, y - r, x + r, y + r):
            for a, b in self._cells.get(key, ()):
                best = min(best, _segment_distance(x, y, a, b))
        return best if best <= max_distance else math.inf


class GpsZoneMap:
    """Look up the GPS conditions at a position.

    surface: optional SegmentIndex of all non-tunnel edges; zones with
    yields_to_surface (tunnels) do not apply where a surface edge is nearer
    than the zone's centre line.
    """

    def __init__(self, default, zones=(), surface=None):
        self.default = default
        self.zones = tuple(zones)
        self.surface = surface
        self._bounds = [zone.bounds() for zone in self.zones]

    def zones_at(self, x, y):
        """Return all zones containing (x, y), in map order."""
        found = []
        for zone, (x0, y0, x1, y1) in zip(self.zones, self._bounds):
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            distance = zone.distance(x, y)
            if distance > zone.half_width:
                continue
            if zone.yields_to_surface and self.surface is not None \
                    and self.surface.nearest_distance(x, y, distance) < distance:
                continue      # on a surface path above / beside the tunnel
            found.append(zone)
        return found

    def conditions_at(self, x, y):
        """Return (conditions, zone name) at (x, y); name '' means the default."""
        found = self.zones_at(x, y)
        if not found:
            return self.default, ''
        worst = max(found, key=lambda zone: severity(zone.conditions))
        return worst.conditions, worst.name


def _parse_wkt_linestring(text):
    """Return [(lon, lat), ...] from 'LINESTRING (lon lat, lon lat, ...)'."""
    body = text.strip()
    if not body.upper().startswith('LINESTRING'):
        raise ValueError(f'not a WKT LINESTRING: {text[:40]!r}')
    body = body[len('LINESTRING'):].strip().lstrip('(').rstrip(')')
    points = []
    for pair in body.split(','):
        lon, lat = pair.split()[:2]
        points.append((float(lon), float(lat)))
    return points


def _edge_lonlats(graph, u, v, data):
    """Return the edge's shape as [(lon, lat), ...]."""
    geometry = data.get('geometry')
    if geometry is not None:
        if hasattr(geometry, 'coords'):
            return [(float(c[0]), float(c[1])) for c in geometry.coords]
        return _parse_wkt_linestring(str(geometry))
    nu, nv = graph.nodes[u], graph.nodes[v]
    return [(float(nu['x']), float(nu['y'])), (float(nv['x']), float(nv['y']))]


def tunnel_zones(graph, origin_lat, origin_lon, conditions=None,
                 half_width=DEFAULT_TUNNEL_HALF_WIDTH):
    """Return one zone per tunnel edge of the graph (both directions once)."""
    if conditions is None:
        conditions = PROFILES['no_fix']
    zones = []
    seen = set()
    for u, v, data in graph.edges(data=True):
        if not is_tunnel(data):
            continue
        key = (min(str(u), str(v)), max(str(u), str(v)), str(data.get('osmid', '')))
        if key in seen:
            continue
        seen.add(key)
        points = tuple(latlon_to_enu(lat, lon, origin_lat, origin_lon)
                       for lon, lat in _edge_lonlats(graph, u, v, data))
        zones.append(GpsZone(TUNNEL_ZONE_NAME, conditions, points, half_width,
                             yields_to_surface=True))
    return zones


def surface_segments(graph, origin_lat, origin_lon):
    """Return the ENU segments of every non-tunnel edge (both directions once)."""
    segments = []
    seen = set()
    for u, v, data in graph.edges(data=True):
        if is_tunnel(data):
            continue
        key = (min(str(u), str(v)), max(str(u), str(v)), str(data.get('osmid', '')))
        if key in seen:
            continue
        seen.add(key)
        points = [latlon_to_enu(lat, lon, origin_lat, origin_lon)
                  for lon, lat in _edge_lonlats(graph, u, v, data)]
        segments.extend(zip(points, points[1:]))
    return segments


_TOP_KEYS = {'default', 'tunnels_from_graph', 'tunnel_half_width', 'tunnel',
             'tunnel_surface_check', 'zones'}
_ZONE_KEYS = {'name', 'conditions', 'points', 'half_width'}


def zone_map_from_dict(values, origin_lat, origin_lon, graph=None):
    """Build a GpsZoneMap from a YAML-style dictionary.

    Keys (all optional): default (conditions dict, default the 'perfect'
    profile), tunnels_from_graph (bool, default False; needs graph),
    tunnel_half_width (m), tunnel (conditions dict for tunnels, default
    'no_fix'), tunnel_surface_check (bool, default True: tunnels do not apply
    where a surface edge is nearer), zones (list of {name, conditions,
    points: [[lat, lon], ...], half_width}). Unknown keys raise ValueError.
    """
    values = dict(values or {})
    unknown = sorted(set(values) - _TOP_KEYS)
    if unknown:
        raise ValueError(f'unknown GPS zone map keys: {unknown}')
    default = conditions_from_dict(values.get('default', {}))
    zones = []
    surface = None
    if values.get('tunnels_from_graph', False):
        if graph is None:
            raise ValueError('tunnels_from_graph is true but no graph was given')
        tunnel = conditions_from_dict(values.get('tunnel', {'profile': 'no_fix'}))
        zones.extend(tunnel_zones(
            graph, origin_lat, origin_lon, tunnel,
            float(values.get('tunnel_half_width', DEFAULT_TUNNEL_HALF_WIDTH))))
        if values.get('tunnel_surface_check', True):
            surface = SegmentIndex(surface_segments(graph, origin_lat, origin_lon))
    for i, entry in enumerate(values.get('zones') or []):
        unknown = sorted(set(entry) - _ZONE_KEYS)
        if unknown:
            raise ValueError(f'zone {i}: unknown keys {unknown}')
        points = tuple(latlon_to_enu(float(lat), float(lon), origin_lat, origin_lon)
                       for lat, lon in entry.get('points', []))
        zones.append(GpsZone(str(entry.get('name', f'zone {i}')),
                             conditions_from_dict(entry.get('conditions', {})),
                             points, float(entry.get('half_width', 0.0))))
    return GpsZoneMap(default, zones, surface)
