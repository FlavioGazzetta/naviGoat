"""Walkway and crossing classification, and edge costs for route planning.

Works on plain dictionaries of OpenStreetMap tags, as stored on the nodes and
edges of an osmnx graph, so it needs neither osmnx nor networkx.

Cost model (all values in metres):

    cost = length * road factor + crossing penalty

- Steps and indoor corridors are forbidden: the robot cannot use them.
- Walking along a road centreline is strongly discouraged (minor roads x10,
  major x50) but never forbidden: some places can only be reached that way.
  The robot's goals would then lie on the road's centre line.
- Tunnels (tunnel=yes) cost tunnel_factor (x3) on top: there is no GPS
  underground, so a tunnel is used only when it clearly saves walking or
  crossings. Short building passages (tunnel=building_passage) are not
  affected.
- Service roads (highway=service: campus and access roads, car-park lanes,
  driveways; few, slow cars) cost service_road_factor (x3) instead of the
  minor road factor.
- A road whose pavement is tagged on it (sidewalk=both/left/right/yes) is
  walked on that pavement: sidewalk_road_factor (x1.5) instead of the road
  factor. sidewalk=separate/no or no tag keeps the road factor.
- Walking along a road into a junction (node attributes junction_major_dirs /
  junction_minor_dirs, added by fetch_osm_graph) crosses the other road(s)
  meeting there with no crossing facility: charged as an unknown crossing of
  the most important other road (see crossed_road_at_junction), and the
  route planner reports it as a crossing. Turning off a major road onto a
  minor one also crosses the major road (the pavement may be on the far
  side): a minor-road edge leaving a junction where a major road meets is
  charged as a major crossing too (crossed_road_leaving_junction). Going
  straight across along a minor road is therefore charged twice.
- A crossing is charged once, on the walkway edge that steps onto the
  crossing node, with a penalty that depends on the crossing type and on the
  class of road crossed (node attribute 'crossed_road', added by
  fetch_osm_graph; missing means unknown and is treated as major).
- A pedestrian island (two-stage crossing) costs island_penalty extra,
  charged once per island: on the edge that steps onto an island node (the
  route planner decides which nodes are island nodes and passes
  onto_island=True), or on a crossing way tagged crossing:island=yes where its
  crossing is charged. Islands are discouraged, not forbidden: the robot can
  only stop on a narrow island as precisely as its localisation allows.
"""

from dataclasses import dataclass, fields
import enum
import math

FORBIDDEN_TYPES = frozenset({'steps', 'corridor'})
WALKWAY_TYPES = frozenset({'footway', 'pedestrian', 'path'})
MAJOR_ROADS = frozenset({
    'trunk', 'trunk_link', 'primary', 'primary_link',
    'secondary', 'secondary_link', 'tertiary', 'tertiary_link',
})
MINOR_ROADS = frozenset({'residential', 'unclassified', 'service', 'living_street', 'road'})

# UK crossing_ref values for crossings with traffic lights.
SIGNALLED_CROSSING_REFS = frozenset({'pelican', 'puffin', 'toucan', 'pegasus'})
# crossing=* values that mean "no traffic lights".
UNSIGNALLED_CROSSING_VALUES = frozenset(
    {'unmarked', 'uncontrolled', 'marked', 'zebra', 'informal'})


class WalkwayClass(enum.Enum):
    """How an edge may be used, from its highway=* tag."""

    FORBIDDEN = 'forbidden'
    WALKWAY = 'walkway'
    MINOR_ROAD = 'minor_road'
    MAJOR_ROAD = 'major_road'


class CrossingType(enum.Enum):
    """What controls a crossing."""

    SIGNALLED = 'signalled'
    UNSIGNALLED = 'unsignalled'
    UNKNOWN = 'unknown'


def _norm(value):
    """Return value as a stripped lower-case string, or None if missing/empty."""
    if value is None:
        return None
    text = str(value).strip().lower()
    return text or None


def _tag(tags, key):
    """Return the normalised value of tags[key], or None (tags may be None)."""
    if not tags:
        return None
    return _norm(tags.get(key))


def walkway_class(highway):
    """Classify a highway=* value.

    Unrecognised or missing values (e.g. bridleway) count as minor roads:
    usable, but discouraged.
    """
    value = _norm(highway)
    if value in FORBIDDEN_TYPES:
        return WalkwayClass.FORBIDDEN
    if value in WALKWAY_TYPES:
        return WalkwayClass.WALKWAY
    if value in MAJOR_ROADS:
        return WalkwayClass.MAJOR_ROAD
    return WalkwayClass.MINOR_ROAD


def is_road(highway):
    """Return True if a highway=* value is a road that cars use."""
    value = _norm(highway)
    return value in MAJOR_ROADS or value in MINOR_ROADS


def is_tunnel(edge_tags):
    """Return True for an underground way (tunnel=yes; not a building passage)."""
    return _tag(edge_tags, 'tunnel') == 'yes'


SIDEWALK_VALUES = frozenset({'both', 'left', 'right', 'yes'})
# service=* values of ways too minor to count at junctions: crossing a
# driveway or a car-park lane is not a crossing stop.
MINOR_SERVICE_VALUES = frozenset({'driveway', 'parking_aisle', 'drive-through'})


def is_minor_service(edge_tags):
    """Return True for a driveway, car-park lane or drive-through (highway=service)."""
    return (_tag(edge_tags, 'highway') == 'service'
            and _tag(edge_tags, 'service') in MINOR_SERVICE_VALUES)


def has_sidewalk(edge_tags):
    """Return True if a road's pavement is tagged on the road itself.

    sidewalk=separate (pavement mapped as its own footway, which should be
    used instead) and sidewalk=no count as no pavement here.
    """
    return _tag(edge_tags, 'sidewalk') in SIDEWALK_VALUES


def _count(tags, key):
    """Return tags[key] as a non-negative int (GraphML stores it as text), else 0."""
    try:
        return max(0, int(float(_tag(tags, key) or 0)))
    except ValueError:
        return 0


def crossed_road_leaving_junction(edge_tags, from_tags):
    """Return 'major' if walking this minor-road edge leaves a junction with a major road.

    Covers turning off a major road into a side street: with the pavement
    only tagged on the road, the robot may be on the far side and must cross
    the major road. Returns None otherwise.
    """
    highway = _norm(_tag(edge_tags, 'highway'))
    if highway in MINOR_ROADS and _count(from_tags, 'junction_major_dirs') >= 1:
        return 'major'
    return None


def crossed_road_at_junction(edge_tags, to_tags):
    """Return the class of road crossed by walking this road edge into a junction.

    to_tags carries junction_major_dirs / junction_minor_dirs: how many road
    directions of each class meet at the node (the walked road counts too,
    usually 2). Returns 'major', 'minor' or None (no other road meets, or the
    edge is not a road). The most important other road wins:
    walking a minor road: a major road meets (any major direction) -> 'major';
    else another minor road (3+ minor directions) -> 'minor'.
    walking a major road: another major road (3+ major directions) -> 'major';
    else a minor road meets -> 'minor'.
    """
    highway = _norm(_tag(edge_tags, 'highway'))
    if not is_road(highway):
        return None
    major = _count(to_tags, 'junction_major_dirs')
    minor = _count(to_tags, 'junction_minor_dirs')
    if highway in MAJOR_ROADS:
        if major >= 3:
            return 'major'
        return 'minor' if minor >= 1 else None
    if major >= 1:
        return 'major'
    return 'minor' if minor >= 3 else None


def classify_crossing(*tag_dicts):
    """Classify a crossing from one or more tag dictionaries (node, way).

    Evidence from all dictionaries is combined. If the tags contradict each
    other, the result is UNSIGNALLED: the safer assumption for a blind user.
    None or empty dictionaries are ignored.
    """
    signalled = False
    unsignalled = False
    for tags in tag_dicts:
        crossing = _tag(tags, 'crossing')
        signals = _tag(tags, 'crossing:signals')
        ref = _tag(tags, 'crossing_ref')
        if crossing == 'traffic_signals' or signals == 'yes' or ref in SIGNALLED_CROSSING_REFS:
            signalled = True
        if crossing in UNSIGNALLED_CROSSING_VALUES or signals == 'no' or ref == 'zebra':
            unsignalled = True
    if unsignalled:
        return CrossingType.UNSIGNALLED
    if signalled:
        return CrossingType.SIGNALLED
    return CrossingType.UNKNOWN


@dataclass(frozen=True)
class CostConfig:
    """Tunable weights of the cost model, in metres of extra walking.

    Road factors must be >= 1 so a road is never cheaper than a walkway of
    the same length (and the straight-line distance stays a lower bound on
    cost). Penalties must be finite and >= 0, as Dijkstra requires.
    """

    minor_road_factor: float = 10.0
    major_road_factor: float = 50.0
    signalled_minor: float = 10.0
    unsignalled_minor: float = 30.0
    unknown_minor: float = 40.0
    signalled_major: float = 20.0
    unsignalled_major: float = 300.0
    unknown_major: float = 300.0
    island_penalty: float = 50.0
    tunnel_factor: float = 3.0
    sidewalk_road_factor: float = 1.5
    service_road_factor: float = 3.0

    def __post_init__(self):
        """Validate all values."""
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value):
                raise ValueError(f'{f.name} must be a finite number, got {value!r}')
            minimum = 1.0 if f.name.endswith('_factor') else 0.0
            if value < minimum:
                raise ValueError(f'{f.name} must be >= {minimum}, got {value!r}')


def crossing_penalty(crossing_type, crossed_road, config):
    """Return the penalty for one crossing.

    crossed_road: 'major' or 'minor' (class of road crossed), or 'none' (we
    looked and no public road passes through it, e.g. a cycle track): 'none'
    uses the minor penalties. Missing or anything else (no information)
    counts as major, the cautious assumption.
    """
    minor = _norm(crossed_road) in ('minor', 'none')
    if crossing_type is CrossingType.SIGNALLED:
        return config.signalled_minor if minor else config.signalled_major
    if crossing_type is CrossingType.UNSIGNALLED:
        return config.unsignalled_minor if minor else config.unsignalled_major
    return config.unknown_minor if minor else config.unknown_major


def is_island_tagged(tags):
    """Return True if node or way tags mark a pedestrian island.

    crossing:island=yes (on a crossing node or way) or traffic_calming=island
    (on a node). footway=traffic_island ways are found by the route planner,
    which looks at the graph around a node.
    """
    return _tag(tags, 'crossing:island') == 'yes' or _tag(tags, 'traffic_calming') == 'island'


def edge_cost(length, edge_tags, from_tags, to_tags, config=None, onto_island=False):
    """Return the cost of walking one directed edge, or None if forbidden.

    length: edge length in metres. edge_tags: the edge's (way's) tags.
    from_tags / to_tags: tags of the start / end node (may be None).
    onto_island: True if this edge steps from outside a pedestrian island onto
    it (decided by the caller). Adds island_penalty once; never on roads.
    """
    if config is None:
        config = CostConfig()
    length = float(length)
    if not (math.isfinite(length) and length >= 0.0):
        raise ValueError(f'length must be finite and >= 0, got {length!r}')

    highway = _tag(edge_tags, 'highway')
    walk_class = walkway_class(highway)
    if walk_class is WalkwayClass.FORBIDDEN:
        return None
    factor = {
        WalkwayClass.WALKWAY: 1.0,
        WalkwayClass.MINOR_ROAD: config.minor_road_factor,
        WalkwayClass.MAJOR_ROAD: config.major_road_factor,
    }[walk_class]
    if walk_class is not WalkwayClass.WALKWAY:
        if has_sidewalk(edge_tags):
            factor = config.sidewalk_road_factor
        elif highway == 'service':
            factor = config.service_road_factor
    if is_tunnel(edge_tags):
        factor *= config.tunnel_factor
    cost = length * factor

    # Walking along a road through a crossing point is not crossing that road,
    # but walking it into a junction crosses the other road(s) meeting there.
    if is_road(highway):
        for crossed in (crossed_road_at_junction(edge_tags, to_tags),
                        crossed_road_leaving_junction(edge_tags, from_tags)):
            if crossed is not None:
                cost += crossing_penalty(CrossingType.UNKNOWN, crossed, config)
        return cost
    island = onto_island
    # Stepping onto a crossing node: charge the crossing once, here.
    if _tag(to_tags, 'highway') == 'crossing':
        crossing_type = classify_crossing(to_tags, edge_tags)
        cost += crossing_penalty(crossing_type, _tag(to_tags, 'crossed_road'), config)
        island = island or is_island_tagged(edge_tags)
    # Fallback: a crossing way with no crossing node at either end.
    elif _tag(edge_tags, 'footway') == 'crossing' and _tag(from_tags, 'highway') != 'crossing':
        cost += crossing_penalty(classify_crossing(edge_tags), None, config)
        island = island or is_island_tagged(edge_tags)
    if island:
        cost += config.island_penalty
    return cost
