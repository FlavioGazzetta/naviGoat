"""Route planning on an OpenStreetMap walking graph.

Works on a networkx MultiDiGraph shaped like an osmnx graph: node attributes
x (longitude), y (latitude) and OSM tags; edge attributes = the way's OSM tags.
Needs neither osmnx nor ROS.

The graph must be unsimplified: every edge is a straight line between its two
nodes, so an edge's length is the straight-line distance in the local
east-north (ENU) frame around the origin.
"""

from dataclasses import dataclass, field
import math

from guide_core.crossings import RouteCrossing
from guide_core.geo import latlon_to_enu
from guide_core.polyline import simplify_polyline
from guide_core.route import MIN_SEGMENT_LENGTH, Route
from guide_core.walk_costs import (
    classify_crossing, CostConfig, crossed_road_at_junction, crossed_road_leaving_junction,
    CrossingType, edge_cost, has_sidewalk, is_island_tagged, is_road, is_tunnel, walkway_class,
    WalkwayClass,
)
import networkx as nx


class PlanningError(ValueError):
    """No route can be planned. The message says why, in plain words."""


def _value(tags, key):
    """Return tags[key] as a stripped lower-case string, or None."""
    value = tags.get(key) if tags else None
    if value is None:
        return None
    text = str(value).strip().lower()
    return text or None


def _is_finite_number(value):
    """Return True for a finite int/float that is not a bool."""
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


@dataclass(frozen=True)
class PlannerConfig:
    """Planner settings.

    walkway_snap_distance: a start/goal snaps to the nearest walkway node
    (footway, pedestrian street or path; not a crossing node or island) within
    this distance (m). Destinations looked up by name are often a building's
    centre, tens of metres from the pavement, so this is generous.
    max_snap_distance: with no walkway that close, the start/goal may snap to
    any surface node (e.g. a road centre line) within this distance (m); the
    plan then carries a warning. goal_snap_slack / goal_snap_weight: the goal
    is the walkway node, at most goal_snap_slack (m) further than the nearest
    one, with the lowest route cost + goal_snap_weight x snap distance (m).
    So a node a little further away beats a nearer one that is a long way
    round, but the goal never moves across a road to save a crossing (30 per
    unguided metre outweighs any crossing penalty over a road's width).
    simplify_tolerance: Douglas-Peucker
    tolerance (m).
    crossing_half_width: assumed half-width (m) of a crossing known only from
    its crossing node (no mapped crossing way). junction_half_width /
    side_street_half_width: assumed half-width (m) of a major / minor road
    crossed by walking along a road through a junction (the junction node is
    on the crossed road's centre line). island_max_length: a piece of
    walkway reachable only over crossings counts as a pedestrian island if its
    walkways add up to at most this (m). detour_warning_ratio /
    detour_warning_extra: a plan whose route is more than ratio x the
    straight-line distance AND more than extra (m) longer carries a warning.
    costs: edge cost weights.
    """

    walkway_snap_distance: float = 60.0
    goal_snap_slack: float = 15.0
    goal_snap_weight: float = 30.0
    max_snap_distance: float = 30.0
    simplify_tolerance: float = 0.5
    crossing_half_width: float = 3.0
    junction_half_width: float = 10.0
    side_street_half_width: float = 5.0
    island_max_length: float = 30.0
    detour_warning_ratio: float = 3.0
    detour_warning_extra: float = 300.0
    costs: CostConfig = field(default_factory=CostConfig)

    def __post_init__(self):
        """Validate all values."""
        if not _is_finite_number(self.goal_snap_slack) or self.goal_snap_slack < 0.0:
            raise ValueError(f'goal_snap_slack must be finite and >= 0, '
                             f'got {self.goal_snap_slack!r}')
        if not _is_finite_number(self.goal_snap_weight) or self.goal_snap_weight < 0.0:
            raise ValueError(f'goal_snap_weight must be finite and >= 0, '
                             f'got {self.goal_snap_weight!r}')
        if not _is_finite_number(self.walkway_snap_distance) or self.walkway_snap_distance <= 0.0:
            raise ValueError(f'walkway_snap_distance must be finite and > 0, '
                             f'got {self.walkway_snap_distance!r}')
        if not _is_finite_number(self.detour_warning_ratio) or self.detour_warning_ratio < 1.0:
            raise ValueError(f'detour_warning_ratio must be finite and >= 1, '
                             f'got {self.detour_warning_ratio!r}')
        if not _is_finite_number(self.detour_warning_extra) or self.detour_warning_extra < 0.0:
            raise ValueError(f'detour_warning_extra must be finite and >= 0, '
                             f'got {self.detour_warning_extra!r}')
        if not _is_finite_number(self.max_snap_distance) or self.max_snap_distance <= 0.0:
            raise ValueError(f'max_snap_distance must be finite and > 0, '
                             f'got {self.max_snap_distance!r}')
        if not _is_finite_number(self.simplify_tolerance) or self.simplify_tolerance < 0.0:
            raise ValueError(f'simplify_tolerance must be finite and >= 0, '
                             f'got {self.simplify_tolerance!r}')
        if not _is_finite_number(self.crossing_half_width) or self.crossing_half_width <= 0.0:
            raise ValueError(f'crossing_half_width must be finite and > 0, '
                             f'got {self.crossing_half_width!r}')
        if not _is_finite_number(self.side_street_half_width) \
                or self.side_street_half_width <= 0.0:
            raise ValueError(f'side_street_half_width must be finite and > 0, '
                             f'got {self.side_street_half_width!r}')
        if not _is_finite_number(self.junction_half_width) or self.junction_half_width <= 0.0:
            raise ValueError(f'junction_half_width must be finite and > 0, '
                             f'got {self.junction_half_width!r}')
        if not _is_finite_number(self.island_max_length) or self.island_max_length < 0.0:
            raise ValueError(f'island_max_length must be finite and >= 0, '
                             f'got {self.island_max_length!r}')
        if not isinstance(self.costs, CostConfig):
            raise ValueError(f'costs must be a CostConfig, got {self.costs!r}')


@dataclass(frozen=True)
class PlannedRoute:
    """The result of a successful plan.

    route: the simplified route in the map (ENU) frame.
    latlons: (lat, lon) of each route point, in the same order.
    crossings: RouteCrossing tuple, in route order.
    warnings: plain-language reasons to doubt the route (snapped to a road,
    a long detour); empty for a normal plan.
    """

    route: Route
    latlons: tuple
    start_snap_distance: float
    goal_snap_distance: float
    crossings: tuple
    warnings: tuple = ()

    @property
    def length(self):
        """Route length in metres."""
        return self.route.total_length


class RoutePlanner:
    """Plans walking routes on one graph. Create once; call plan() many times."""

    def __init__(self, graph, origin_lat, origin_lon, config=None):
        """Precompute node positions, edge costs and usable nodes.

        Raises ValueError if a node lacks numeric coordinates or nothing is usable.
        """
        self._config = PlannerConfig() if config is None else config
        if not isinstance(self._config, PlannerConfig):
            raise ValueError(f'config must be a PlannerConfig, got {config!r}')
        self._graph = graph
        self._origin = (float(origin_lat), float(origin_lon))

        # Every node's position in metres east/north of the origin.
        self._xy = {}
        for node, data in graph.nodes(data=True):
            try:
                lon, lat = float(data['x']), float(data['y'])
            except (KeyError, TypeError, ValueError):
                raise ValueError(f'graph node {node!r} has no numeric x (lon) / y (lat)') from None
            if not (math.isfinite(lat) and math.isfinite(lon)):
                raise ValueError(f'graph node {node!r} has non-finite coordinates')
            self._xy[node] = latlon_to_enu(lat, lon, *self._origin)

        # Pedestrian island nodes: both ends of every footway=traffic_island
        # edge, nodes tagged as islands (crossing:island=yes,
        # traffic_calming=island), and small pieces of walkway reachable only
        # over crossings (see _find_untagged_islands). Used to charge each
        # island once and to mark crossings that touch one (has_island).
        self._island_nodes = {node for node, data in graph.nodes(data=True)
                              if is_island_tagged(data)}
        for u, v, data in graph.edges(data=True):
            if _value(data, 'footway') == 'traffic_island':
                self._island_nodes.update((u, v))
        self._island_nodes |= self._find_untagged_islands(graph)

        # Cost of each directed node pair; for parallel edges keep the cheapest
        # usable one (and its tags). Forbidden edges get no entry at all. An
        # edge from a non-island node onto an island node pays the island
        # penalty, so walking along an island (island to island) never does
        # and each island is charged once per visit.
        self._cost = {}
        self._edge_tags = {}
        for u, v, data in graph.edges(data=True):
            length = math.dist(self._xy[u], self._xy[v])
            onto_island = v in self._island_nodes and u not in self._island_nodes
            cost = edge_cost(length, data, graph.nodes[u], graph.nodes[v], self._config.costs,
                             onto_island=onto_island)
            if cost is None:
                continue
            if (u, v) not in self._cost or cost < self._cost[(u, v)]:
                self._cost[(u, v)] = cost
                self._edge_tags[(u, v)] = data

        # Snap targets: nodes touching at least one usable edge. The start
        # (the robot's GPS position) is always on the surface; a goal may be
        # underground (a station reached through a pedestrian subway).
        usable = {u for u, _ in self._cost} | {v for _, v in self._cost}
        underground = set()
        for u, v, data in graph.edges(data=True):
            if is_tunnel(data):
                underground.update((u, v))
        all_usable = [(node, self._xy[node]) for node in graph.nodes if node in usable]
        self._usable = [(node, xy) for node, xy in all_usable if node not in underground]
        if not self._usable:
            raise ValueError('graph has no usable surface walkways')
        # Preferred snap targets: surface nodes on a walkway (footway,
        # pedestrian street, path, or a street whose pavement is tagged on
        # it), never in the middle of a road (crossing nodes) or on an
        # island. Other roads are only the fallback.
        walkway = set()
        for (u, v), data in self._edge_tags.items():
            on_foot = (walkway_class(data.get('highway')) is WalkwayClass.WALKWAY
                       or has_sidewalk(data))  # a street with its pavement tagged on it
            if on_foot and _value(data, 'footway') != 'crossing':
                walkway.update((u, v))
        # Goal candidates may be underground; start candidates may not.
        self._goal_walkway_nodes = [
            (node, xy) for node, xy in all_usable
            if node in walkway and node not in self._island_nodes
            and _value(graph.nodes[node], 'highway') != 'crossing']
        self._walkway_nodes = [(node, xy) for node, xy in self._goal_walkway_nodes
                               if node not in underground]

    @property
    def config(self):
        """Return the PlannerConfig in use."""
        return self._config

    @property
    def usable_node_count(self):
        """Return the number of nodes a start or goal can snap to (surface nodes only)."""
        return len(self._usable)

    @property
    def island_node_count(self):
        """Return the number of pedestrian island nodes found in the graph."""
        return len(self._island_nodes)

    def plan(self, start_lat, start_lon, goal_lat, goal_lon):
        """Plan a route between two lat/lon points. Returns a PlannedRoute.

        Raises PlanningError (with a readable message) if no route is possible.
        """
        for value in (start_lat, start_lon, goal_lat, goal_lon):
            if not _is_finite_number(value):
                raise PlanningError(f'coordinates must be finite numbers, got {value!r}')
        warnings = []
        no_route = PlanningError('No walkable route between start and goal '
                                 '(steps and indoor corridors are not allowed).')
        start_node, start_dist = self._snap(start_lat, start_lon, 'start', warnings)

        # Goal: the best walkway node near the goal point (route cost + a
        # charge per metre of snapping), found with one Dijkstra run from the
        # start; with no walkway near, the nearest node of any kind.
        candidates = self._near_walkway_nodes(goal_lat, goal_lon)
        if candidates:
            costs, paths = nx.single_source_dijkstra(self._graph, start_node,
                                                     weight=self._weight)
            weight = self._config.goal_snap_weight
            reachable = [(costs[node] + weight * dist, dist, node)
                         for node, dist in candidates if node in costs]
            if not reachable:
                raise no_route
            _, goal_dist, goal_node = min(reachable, key=lambda c: (c[0], c[1]))
            path = paths[goal_node]
        else:
            goal_node, goal_dist = self._snap(goal_lat, goal_lon, 'goal', warnings)
            path = None
        if start_node == goal_node:
            raise PlanningError('Start and goal are at the same point on the walking graph '
                                '(already at the destination).')
        if path is None:
            try:
                path = nx.dijkstra_path(self._graph, start_node, goal_node, weight=self._weight)
            except nx.NetworkXNoPath:
                raise no_route from None

        crossing_spans = self._find_crossings(path)

        # Simplify, always keeping the kerb points of mapped crossing ways, and
        # the node itself for crossings known only from a node.
        xy = [self._xy[node] for node in path]
        keep = set()
        for first, last, centre, *_ in crossing_spans:
            keep.update((first, last) if centre is None else (centre,))
        kept = simplify_polyline(xy, self._config.simplify_tolerance, keep=keep)

        # Build the point and lat/lon lists, merging practically identical
        # points exactly as Route would, so both lists stay the same length.
        # position[path index] = index of the matching route point.
        points, latlons, position = [], [], {}
        for i in kept:
            if points and math.dist(points[-1], xy[i]) < MIN_SEGMENT_LENGTH:
                position[i] = len(points) - 1
                continue
            data = self._graph.nodes[path[i]]
            points.append(xy[i])
            latlons.append((float(data['y']), float(data['x'])))
            position[i] = len(points) - 1
        try:
            route = Route(points)
        except ValueError as e:
            raise PlanningError(f'Planned path is not a valid route: {e}') from e

        # Distance along the route of every route point.
        along = [0.0]
        for a, b in zip(route.points, route.points[1:]):
            along.append(along[-1] + math.dist(a, b))
        total = along[-1]

        crossings = []
        for first, last, centre, crossing_type, crossed_road, tactile, island, junction \
                in crossing_spans:
            if centre is None:  # mapped crossing way: kerb to kerb
                start_s, end_s = along[position[first]], along[position[last]]
            else:  # crossing / junction node only: assume half-width either side of it
                half = {'major': self._config.junction_half_width,
                        'minor': self._config.side_street_half_width,
                        }.get(junction, self._config.crossing_half_width)
                s_node = along[position[centre]]
                start_s, end_s = max(0.0, s_node - half), min(total, s_node + half)
            crossings.append(RouteCrossing(start_s=start_s, end_s=end_s,
                                           crossing_type=crossing_type,
                                           crossed_road=crossed_road, tactile_paving=tactile,
                                           has_island=island))

        # No GPS underground: say how much of the route is in tunnels.
        tunnel_m = sum(math.dist(self._xy[u], self._xy[v]) for u, v in zip(path, path[1:])
                       if is_tunnel(self._edge_tags[(u, v)]))
        if tunnel_m > 0.0:
            warnings.append(f'route uses {tunnel_m:.0f} m of tunnel (no GPS there)')

        # A route much longer than the straight line usually means a badly
        # placed start/goal (e.g. a building's centre) or a gap in the map.
        straight = math.dist(latlon_to_enu(start_lat, start_lon, *self._origin),
                             latlon_to_enu(goal_lat, goal_lon, *self._origin))
        cfg = self._config
        if route.total_length > cfg.detour_warning_ratio * straight \
                and route.total_length - straight > cfg.detour_warning_extra:
            warnings.append(f'route is {route.total_length / max(straight, 1e-9):.1f}x the '
                            f'straight-line distance ({route.total_length:.0f} m vs '
                            f'{straight:.0f} m)')

        return PlannedRoute(route=route, latlons=tuple(latlons),
                            start_snap_distance=start_dist, goal_snap_distance=goal_dist,
                            crossings=tuple(crossings), warnings=tuple(warnings))

    def _find_untagged_islands(self, graph):
        """Return the nodes of pedestrian islands found from the graph's shape.

        Many islands are mapped as two crossing ways joined by a short plain
        footway, with no island tag at all (each half even says
        crossing:island=no: no island *within* that half). Such an island is
        literally an island in the walking graph: a small piece of walkway
        that can only be left over crossings. Found as connected pieces of
        walkway (footway/pedestrian/path, not crossing ways) whose lengths add
        up to at most island_max_length, with no crossing node inside, left
        only by crossing ways, at least two of them. A single node joining two
        crossing ways counts too (length 0).
        """
        limit = self._config.island_max_length
        # Undirected adjacency: node -> {neighbour: kind}, kind = 'crossing'
        # (footway=crossing), 'walk' (other walkway) or 'other' (roads, steps...).
        adjacency = {}
        for u, v, data in graph.edges(data=True):
            if _value(data, 'footway') == 'crossing':
                kind = 'crossing'
            elif walkway_class(data.get('highway')) is WalkwayClass.WALKWAY:
                kind = 'walk'
            else:
                kind = 'other'
            for a, b in ((u, v), (v, u)):
                # If parallel edges disagree, the most restrictive kind wins.
                previous = adjacency.setdefault(a, {}).get(b)
                if previous is None or (previous, kind) in (('walk', 'crossing'),
                                                            ('walk', 'other'),
                                                            ('crossing', 'other')):
                    adjacency[a][b] = kind

        def is_crossing_node(node):
            return _value(graph.nodes[node], 'highway') == 'crossing'

        islands, checked = set(), set()
        for start, neighbours in adjacency.items():
            if start in checked or is_crossing_node(start) \
                    or 'crossing' not in neighbours.values():
                continue
            # Collect the walkway piece around start, giving up once too long.
            piece, stack, length, too_long = {start}, [start], 0.0, False
            while stack and not too_long:
                node = stack.pop()
                for other, kind in adjacency[node].items():
                    if kind != 'walk' or other in piece:
                        continue
                    piece.add(other)
                    stack.append(other)
                    length += math.dist(self._xy[node], self._xy[other])
                    if length > limit:
                        too_long = True
                        break
            checked |= piece
            if too_long or any(is_crossing_node(n) for n in piece):
                continue
            exits = [(other, kind) for n in piece for other, kind in adjacency[n].items()
                     if other not in piece]
            if exits and all(kind == 'crossing' for _, kind in exits) and len(exits) >= 2:
                islands |= piece
        return islands

    def _weight(self, u, v, _data):
        """Dijkstra edge weight; None hides the edge (forbidden)."""
        return self._cost.get((u, v))

    def _near_walkway_nodes(self, lat, lon):
        """Return [(node, distance m)]: walkway nodes the goal may snap to.

        Within walkway_snap_distance of the point, and at most goal_snap_slack
        further than the nearest walkway node.
        """
        x, y = latlon_to_enu(lat, lon, *self._origin)
        cfg = self._config
        near = []
        for node, (node_x, node_y) in self._goal_walkway_nodes:
            d = math.hypot(node_x - x, node_y - y)
            if d <= cfg.walkway_snap_distance:
                near.append((node, d))
        if not near:
            return []
        nearest = min(d for _, d in near)
        return [(node, d) for node, d in near if d <= nearest + cfg.goal_snap_slack]

    def _snap(self, lat, lon, what, warnings):
        """Return (node, distance in m) for a start/goal point.

        The nearest walkway node within walkway_snap_distance; else the
        nearest surface node of any kind within max_snap_distance, adding a
        warning to `warnings`; else PlanningError.
        """
        x, y = latlon_to_enu(lat, lon, *self._origin)

        def nearest(candidates):
            best_node, best_dist = None, math.inf
            for node, (node_x, node_y) in candidates:
                d = math.hypot(node_x - x, node_y - y)
                if d < best_dist:
                    best_node, best_dist = node, d
            return best_node, best_dist

        cfg = self._config
        walk_node, walk_dist = nearest(self._walkway_nodes)
        if walk_dist <= cfg.walkway_snap_distance:
            return walk_node, walk_dist
        any_node, any_dist = nearest(self._usable)
        if any_dist <= cfg.max_snap_distance:
            warnings.append(f'{what} is {walk_dist:.1f} m from the nearest walkway; snapped '
                            f'{any_dist:.1f} m to a road')
            return any_node, any_dist
        raise PlanningError(f'The {what} is {walk_dist:.1f} m from the nearest walkway '
                            f'(limit {cfg.walkway_snap_distance:.1f} m).')

    def _find_crossings(self, path):
        """Return (first, last, centre, type, road, tactile, island, junction) per crossing.

        A crossing is either a run of consecutive footway=crossing edges, split
        at island nodes (first/last = path indices of the near and far kerb,
        centre = None),
        or a crossing node reached from a walkway outside any such run
        (centre = its path index; first/last = its neighbours, used only to
        collect tags), or a road junction reached along a road, or left along
        a minor road where a major road meets (junction = the class of road
        crossed, 'major' or 'minor', see crossed_road_at_junction and
        crossed_road_leaving_junction; 'major' wins;
        centre = its path index; type unknown: there is no crossing facility;
        junction is None for the other kinds). island: a node of the
        span is a pedestrian island node, or a crossing node or edge of the
        span is tagged as an island.
        """
        graph = self._graph
        n_edges = len(path) - 1
        is_crossing_edge = [
            _value(self._edge_tags[(path[i], path[i + 1])], 'footway') == 'crossing'
            for i in range(n_edges)
        ]

        spans = []
        covered = set()
        i = 0
        while i < n_edges:
            if is_crossing_edge[i]:
                j = i
                # A run of crossing edges is split at a pedestrian island node
                # (two halves mapped as one way, or joined at one node): each
                # half is its own crossing, so the robot stops on the island.
                while j + 1 < n_edges and is_crossing_edge[j + 1] \
                        and path[j + 1] not in self._island_nodes:
                    j += 1
                spans.append((i, j + 1, None))  # edges i..j join path nodes i..j+1
                covered.update(range(i, j + 2))
                i = j + 1
            else:
                i += 1
        for k in range(len(path)):
            node = graph.nodes[path[k]]
            entering = (crossed_road_at_junction(self._edge_tags[(path[k - 1], path[k])], node)
                        if k > 0 else None)
            leaving = (crossed_road_leaving_junction(
                self._edge_tags[(path[k], path[k + 1])], node) if k + 1 < len(path) else None)
            crossed = 'major' if 'major' in (entering, leaving) else entering
            if crossed is not None:
                spans.append((max(k - 1, 0), min(k + 1, len(path) - 1), k, crossed))
                continue
            if k == 0:
                continue
            edge = self._edge_tags[(path[k - 1], path[k])]
            if k in covered or _value(graph.nodes[path[k]], 'highway') != 'crossing':
                continue
            if is_road(edge.get('highway')):
                continue  # walking along a road through a crossing point
            spans.append((k - 1, min(k + 1, len(path) - 1), k, None))
        spans = [span if len(span) == 4 else span + (None,) for span in spans]
        spans.sort(key=lambda span: span[0])

        result = []
        for first, last, centre, junction in spans:
            if junction is not None:
                result.append((first, last, centre, CrossingType.UNKNOWN, junction, False,
                               False, junction))
                continue
            node_tags = [graph.nodes[path[k]] for k in range(first, last + 1)
                         if _value(graph.nodes[path[k]], 'highway') == 'crossing']
            edge_tags = [self._edge_tags[(path[k], path[k + 1])] for k in range(first, last)]
            crossing_type = classify_crossing(*node_tags, *edge_tags)
            roads = {_value(tags, 'crossed_road') for tags in node_tags}
            crossed_road = ('major' if 'major' in roads else 'minor' if 'minor' in roads
                            else 'none' if 'none' in roads else 'unknown')
            tactile = any(_value(tags, 'tactile_paving') == 'yes'
                          for tags in node_tags + edge_tags)
            island = (any(path[k] in self._island_nodes for k in range(first, last + 1))
                      or any(is_island_tagged(tags) for tags in edge_tags))
            result.append((first, last, centre, crossing_type, crossed_road, tactile, island,
                           None))
        return result
