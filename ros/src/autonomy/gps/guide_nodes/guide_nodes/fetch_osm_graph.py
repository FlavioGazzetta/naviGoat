#!/usr/bin/env python3
"""Download the OpenStreetMap walking graph around the origin and save it to disk.

Run once, with internet. The route planner node loads the saved file at
startup, so the robot never needs internet to plan routes. This is not a
ROS node: it uses osmnx only, no rclpy.

Map data (c) OpenStreetMap contributors, available under the ODbL.
"""

import argparse
from collections import Counter
import datetime
import os
import time

# The graph's area, as the geocoder reads it back (same calculation).
from guide_core.geocoding import BBox
# Road types (major/minor) shared with the planner's cost model, so crossing
# labels and costs always agree.
from guide_core.walk_costs import is_minor_service, is_road, MAJOR_ROADS, MINOR_ROADS
import osmnx as ox
import yaml

# Same origin as the localizer: the centre of the local east-north (map) frame.
DEFAULT_ORIGIN_LAT = 51.4988
DEFAULT_ORIGIN_LON = -0.1749

# Extra OSM tags to keep. osmnx keeps only a short default list, and the
# planner's edge costs need crossing and pavement information.
# Crossings are tagged on nodes (where a footway meets the road) and/or on
# the short way across the road (footway=crossing), so we keep both.
EXTRA_NODE_TAGS = [
    'crossing', 'crossing:signals', 'crossing:markings', 'crossing_ref',
    'tactile_paving', 'crossing:island', 'traffic_calming',
]
EXTRA_WAY_TAGS = [
    'footway', 'crossing', 'crossing:signals', 'crossing:markings',
    'crossing_ref', 'tactile_paving', 'sidewalk', 'foot', 'crossing:island',
]


def parse_args(argv=None):
    """Read the command-line options."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--lat', type=float, default=DEFAULT_ORIGIN_LAT,
                        help='centre latitude (default: the map origin)')
    parser.add_argument('--lon', type=float, default=DEFAULT_ORIGIN_LON,
                        help='centre longitude (default: the map origin)')
    parser.add_argument('--dist', type=float, default=1500.0,
                        help='half-width of the square area in metres (default 1500)')
    parser.add_argument('--out', required=True,
                        help='output GraphML file, e.g. .../osm/south_ken_walk.graphml')
    parser.add_argument('--overpass-url', default=None,
                        help='other Overpass API server, e.g. https://overpass.private.coffee/api '
                             '(default: osmnx default, overpass-api.de)')
    return parser.parse_args(argv)


def summarise(graph):
    """Return printable lines describing the graph's size, crossings, islands and walkway types."""
    crossing_nodes = Counter()
    node_signals = Counter()
    node_refs = Counter()
    node_island = Counter()        # crossing:island=* values on nodes
    node_calming = Counter()       # traffic_calming=* values on nodes
    for _, data in graph.nodes(data=True):
        if data.get('highway') == 'crossing':
            crossing_nodes[str(data.get('crossing', '(no crossing= tag)'))] += 1
            if 'crossing:signals' in data:
                node_signals[str(data['crossing:signals'])] += 1
            if 'crossing_ref' in data:
                node_refs[str(data['crossing_ref'])] += 1
        if 'crossing:island' in data:
            node_island[str(data['crossing:island'])] += 1
        if 'traffic_calming' in data:
            node_calming[str(data['traffic_calming'])] += 1

    # Walking graphs are two-way: every walkway is stored as two directed
    # edges. Count each pair of nodes once so the numbers match the real map.
    seen = set()
    crossing_ways = Counter()
    walkway_types = Counter()
    island_pieces = 0
    for u, v, data in graph.edges(data=True):
        pair = (min(u, v), max(u, v))
        if pair in seen:
            continue
        seen.add(pair)
        walkway_types[str(data.get('highway', '(none)'))] += 1
        if data.get('footway') == 'crossing':
            crossing_ways[str(data.get('crossing', '(no crossing= tag)'))] += 1
        if data.get('footway') == 'traffic_island':
            island_pieces += 1

    return [
        f'Nodes: {graph.number_of_nodes()}, directed edges: {graph.number_of_edges()} '
        f'(= {len(seen)} walkway pieces, each stored in both directions)',
        f'Crossing nodes (highway=crossing), by crossing=: {dict(crossing_nodes.most_common())}',
        f'  ... with crossing:signals=: {dict(node_signals.most_common())}',
        f'  ... with crossing_ref=: {dict(node_refs.most_common())}',
        'Crossing way pieces (footway=crossing), by crossing=: '
        f'{dict(crossing_ways.most_common())}',
        f'Pedestrian island pieces (footway=traffic_island): {island_pieces}',
        f'Island-tagged nodes: crossing:island= {dict(node_island.most_common())}, '
        f'traffic_calming= {dict(node_calming.most_common())}',
        f'Walkway pieces by highway= (top 12): {dict(walkway_types.most_common(12))}',
    ]


def annotate_crossed_roads(walk, roads):
    """Label each crossing node with the class of road it crosses.

    A crossing node normally lies on the road's centreline. The walk graph
    drops roads whose pavements are mapped separately, so we also look the
    node up in a separately downloaded road graph. Sets the node attribute
    'crossed_road' to 'major' or 'minor'; leaves it unset if no road is found
    (the planner then assumes 'major'). Returns counts per outcome.
    """
    counts = Counter()
    for node, data in walk.nodes(data=True):
        if data.get('highway') != 'crossing':
            continue
        types = set()
        for graph in (walk, roads):
            if node in graph:
                # In- and out-edges: roads can be one-way in the road graph.
                for _, _, edge in list(graph.in_edges(node, data=True)) + \
                        list(graph.out_edges(node, data=True)):
                    types.add(str(edge.get('highway')))
        if types & MAJOR_ROADS:
            data['crossed_road'] = 'major'
        elif types & MINOR_ROADS:
            data['crossed_road'] = 'minor'
        else:
            data['crossed_road'] = 'none'  # no public road here (e.g. a cycle track)
        counts[data['crossed_road']] += 1
    return counts


def annotate_junctions(walk, roads):
    """Label walk-graph nodes where roads meet with the road directions of each class.

    Walking along a road through a junction crosses the other road(s) there,
    usually with no crossing facility; the planner charges and reports it
    (guide_core.walk_costs.crossed_road_at_junction). Directions are counted
    from both graphs: the road graph knows roads the walk graph dropped (e.g.
    Cromwell Road, whose pavements are mapped separately), the walk graph
    knows streets closed to cars (e.g. a shared street). Sets
    junction_major_dirs / junction_minor_dirs (as text, GraphML stores text)
    where at least 3 road directions meet; returns the number of junctions.
    """
    count = 0
    for node, data in walk.nodes(data=True):
        classes = {}  # neighbour node -> 'major' or 'minor' (major wins)
        for graph in (walk, roads):
            if node not in graph:
                continue
            for u, v, edge in list(graph.in_edges(node, data=True)) + \
                    list(graph.out_edges(node, data=True)):
                highway = edge.get('highway')
                # Driveways and car-park lanes don't make a junction a crossing.
                if not is_road(highway) or is_minor_service(edge):
                    continue
                other = v if u == node else u
                if classes.get(other) != 'major':
                    classes[other] = 'major' if str(highway) in MAJOR_ROADS else 'minor'
        if len(classes) >= 3:
            values = list(classes.values())
            data['junction_major_dirs'] = str(values.count('major'))
            data['junction_minor_dirs'] = str(values.count('minor'))
            count += 1
    return count


def download_with_retries(what, attempts=4, first_wait=30.0, **kwargs):
    """Call ox.graph_from_point(**kwargs), retrying with doubling waits on failure.

    The public Overpass server often answers 'busy' (HTTP 429/502/504) and
    asks clients to back off. Ctrl+C still stops immediately.
    """
    wait = first_wait
    for attempt in range(1, attempts + 1):
        try:
            return ox.graph_from_point(**kwargs)
        except Exception as e:  # network/server errors; re-raised after the last try
            if attempt == attempts:
                raise
            print(f'  {what}: attempt {attempt}/{attempts} failed '
                  f'({type(e).__name__}: {str(e).splitlines()[0]}); retrying in {wait:.0f} s ...')
            time.sleep(wait)
            wait *= 2


def graph_bbox(lat, lon, dist):
    """Return the area downloaded, a square of +-dist metres around (lat, lon), as a dict.

    Saved in the .meta.yaml so the geocoder limits online searches to it.
    """
    box = BBox.around(lat, lon, dist)
    return {'min_lat': round(box.min_lat, 7), 'min_lon': round(box.min_lon, 7),
            'max_lat': round(box.max_lat, 7), 'max_lon': round(box.max_lon, 7)}


def main(argv=None):
    """Download, save, write metadata and print a summary."""
    args = parse_args(argv)
    out_path = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    # Tell osmnx which extra tags to keep (duplicates removed, order kept),
    # and keep its download cache out of whatever folder this is run from.
    ox.settings.useful_tags_node = list(dict.fromkeys(
        list(ox.settings.useful_tags_node) + EXTRA_NODE_TAGS))
    ox.settings.useful_tags_way = list(dict.fromkeys(
        list(ox.settings.useful_tags_way) + EXTRA_WAY_TAGS))
    ox.settings.cache_folder = os.path.expanduser('~/.cache/osmnx')

    if args.overpass_url:
        ox.settings.overpass_url = args.overpass_url

    # network_type='walk': everything pedestrians may use, made two-way.
    # simplify=False: keep every OSM node, so tagged crossing nodes survive
    # and every edge is a straight line between two nodes.
    # retain_all=False: keep only the largest connected piece, so a start or
    # goal can never snap onto an unreachable island.
    print(f'Downloading walking graph: {2 * args.dist:.0f} m square around '
          f'({args.lat}, {args.lon}). This can take a minute or two ...')
    graph = download_with_retries('walking graph', center_point=(args.lat, args.lon),
                                  dist=args.dist, network_type='walk',
                                  simplify=False, retain_all=False)

    # The road network (including roads the walk graph dropped) is only used
    # to label crossings; it is not saved. 'drive_service' = all public roads
    # incl. service roads; retain_all=True keeps every piece.
    print('Downloading road network to label crossings ...')
    roads = download_with_retries('road network', center_point=(args.lat, args.lon),
                                  dist=args.dist, network_type='drive_service',
                                  simplify=False, retain_all=True)
    crossed_counts = annotate_crossed_roads(graph, roads)
    junction_count = annotate_junctions(graph, roads)

    ox.save_graphml(graph, filepath=out_path)

    # A small YAML next to the graph records where it came from, so the
    # planner can check its origin matches and anyone can re-create it.
    meta_path = os.path.splitext(out_path)[0] + '.meta.yaml'
    meta = {
        'origin_lat': args.lat,
        'origin_lon': args.lon,
        'dist_m': args.dist,
        'bbox': graph_bbox(args.lat, args.lon, args.dist),
        'network_type': 'walk',
        'simplify': False,
        'downloaded_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(
            timespec='seconds'),
        'osmnx_version': ox.__version__,
        'nodes': graph.number_of_nodes(),
        'directed_edges': graph.number_of_edges(),
        'crossing_nodes_by_road_crossed': dict(crossed_counts),
        'road_junctions': junction_count,
        'attribution': '(c) OpenStreetMap contributors, ODbL',
    }
    with open(meta_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)

    size_mb = os.path.getsize(out_path) / 1e6
    print(f'Saved {out_path} ({size_mb:.1f} MB)')
    print(f'Saved {meta_path}')
    for line in summarise(graph):
        print(line)

    print(f'Crossing nodes by road crossed: {dict(crossed_counts.most_common())}')
    print(f'Road junctions labelled (junction_*_dirs): {junction_count}')


if __name__ == '__main__':
    main()
