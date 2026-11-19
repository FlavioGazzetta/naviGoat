"""Polyline simplification (Ramer-Douglas-Peucker) for planned routes.

Removes points that add nothing to a route's shape while guaranteeing that
every original point stays within `tolerance` of the simplified line.
"""

import math


def point_segment_distance(p, a, b):
    """Return the distance from point p to the line segment a-b (not the infinite line).

    All arguments are (x, y) pairs. If a == b, this is the distance from p to a.
    """
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_sq = dx * dx + dy * dy
    if seg_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    # Position of p's projection along a-b: 0 at a, 1 at b; clamped to the segment.
    t = ((px - ax) * dx + (py - ay) * dy) / seg_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def simplify_polyline(points, tolerance, keep=()):
    """Return the sorted indices of the points to keep.

    points: sequence of (x, y). tolerance: max allowed distance (metres) from
    any original point to the simplified line; finite and >= 0.
    keep: indices that must be kept (e.g. crossing kerb points). The first
    and last points are always kept. Points exactly within tolerance are dropped.

    Raises ValueError for an invalid tolerance or keep index.
    """
    n = len(points)
    tolerance = float(tolerance)
    if not (math.isfinite(tolerance) and tolerance >= 0.0):
        raise ValueError(f'tolerance must be finite and >= 0, got {tolerance!r}')
    protected = set()
    for i in keep:
        if isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < n:
            raise ValueError(f'keep index {i!r} is not an int in [0, {n - 1}]')
        protected.add(i)
    if n <= 2:
        return list(range(n))

    # Protected points split the line into pieces; each piece is simplified
    # on its own, so protected points are never dropped.
    anchors = sorted(protected | {0, n - 1})
    kept = set(anchors)
    for start, end in zip(anchors, anchors[1:]):
        # A stack instead of recursion: long routes can't hit Python's recursion limit.
        stack = [(start, end)]
        while stack:
            s, e = stack.pop()
            if e - s < 2:
                continue
            worst_i, worst_d = s + 1, -1.0
            for i in range(s + 1, e):
                d = point_segment_distance(points[i], points[s], points[e])
                if d > worst_d:
                    worst_i, worst_d = i, d
            if worst_d > tolerance:
                kept.add(worst_i)
                stack.append((s, worst_i))
                stack.append((worst_i, e))
    return sorted(kept)


def resample_polyline(points, spacing):
    """Return points every `spacing` metres along the line, as (x, y, heading).

    points: sequence of (x, y). spacing: metres between the returned points,
    measured along the line; finite and > 0. The first and the last point are
    always returned, so the last step can be shorter than spacing. heading is
    the direction of travel at the point, in radians (0 = +x, counter-clockwise);
    the last point keeps the heading of the stretch that arrives at it.
    Repeated points are skipped. Fewer than two different points give [].

    Raises ValueError for an invalid spacing.
    """
    spacing = float(spacing)
    if not (math.isfinite(spacing) and spacing > 0.0):
        raise ValueError(f'spacing must be finite and > 0, got {spacing!r}')
    line = []
    for x, y in points:
        point = (float(x), float(y))
        if not line or point != line[-1]:
            line.append(point)
    if len(line) < 2:
        return []
    out = []
    target = 0.0    # distance along the line of the next point to return
    walked = 0.0    # distance along the line of the start of this segment
    heading = 0.0
    for (ax, ay), (bx, by) in zip(line, line[1:]):
        length = math.hypot(bx - ax, by - ay)
        heading = math.atan2(by - ay, bx - ax)
        while target <= walked + length:
            t = (target - walked) / length
            out.append((ax + t * (bx - ax), ay + t * (by - ay), heading))
            target += spacing
        walked += length
    end_x, end_y = line[-1]
    if math.hypot(out[-1][0] - end_x, out[-1][1] - end_y) > 1e-6:
        out.append((end_x, end_y, heading))
    else:
        out[-1] = (end_x, end_y, heading)
    return out
