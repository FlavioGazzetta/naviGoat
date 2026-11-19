"""Tests for guide_core.polyline."""

import math

from guide_core.polyline import point_segment_distance, resample_polyline, simplify_polyline
import pytest


# --- point_segment_distance --------------------------------------------------

def test_distance_perpendicular_to_middle():
    assert point_segment_distance((5, 3), (0, 0), (10, 0)) == pytest.approx(3.0)


def test_distance_beyond_end_is_to_endpoint():
    assert point_segment_distance((13, 4), (0, 0), (10, 0)) == pytest.approx(5.0)


def test_distance_before_start_is_to_startpoint():
    assert point_segment_distance((-3, -4), (0, 0), (10, 0)) == pytest.approx(5.0)


def test_distance_to_degenerate_segment():
    assert point_segment_distance((4, 5), (1, 1), (1, 1)) == pytest.approx(5.0)


# --- simplify_polyline -------------------------------------------------------

def test_collinear_points_dropped():
    points = [(0, 0), (10, 0), (20, 0), (30, 0), (40, 0)]
    assert simplify_polyline(points, 0.5) == [0, 4]


def test_small_wiggle_dropped():
    assert simplify_polyline([(0, 0), (5, 0.3), (10, 0)], 0.5) == [0, 2]


def test_bump_above_tolerance_kept():
    assert simplify_polyline([(0, 0), (5, 0.8), (10, 0)], 0.5) == [0, 1, 2]


def test_l_shape_keeps_corner():
    points = [(0, 0), (20, 0), (40, 0), (40, 15), (40, 30)]
    assert simplify_polyline(points, 0.5) == [0, 2, 4]


def test_out_and_back_keeps_turnaround():
    # (10, 0) lies on the infinite line through the ends, but 5 m past the segment.
    assert simplify_polyline([(0, 0), (10, 0), (5, 0)], 0.5) == [0, 1, 2]


def test_protected_collinear_point_kept():
    points = [(0, 0), (10, 0), (20, 0), (30, 0), (40, 0)]
    assert simplify_polyline(points, 0.5, keep=[2]) == [0, 2, 4]


def test_keep_may_repeat_and_include_ends():
    points = [(0, 0), (10, 0), (20, 0), (30, 0), (40, 0)]
    assert simplify_polyline(points, 0.5, keep=[0, 4, 4]) == [0, 4]


@pytest.mark.parametrize('points, expected', [
    ([], []),
    ([(1, 2)], [0]),
    ([(1, 2), (3, 4)], [0, 1]),
])
def test_short_inputs_returned_whole(points, expected):
    assert simplify_polyline(points, 0.5) == expected


def test_zero_tolerance_drops_only_exactly_collinear():
    points = [(0, 0), (5, 0), (10, 0), (10, 5)]
    assert simplify_polyline(points, 0.0) == [0, 2, 3]


@pytest.mark.parametrize('tolerance', [-0.1, math.nan, math.inf])
def test_invalid_tolerance_rejected(tolerance):
    with pytest.raises(ValueError):
        simplify_polyline([(0, 0), (1, 0), (2, 0)], tolerance)


@pytest.mark.parametrize('keep', [[5], [-1], [1.0], [True]])
def test_invalid_keep_rejected(keep):
    with pytest.raises(ValueError):
        simplify_polyline([(0, 0), (1, 0), (2, 0)], 0.5, keep=keep)


def test_result_stays_within_tolerance():
    # A wavy 100 m line: every original point must lie within 0.5 m of the result.
    points = [(float(i), 3.0 * math.sin(i / 8.0)) for i in range(101)]
    kept = simplify_polyline(points, 0.5)
    assert len(kept) < len(points)
    segments = list(zip(kept, kept[1:]))
    for p in points:
        nearest = min(point_segment_distance(p, points[a], points[b]) for a, b in segments)
        assert nearest <= 0.5 + 1e-9


# --- resample_polyline ---

def test_resample_straight_line_every_five_metres():
    points = resample_polyline([(0, 0), (20, 0)], 5.0)
    assert [(x, y) for x, y, _ in points] == [(0, 0), (5, 0), (10, 0), (15, 0), (20, 0)]
    assert all(heading == 0.0 for _, _, heading in points)


def test_resample_keeps_the_end_when_the_last_step_is_short():
    points = resample_polyline([(0, 0), (12, 0)], 5.0)
    assert [(x, y) for x, y, _ in points] == [(0, 0), (5, 0), (10, 0), (12, 0)]


def test_resample_measures_along_the_line_round_a_corner():
    # 8 m east, then 8 m north: the points are 5 m apart along the route, not in a straight line.
    points = resample_polyline([(0, 0), (8, 0), (8, 8)], 5.0)
    assert [(round(x, 6), round(y, 6)) for x, y, _ in points] == [
        (0, 0), (5, 0), (8, 2), (8, 7), (8, 8)]
    assert [round(heading, 6) for _, _, heading in points] == [
        0.0, 0.0, round(math.pi / 2, 6), round(math.pi / 2, 6), round(math.pi / 2, 6)]


def test_resample_skips_repeated_points_and_short_lines():
    assert resample_polyline([(1, 1), (1, 1)], 5.0) == []
    assert resample_polyline([(1, 1)], 5.0) == []
    assert resample_polyline([], 5.0) == []
    points = resample_polyline([(0, 0), (0, 0), (4, 0), (4, 0)], 5.0)
    assert [(x, y) for x, y, _ in points] == [(0, 0), (4, 0)]


@pytest.mark.parametrize('spacing', [0.0, -1.0, math.nan, math.inf])
def test_resample_invalid_spacing_rejected(spacing):
    with pytest.raises(ValueError):
        resample_polyline([(0, 0), (1, 0)], spacing)
