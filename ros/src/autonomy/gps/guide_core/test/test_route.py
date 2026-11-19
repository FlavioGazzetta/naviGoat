"""Tests for guide_core.route: construction, validation and lengths."""

import math

from guide_core.route import InvalidRouteError, Route, route_from_dict
import pytest

L_ROUTE = [[0.0, 0.0], [40.0, 0.0], [40.0, 30.0]]


# --- Lengths -----------------------------------------------------------------

def test_l_route_lengths():
    route = Route(L_ROUTE)
    assert route.num_segments == 2
    assert route.segment_lengths == pytest.approx((40.0, 30.0))
    assert route.cumulative == pytest.approx((0.0, 40.0, 70.0))
    assert route.total_length == pytest.approx(70.0)


def test_diagonal_segment_length():
    # 3-4-5 triangle: a single diagonal segment of length 5.
    route = Route([[0, 0], [3, 4]])
    assert route.total_length == pytest.approx(5.0)


def test_points_are_float_tuples():
    route = Route([[0, 0], [1, 2]])
    assert route.points == ((0.0, 0.0), (1.0, 2.0))
    assert all(isinstance(c, float) for p in route.points for c in p)


def test_route_is_read_only():
    route = Route(L_ROUTE)
    with pytest.raises(AttributeError):
        route.points = ()


# --- Duplicates --------------------------------------------------------------

def test_consecutive_duplicates_removed():
    route = Route([[0, 0], [0, 0], [10, 0], [10, 0], [10, 5]])
    assert route.points == ((0.0, 0.0), (10.0, 0.0), (10.0, 5.0))
    assert route.num_segments == 2


def test_loop_back_to_start_is_kept():
    # Returning to the start is a real route, not a duplicate.
    route = Route([[0, 0], [10, 0], [10, 10], [0, 0]])
    assert len(route.points) == 4
    assert route.total_length == pytest.approx(20.0 + math.hypot(10, 10))


# --- Invalid waypoints -------------------------------------------------------

@pytest.mark.parametrize('waypoints', [
    [],
    [[1.0, 2.0]],
    [[1.0, 2.0], [1.0, 2.0]],  # only one distinct point
])
def test_too_few_points(waypoints):
    with pytest.raises(InvalidRouteError):
        Route(waypoints)


@pytest.mark.parametrize('bad', [
    [1.0],
    [1.0, 2.0, 3.0],
    'ab',
    5.0,
    None,
])
def test_wrong_shape_rejected(bad):
    with pytest.raises(InvalidRouteError):
        Route([[0.0, 0.0], bad])


@pytest.mark.parametrize('bad', [
    [math.nan, 0.0],
    [0.0, math.inf],
    [True, 1.0],
    ['1.0', 2.0],
])
def test_bad_coordinates_rejected(bad):
    with pytest.raises(InvalidRouteError):
        Route([[0.0, 0.0], bad])


def test_error_message_names_waypoint_index():
    with pytest.raises(InvalidRouteError, match='waypoint 2'):
        Route([[0, 0], [1, 0], [math.nan, 0]])


# --- route_from_dict ---------------------------------------------------------

def test_from_dict_valid():
    route = route_from_dict({'frame_id': 'map', 'waypoints': L_ROUTE})
    assert route.total_length == pytest.approx(70.0)


def test_from_dict_wrong_frame():
    with pytest.raises(InvalidRouteError, match='frame_id'):
        route_from_dict({'frame_id': 'odom', 'waypoints': L_ROUTE})


def test_from_dict_missing_frame():
    with pytest.raises(InvalidRouteError, match='frame_id'):
        route_from_dict({'waypoints': L_ROUTE})


def test_from_dict_missing_waypoints():
    with pytest.raises(InvalidRouteError, match='waypoints'):
        route_from_dict({'frame_id': 'map'})


@pytest.mark.parametrize('data', [None, [], 'route'])
def test_from_dict_not_a_mapping(data):
    with pytest.raises(InvalidRouteError):
        route_from_dict(data)

# --- Projection --------------------------------------------------------------
# L route: (0,0) -> (40,0) heading east, then (40,0) -> (40,30) heading north.


def test_point_on_route():
    p = Route(L_ROUTE).project(10.0, 0.0)
    assert p.s == pytest.approx(10.0)
    assert p.segment_index == 0
    assert p.point == pytest.approx((10.0, 0.0))
    assert p.cross_track_error == pytest.approx(0.0)


@pytest.mark.parametrize('y, expected_cte', [(2.0, 2.0), (-2.0, -2.0)])
def test_side_sign_first_leg(y, expected_cte):
    # Heading east: north (+y) is left, so positive.
    p = Route(L_ROUTE).project(10.0, y)
    assert p.s == pytest.approx(10.0)
    assert p.cross_track_error == pytest.approx(expected_cte)


@pytest.mark.parametrize('x, expected_cte', [(38.0, 2.0), (42.0, -2.0)])
def test_side_sign_second_leg(x, expected_cte):
    # Heading north: west (smaller x) is left, so positive.
    p = Route(L_ROUTE).project(x, 10.0)
    assert p.segment_index == 1
    assert p.s == pytest.approx(50.0)
    assert p.cross_track_error == pytest.approx(expected_cte)


def test_before_start_snaps_to_start():
    # 3 m behind and 4 m right of the start: 5 m from (0, 0), right side.
    p = Route(L_ROUTE).project(-3.0, -4.0)
    assert p.s == pytest.approx(0.0)
    assert p.point == pytest.approx((0.0, 0.0))
    assert p.cross_track_error == pytest.approx(-5.0)


def test_past_end_snaps_to_end():
    p = Route(L_ROUTE).project(40.0, 35.0)
    assert p.s == pytest.approx(70.0)
    assert p.segment_index == 1
    assert abs(p.cross_track_error) == pytest.approx(5.0)


def test_outside_corner_measures_distance_to_corner():
    # Outside (right) of the left turn: closest point is the corner itself.
    p = Route(L_ROUTE).project(45.0, -5.0)
    assert p.s == pytest.approx(40.0)
    assert p.point == pytest.approx((40.0, 0.0))
    assert p.cross_track_error == pytest.approx(-math.hypot(5.0, 5.0))


def test_inside_corner_tie_keeps_earlier_segment():
    # Exactly 5 m from both legs: the earlier segment wins.
    p = Route(L_ROUTE).project(35.0, 5.0)
    assert p.segment_index == 0
    assert p.s == pytest.approx(35.0)
    assert p.cross_track_error == pytest.approx(5.0)


# Out-and-back route along both sides of a 4 m-wide street:
# (0,0) -> (50,0) east, (50,0) -> (50,4) north, (50,4) -> (0,4) west.
OUT_AND_BACK = [[0, 0], [50, 0], [50, 4], [0, 4]]


def test_doubling_back_global_search_jumps_to_return_leg():
    # Robot on the outward leg but drifted 2.5 m north: the return leg is
    # closer (1.5 m), so an unwindowed search picks the wrong part.
    p = Route(OUT_AND_BACK).project(10.0, 2.5)
    assert p.segment_index == 2
    assert p.s == pytest.approx(94.0)  # 50 + 4 + 40


def test_doubling_back_window_keeps_outward_leg():
    # With a window around the robot's known progress, it stays on leg 0.
    p = Route(OUT_AND_BACK).project(10.0, 2.5, s_min=0.0, s_max=30.0)
    assert p.segment_index == 0
    assert p.s == pytest.approx(10.0)
    assert p.cross_track_error == pytest.approx(2.5)


def test_window_clamps_inside_a_segment():
    # Robot at s = 10, but the window starts at 20: clamped to s = 20.
    p = Route(L_ROUTE).project(10.0, 0.0, s_min=20.0, s_max=40.0)
    assert p.s == pytest.approx(20.0)
    assert p.point == pytest.approx((20.0, 0.0))
    assert abs(p.cross_track_error) == pytest.approx(10.0)


def test_window_beyond_route_is_clamped():
    route = Route(L_ROUTE)
    assert route.project(38.0, 10.0, s_min=-100.0, s_max=1000.0) == \
        route.project(38.0, 10.0)


def test_inverted_window_rejected():
    with pytest.raises(ValueError):
        Route(L_ROUTE).project(0.0, 0.0, s_min=30.0, s_max=10.0)


def test_projection_is_frozen():
    p = Route(L_ROUTE).project(10.0, 0.0)
    with pytest.raises(AttributeError):
        p.s = 0.0


# --- point_at / segment_index_at / heading_at --------------------------------

@pytest.mark.parametrize('s, expected', [
    (0.0, (0.0, 0.0)),
    (10.0, (10.0, 0.0)),
    (40.0, (40.0, 0.0)),     # the corner
    (50.0, (40.0, 10.0)),    # 10 m up the second leg
    (70.0, (40.0, 30.0)),    # the end
    (-5.0, (0.0, 0.0)),      # before the start: clamped
    (100.0, (40.0, 30.0)),   # past the end: clamped
])
def test_point_at(s, expected):
    assert Route(L_ROUTE).point_at(s) == pytest.approx(expected)


def test_point_at_diagonal():
    # Halfway along the 3-4-5 segment.
    assert Route([[0, 0], [3, 4]]).point_at(2.5) == pytest.approx((1.5, 2.0))


@pytest.mark.parametrize('s, expected', [
    (-1.0, 0),
    (39.9, 0),
    (40.0, 1),   # at the corner: the outgoing segment
    (70.0, 1),   # at the end: the last segment
])
def test_segment_index_at(s, expected):
    assert Route(L_ROUTE).segment_index_at(s) == expected


@pytest.mark.parametrize('s, expected', [
    (10.0, 0.0),             # heading east
    (40.0, math.pi / 2),     # at the corner: already facing north
    (70.0, math.pi / 2),     # at the end: last segment's heading
])
def test_heading_at_l_route(s, expected):
    assert Route(L_ROUTE).heading_at(s) == pytest.approx(expected)


def test_heading_west():
    # The return leg of OUT_AND_BACK heads west: +-pi.
    assert abs(Route(OUT_AND_BACK).heading_at(60.0)) == pytest.approx(math.pi)


@pytest.mark.parametrize('x, y', [(10.0, 2.0), (38.0, 10.0), (45.0, -5.0)])
def test_point_at_matches_projection(x, y):
    # project() and point_at() must agree: going position -> s -> position
    # lands on the projected point.
    route = Route(L_ROUTE)
    p = route.project(x, y)
    assert route.point_at(p.s) == pytest.approx(p.point)


# --- Corners -----------------------------------------------------------------

def test_l_route_corner():
    route = Route(L_ROUTE)
    assert route.turn_angle(1) == pytest.approx(math.pi / 2)  # left turn
    assert route.corners(math.radians(30)) == (1,)


def test_right_turn_is_negative():
    route = Route([[0, 0], [10, 0], [10, -10]])
    assert route.turn_angle(1) == pytest.approx(-math.pi / 2)


def test_straight_through_waypoint_is_not_corner():
    # Waypoint 1 is in a straight line (as OSM often has); waypoint 2 turns.
    route = Route([[0, 0], [10, 0], [20, 0], [20, 10]])
    assert route.turn_angle(1) == pytest.approx(0.0)
    assert route.corners(math.radians(30)) == (2,)


@pytest.mark.parametrize('threshold_deg, expected', [(30.0, ()), (10.0, (1,))])
def test_gentle_bend_threshold(threshold_deg, expected):
    # A 20 degree bend: a corner only if the threshold is below 20.
    bend = math.radians(20)
    route = Route([[0, 0], [10, 0], [10 + 10 * math.cos(bend), 10 * math.sin(bend)]])
    assert route.corners(math.radians(threshold_deg)) == expected


def test_out_and_back_has_two_left_corners():
    route = Route(OUT_AND_BACK)
    assert route.corners(math.radians(30)) == (1, 2)
    assert route.turn_angle(1) == pytest.approx(math.pi / 2)
    assert route.turn_angle(2) == pytest.approx(math.pi / 2)


def test_u_turn_is_corner():
    route = Route([[0, 0], [10, 0], [0, 0]])
    assert abs(route.turn_angle(1)) == pytest.approx(math.pi)
    assert route.corners(math.radians(30)) == (1,)


def test_negative_threshold_rejected():
    with pytest.raises(ValueError):
        Route(L_ROUTE).corners(-0.1)


@pytest.mark.parametrize('index', [0, 2, -1])
def test_turn_angle_needs_interior_waypoint(index):
    # L route has 3 waypoints; only index 1 is interior.
    with pytest.raises(IndexError):
        Route(L_ROUTE).turn_angle(index)
