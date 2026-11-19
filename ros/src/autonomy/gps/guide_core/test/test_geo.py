"""Tests for guide_core.geo."""
import math

from guide_core.geo import enu_to_latlon, latlon_to_enu

ORIGIN = (51.4988, -0.1749)  # a point in London


def test_origin_maps_to_zero():
    east, north = latlon_to_enu(*ORIGIN, *ORIGIN)
    assert abs(east) < 1e-9 and abs(north) < 1e-9


def test_round_trip():
    lat, lon = enu_to_latlon(1234.5, -678.9, *ORIGIN)
    east, north = latlon_to_enu(lat, lon, *ORIGIN)
    assert math.isclose(east, 1234.5, abs_tol=1e-6)
    assert math.isclose(north, -678.9, abs_tol=1e-6)


def test_one_degree_of_latitude_is_about_111_km():
    _, north = latlon_to_enu(ORIGIN[0] + 1.0, ORIGIN[1], *ORIGIN)
    assert 110_000 < north < 112_000


def test_north_is_positive_y_and_east_is_positive_x():
    east, north = latlon_to_enu(ORIGIN[0] + 0.001, ORIGIN[1] + 0.001, *ORIGIN)
    assert east > 0 and north > 0
