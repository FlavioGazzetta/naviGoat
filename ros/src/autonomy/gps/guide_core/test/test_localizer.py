"""Tests for guide_core.localizer."""

import math

from guide_core.geo import enu_to_latlon
from guide_core.localizer import (
    GpsFix, GpsLocalizer, InvalidFixError, pose_covariance_6x6, PositionEstimate, UNKNOWN_VARIANCE,
)
import pytest

ORIGIN_LAT = 51.4988
ORIGIN_LON = -0.1749
TOL = 1e-6  # metres; flat-earth round trips are far more precise than this

# NavSatFix.position_covariance_type values used below.
COV_DIAGONAL_KNOWN = 2
COV_KNOWN = 3


def make_fix(east=0.0, north=0.0, status=0,
             covariance=None, covariance_type=0):
    """Build a GpsFix at a given east/north offset (metres) from the origin."""
    lat, lon = enu_to_latlon(east, north, ORIGIN_LAT, ORIGIN_LON)
    if covariance is None:
        covariance = [0.0] * 9
    return GpsFix(lat, lon, status, covariance, covariance_type)


@pytest.fixture
def localizer():
    """Return a localizer at the standard test origin with the default variance."""
    return GpsLocalizer(ORIGIN_LAT, ORIGIN_LON)


# --- Conversion -------------------------------------------------------------

def test_origin_maps_to_zero(localizer):
    est = localizer.convert(make_fix())
    assert est.x == pytest.approx(0.0, abs=TOL)
    assert est.y == pytest.approx(0.0, abs=TOL)


def test_ten_metres_north_gives_y_ten(localizer):
    est = localizer.convert(make_fix(north=10.0))
    assert est.x == pytest.approx(0.0, abs=TOL)
    assert est.y == pytest.approx(10.0, abs=TOL)


def test_ten_metres_east_gives_x_ten(localizer):
    est = localizer.convert(make_fix(east=10.0))
    assert est.x == pytest.approx(10.0, abs=TOL)
    assert est.y == pytest.approx(0.0, abs=TOL)


# --- Fix status -------------------------------------------------------------

@pytest.mark.parametrize('status', [0, 1, 2])  # FIX, SBAS_FIX, GBAS_FIX
def test_valid_statuses_accepted(localizer, status):
    est = localizer.convert(make_fix(north=5.0, status=status))
    assert est.y == pytest.approx(5.0, abs=TOL)


def test_no_fix_rejected(localizer):
    with pytest.raises(InvalidFixError):
        localizer.convert(make_fix(status=-1))


# --- Bad coordinates --------------------------------------------------------

@pytest.mark.parametrize('lat, lon', [
    (math.nan, ORIGIN_LON),
    (ORIGIN_LAT, math.nan),
    (math.inf, ORIGIN_LON),
    (91.0, ORIGIN_LON),
    (ORIGIN_LAT, 181.0),
])
def test_invalid_coordinates_rejected(localizer, lat, lon):
    fix = GpsFix(lat, lon, 0, [0.0] * 9, 0)
    with pytest.raises(InvalidFixError):
        localizer.convert(fix)


# --- Uncertainty ------------------------------------------------------------

def test_unknown_covariance_uses_default(localizer):
    est = localizer.convert(make_fix(covariance_type=0))
    assert est.var_x == pytest.approx(25.0)
    assert est.var_y == pytest.approx(25.0)
    assert est.cov_xy == pytest.approx(0.0)


def test_known_covariance_passed_through(localizer):
    # Row-major 3x3 in east-north-up order.
    cov = [4.0, 0.5, 0.0,
           0.5, 9.0, 0.0,
           0.0, 0.0, 16.0]
    est = localizer.convert(make_fix(covariance=cov,
                                     covariance_type=COV_KNOWN))
    assert est.var_x == pytest.approx(4.0)
    assert est.var_y == pytest.approx(9.0)
    assert est.cov_xy == pytest.approx(0.5)


@pytest.mark.parametrize('cov', [
    [0.0] * 9,                                          # zero variance
    [math.nan, 0, 0, 0, 4.0, 0, 0, 0, 9.0],             # NaN
    [1.0, 0.0, 0.0, 1.0],                               # wrong length
])
def test_unusable_known_covariance_uses_default(localizer, cov):
    est = localizer.convert(make_fix(covariance=cov,
                                     covariance_type=COV_DIAGONAL_KNOWN))
    assert est.var_x == pytest.approx(25.0)
    assert est.var_y == pytest.approx(25.0)
    assert est.cov_xy == pytest.approx(0.0)


# --- Setup ------------------------------------------------------------------

@pytest.mark.parametrize('lat, lon', [(math.nan, 0.0), (95.0, 0.0)])
def test_invalid_origin_raises(lat, lon):
    with pytest.raises(ValueError):
        GpsLocalizer(lat, lon)


@pytest.mark.parametrize('variance', [0.0, -1.0])
def test_invalid_default_variance_raises(variance):
    with pytest.raises(ValueError):
        GpsLocalizer(ORIGIN_LAT, ORIGIN_LON, default_variance=variance)


# --- 6x6 pose covariance ----------------------------------------------------

def test_pose_covariance_layout():
    est = PositionEstimate(x=0.0, y=0.0, var_x=4.0, var_y=9.0, cov_xy=0.5)
    cov = pose_covariance_6x6(est, yaw_variance=0.01)
    assert len(cov) == 36
    expected = {
        0: 4.0, 1: 0.5, 6: 0.5, 7: 9.0,
        14: UNKNOWN_VARIANCE, 21: UNKNOWN_VARIANCE, 28: UNKNOWN_VARIANCE,
        35: 0.01,
    }
    for i, value in enumerate(cov):
        assert value == pytest.approx(expected.get(i, 0.0)), f'index {i}'


def test_pose_covariance_symmetric():
    est = PositionEstimate(x=0.0, y=0.0, var_x=4.0, var_y=9.0, cov_xy=0.5)
    cov = pose_covariance_6x6(est, yaw_variance=0.01)
    for i in range(6):
        for j in range(6):
            assert cov[6 * i + j] == cov[6 * j + i]
