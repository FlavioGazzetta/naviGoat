"""Localizer v0: convert GPS fixes into positions in the local map frame.

No filtering: each valid fix becomes one position estimate.

Assumption (v0 only): the ``map`` frame is identical to the local
east-north (ENU) frame around the configured origin, with x = east and
y = north. This holds in the simulator. On the real robot, SLAM's ``map``
frame has a different origin and rotation; aligning the two is Step 8.
"""

from dataclasses import dataclass
import math
from typing import Sequence

from guide_core.geo import latlon_to_enu

# Copies of constants from sensor_msgs/NavSatStatus and sensor_msgs/NavSatFix,
# so this module does not need to import ROS.
STATUS_NO_FIX = -1
COVARIANCE_TYPE_UNKNOWN = 0

# Variance used for pose components we do not measure (z, roll, pitch).
# A huge value is the usual way of saying "don't trust this number at all".
UNKNOWN_VARIANCE = 1.0e6


class InvalidFixError(ValueError):
    """Raised when a GPS fix must not be used. The message explains why."""


@dataclass(frozen=True)
class GpsFix:
    """The parts of a GPS fix the localizer needs.

    Attributes:
        latitude: Degrees, positive north.
        longitude: Degrees, positive east.
        status: Fix status; ``STATUS_NO_FIX`` (-1) means no valid fix.
        position_covariance: 9 values, a row-major 3x3 matrix in
            east-north-up order, in square metres.
        position_covariance_type: 0 means the covariance is unknown.
    """

    latitude: float
    longitude: float
    status: int
    position_covariance: Sequence[float]
    position_covariance_type: int


@dataclass(frozen=True)
class PositionEstimate:
    """A position in the map frame with its uncertainty.

    Attributes:
        x: Metres east of the origin.
        y: Metres north of the origin.
        var_x: Variance of x in square metres.
        var_y: Variance of y in square metres.
        cov_xy: Covariance between x and y in square metres.
    """

    x: float
    y: float
    var_x: float
    var_y: float
    cov_xy: float


class GpsLocalizer:
    """Converts GPS fixes to map-frame positions around a fixed origin."""

    def __init__(self, origin_lat: float, origin_lon: float,
                 default_variance: float = 25.0):
        """Create a localizer.

        Args:
            origin_lat: Latitude of the map origin, degrees.
            origin_lon: Longitude of the map origin, degrees.
            default_variance: Variance (m^2) used for x and y when the
                receiver does not report a usable covariance. 25 m^2
                means a standard deviation of 5 m.

        Raises:
            ValueError: If the origin or default variance is invalid.
        """
        if not _is_valid_latlon(origin_lat, origin_lon):
            raise ValueError(
                f'invalid origin: lat={origin_lat}, lon={origin_lon}')
        if not (math.isfinite(default_variance) and default_variance > 0.0):
            raise ValueError(
                f'default_variance must be positive, got {default_variance}')
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.default_variance = default_variance

    def convert(self, fix: GpsFix) -> PositionEstimate:
        """Convert one GPS fix to a map-frame position.

        Args:
            fix: The GPS fix to convert.

        Returns:
            The position and its uncertainty.

        Raises:
            InvalidFixError: If the fix must not be used.
        """
        if fix.status <= STATUS_NO_FIX:
            raise InvalidFixError('receiver reports no satellite fix')
        if not _is_valid_latlon(fix.latitude, fix.longitude):
            raise InvalidFixError(
                f'invalid coordinates: lat={fix.latitude}, '
                f'lon={fix.longitude}')

        x, y = latlon_to_enu(fix.latitude, fix.longitude,
                             self.origin_lat, self.origin_lon)
        var_x, var_y, cov_xy = self._position_covariance(fix)
        return PositionEstimate(x=x, y=y, var_x=var_x, var_y=var_y,
                                cov_xy=cov_xy)

    def _position_covariance(self, fix: GpsFix) -> tuple[float, float, float]:
        """Return (var_x, var_y, cov_xy) for a fix, using the default if needed.

        The receiver's matrix is in east-north-up order, and in v0 the map
        frame is east-north, so east maps to x and north maps to y directly.
        """
        default = (self.default_variance, self.default_variance, 0.0)
        if fix.position_covariance_type == COVARIANCE_TYPE_UNKNOWN:
            return default
        c = fix.position_covariance
        if len(c) != 9:
            return default
        var_x, cov_xy, var_y = c[0], c[1], c[4]
        if not all(math.isfinite(v) for v in (var_x, var_y, cov_xy)):
            return default
        if var_x <= 0.0 or var_y <= 0.0:
            return default
        return var_x, var_y, cov_xy


def pose_covariance_6x6(estimate: PositionEstimate,
                        yaw_variance: float) -> list[float]:
    """Build the 36-value covariance used by ROS pose messages.

    The matrix is row-major over (x, y, z, roll, pitch, yaw), so element
    (row i, column j) is at index 6 * i + j. Components we do not measure
    (z, roll, pitch) get ``UNKNOWN_VARIANCE``.

    Args:
        estimate: The position estimate supplying the x/y terms.
        yaw_variance: Variance of the heading, in square radians.

    Returns:
        A list of 36 floats.
    """
    cov = [0.0] * 36
    cov[0] = estimate.var_x                    # (x, x)
    cov[1] = cov[6] = estimate.cov_xy          # (x, y) and (y, x)
    cov[7] = estimate.var_y                    # (y, y)
    cov[14] = UNKNOWN_VARIANCE                 # (z, z)
    cov[21] = UNKNOWN_VARIANCE                 # (roll, roll)
    cov[28] = UNKNOWN_VARIANCE                 # (pitch, pitch)
    cov[35] = yaw_variance                     # (yaw, yaw)
    return cov


def _is_valid_latlon(lat: float, lon: float) -> bool:
    """Return True if lat/lon are finite and within their legal ranges."""
    return (math.isfinite(lat) and math.isfinite(lon)
            and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0)
