"""Conversions between latitude/longitude and local east-north metres.

Treats the Earth as flat around a fixed origin point, using the WGS84
ellipsoid's local radii of curvature at that origin. Within a few
kilometres of the origin the error is centimetres, far below GPS error.
"""
import math

# WGS84 ellipsoid constants (the model GPS uses)
_A = 6_378_137.0              # equatorial radius, metres
_E2 = 6.69437999014e-3        # first eccentricity squared


def _radii(origin_lat):
    """Return (north-south, east-west) radii of curvature in metres."""
    s = math.sin(math.radians(origin_lat))
    denom = 1.0 - _E2 * s * s
    meridian = _A * (1.0 - _E2) / denom ** 1.5      # for north-south distances
    prime_vertical = _A / math.sqrt(denom)          # for east-west distances
    return meridian, prime_vertical


def latlon_to_enu(lat, lon, origin_lat, origin_lon):
    """Return (east, north) in metres of a lat/lon point relative to the origin."""
    meridian, prime_vertical = _radii(origin_lat)
    north = math.radians(lat - origin_lat) * meridian
    east = (math.radians(lon - origin_lon)
            * prime_vertical * math.cos(math.radians(origin_lat)))
    return east, north


def enu_to_latlon(east, north, origin_lat, origin_lon):
    """Return (lat, lon) of a point given in metres east and north of the origin."""
    meridian, prime_vertical = _radii(origin_lat)
    lat = origin_lat + math.degrees(north / meridian)
    lon = origin_lon + math.degrees(
        east / (prime_vertical * math.cos(math.radians(origin_lat))))
    return lat, lon
