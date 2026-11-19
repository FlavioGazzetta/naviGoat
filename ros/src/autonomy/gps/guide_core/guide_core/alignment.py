"""Alignment filter: where the SLAM map frame sits in the ENU frame (Step 8).

The robot's pose in the map frame (from RTAB-Map, or plain odometry) is smooth
and locally accurate but anchored nowhere; GPS is anchored to the world but
noisy. This filter estimates the transform T = "map frame expressed in ENU"
(x, y, yaw), so that the robot's ENU pose is T composed with its map pose:
smooth like SLAM, anchored like GPS.

It is a small extended Kalman filter with 5 states: T (x, y, yaw) and the GPS
receiver's slowly wandering bias (east, north). Tracking the bias keeps the
filter honest: consecutive fixes share most of their error, so averaging many
of them must not make the filter believe it is far more accurate than it is.
* predict: T drifts a little as the robot drives (SLAM/odometry drift), so its
  uncertainty grows with the distance driven and the angle turned. The drift is modelled at the
  robot's position (a turn of the map frame about the robot), as real SLAM
  drifts. The bias relaxes towards zero over gps_bias_time seconds.
* GPS update: a fix in ENU, paired with the map pose at the same moment. The
  receiver's reported sigma is split into a bias part (gps_bias_fraction) and
  a white-noise part (gps_white_fraction). Fixes that disagree too much with
  the prediction (Mahalanobis distance above a gate) are rejected: the
  reported sigma cannot be trusted during multipath jumps. If fixes keep being
  rejected for reacquire_time seconds, the position uncertainty is widened so
  the filter can follow a real change (e.g. a SLAM relocalisation).
* IMU heading update: an absolute heading (compass-like) measures T's yaw.
  It is used only while the filter's yaw is less certain than the compass,
  so a biased compass helps at the start but cannot drag the estimate later.

Initialisation: from the first fix plus an IMU heading if one is known;
otherwise once the robot has moved init_baseline metres between two fixes
(heading from the direction of travel in both frames).

Pure Python (no numpy, no rclpy); matrices are lists of rows.
"""

from dataclasses import dataclass
import enum
import math

from guide_core.se2 import compose, Pose2D, wrap_angle

# State order: T's x, y, yaw, then the GPS bias east, north.
X, Y, YAW, BE, BN = range(5)
N = 5


@dataclass(frozen=True)
class AlignmentConfig:
    """Tuning of the alignment filter."""

    # The receiver's reported sigma s is split into a slowly wandering bias with
    # sigma s * gps_bias_fraction and white noise with sigma
    # max(s * gps_white_fraction, min_gps_sigma).
    gps_bias_fraction: float = 0.6
    gps_white_fraction: float = 0.3
    min_gps_sigma: float = 0.5
    # How long the GPS bias takes to change (s).
    gps_bias_time: float = 60.0
    # Fuse at most one fix per this many seconds (receivers smooth their output).
    min_fix_interval: float = 1.0
    # Mahalanobis gate (squared) for GPS fixes: 13.8 = chi-square 2 dof, 99.9 %.
    gps_gate: float = 13.8
    # Gate (squared) for IMU headings: 10.8 = chi-square 1 dof, 99.9 %.
    heading_gate: float = 10.8
    # Drift of the map frame at the robot, per sqrt(metre) driven.
    position_drift: float = 0.02
    yaw_drift: float = 0.002
    # Extra yaw drift per sqrt(radian) turned (skid-steer slip when turning).
    turn_drift: float = 0.01
    # Use IMU headings (compass-like) at all, and their sigma (rad).
    use_imu_heading: bool = True
    imu_yaw_sigma: float = math.radians(15.0)
    # Without an IMU heading: metres between two fixes needed to initialise.
    init_baseline: float = 15.0
    # After this many seconds of rejected fixes, widen the position uncertainty.
    reacquire_time: float = 30.0
    reacquire_sigma: float = 10.0

    def __post_init__(self):
        for name in ('gps_white_fraction', 'min_gps_sigma', 'gps_bias_time', 'gps_gate',
                     'heading_gate', 'imu_yaw_sigma', 'init_baseline', 'reacquire_time',
                     'reacquire_sigma'):
            if not getattr(self, name) > 0.0:
                raise ValueError(f'{name} must be > 0')
        for name in ('gps_bias_fraction', 'min_fix_interval', 'position_drift', 'yaw_drift',
                     'turn_drift'):
            if getattr(self, name) < 0.0:
                raise ValueError(f'{name} must be >= 0')


class FixResult(enum.Enum):
    """What the filter did with one GPS fix."""

    INITIALISED = 'initialised'
    ACCEPTED = 'accepted'
    REACQUIRED = 'reacquired'
    REJECTED = 'rejected'
    SKIPPED = 'skipped'        # too soon after the previous fused fix
    WAITING = 'waiting'        # not initialised yet: needs more baseline


@dataclass(frozen=True)
class FixUpdate:
    """Outcome of one GPS fix: what happened and how surprising it was."""

    result: FixResult
    # sqrt of the Mahalanobis distance (in "sigmas"); 0 if not computed.
    distance: float = 0.0


def _zeros(rows, cols):
    return [[0.0] * cols for _ in range(rows)]


def _identity(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
            for i in range(len(a))]


def _transpose(a):
    return [list(row) for row in zip(*a)]


def _add(a, b):
    return [[x + y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def _sub(a, b):
    return [[x - y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def _symmetric(a):
    n = len(a)
    return [[(a[i][j] + a[j][i]) / 2.0 for j in range(n)] for i in range(n)]


def _quadratic(m, p):
    """Return m p m^T."""
    return _mat_mul(_mat_mul(m, p), _transpose(m))


def _inverse_2x2(m):
    (a, b), (c, d) = m
    det = a * d - b * c
    if det <= 0.0:
        raise ValueError('innovation covariance is not positive definite')
    return [[d / det, -b / det], [-c / det, a / det]]


def largest_eigenvalue_2x2(m):
    """Return the larger eigenvalue of a symmetric 2x2 matrix."""
    (a, b), (_, c) = m
    return (a + c) / 2.0 + math.sqrt(((a - c) / 2.0) ** 2 + b * b)


class AlignmentFilter:
    """Estimate T = the map frame in ENU from GPS, IMU heading and map poses."""

    def __init__(self, config=None):
        self.config = config or AlignmentConfig()
        self.transform = None          # Pose2D (map frame in ENU) once initialised
        self.bias = (0.0, 0.0)         # estimated GPS bias (east, north), metres
        self.covariance = None         # 5x5 list of rows, state order X Y YAW BE BN
        self.distance_since_fix = 0.0  # metres driven since the last accepted fix
        self.last_fix_time = None      # time of the last accepted fix (s)
        self._bias_sigma = 0.0         # current bias sigma (from the reported sigma)
        self._last_map_pose = None
        self._last_time = None
        self._last_fused_time = None
        self._rejecting_since = None
        self._heading_yaw = None       # yaw of T suggested by the IMU (before init)
        self._first_fix = None         # (east, north, map pose, sigma) without a heading

    @property
    def initialised(self):
        """Return True once T has been estimated."""
        return self.transform is not None

    # --- Outputs -------------------------------------------------------

    def robot_pose(self, map_pose):
        """Return the robot's ENU pose for a map pose (None before initialisation)."""
        if not self.initialised:
            return None
        return compose(self.transform, Pose2D(*map_pose))

    def _position_jacobian(self, map_pose):
        """Return d(robot ENU position)/d(state) at a map pose (2x5, bias excluded)."""
        t = self.transform
        c, s = math.cos(t.yaw), math.sin(t.yaw)
        px, py = map_pose[0], map_pose[1]
        return [[1.0, 0.0, -s * px - c * py, 0.0, 0.0],
                [0.0, 1.0, c * px - s * py, 0.0, 0.0]]

    def position_covariance(self, map_pose):
        """Return the 2x2 covariance of the robot's ENU position (None before init)."""
        if not self.initialised:
            return None
        return _quadratic(self._position_jacobian(map_pose), self.covariance)

    def position_sigma(self, map_pose):
        """Return the robot's position sigma in metres (largest axis), or inf."""
        cov = self.position_covariance(map_pose)
        if cov is None:
            return math.inf
        return math.sqrt(max(0.0, largest_eigenvalue_2x2(cov)))

    def yaw_sigma(self):
        """Return the sigma of T's yaw (= of the robot's ENU heading), or inf."""
        if not self.initialised:
            return math.inf
        return math.sqrt(max(0.0, self.covariance[YAW][YAW]))

    def seconds_since_fix(self, now):
        """Return seconds since the last accepted fix (inf if none)."""
        return math.inf if self.last_fix_time is None else now - self.last_fix_time

    # --- Prediction ----------------------------------------------------

    def predict(self, map_pose, now=None):
        """Account for map-frame drift and GPS-bias change since the previous call."""
        map_pose = Pose2D(*map_pose)
        previous, self._last_map_pose = self._last_map_pose, map_pose
        d = turn = 0.0
        if previous is not None:
            d = math.hypot(map_pose.x - previous.x, map_pose.y - previous.y)
            turn = abs(wrap_angle(map_pose.yaw - previous.yaw))
        self.distance_since_fix += d
        dt = 0.0
        if now is not None:
            if self._last_time is not None:
                dt = max(0.0, now - self._last_time)
            self._last_time = now
        if not self.initialised:
            return
        c = self.config
        p = self.covariance
        if d > 0.0 or turn > 0.0:
            # Process noise at the robot (dx, dy, dyaw in ENU), mapped onto T: turning
            # the map frame by dyaw about the robot's position r moves T's origin too.
            rx, ry, _ = self.robot_pose(map_pose)
            t = self.transform
            g = _identity(N)
            g[X][YAW] = -(t.y - ry)
            g[Y][YAW] = t.x - rx
            q = _zeros(N, N)
            q[X][X] = q[Y][Y] = c.position_drift ** 2 * d
            q[YAW][YAW] = c.yaw_drift ** 2 * d + c.turn_drift ** 2 * turn
            p = _add(p, _quadratic(g, q))
        if dt > 0.0:
            # First-order Gauss-Markov bias: b <- a b, variance relaxes to sigma_b^2.
            a = math.exp(-dt / c.gps_bias_time)
            f = _identity(N)
            f[BE][BE] = f[BN][BN] = a
            self.bias = (self.bias[0] * a, self.bias[1] * a)
            p = _quadratic(f, p)
            p[BE][BE] += self._bias_sigma ** 2 * (1.0 - a * a)
            p[BN][BN] += self._bias_sigma ** 2 * (1.0 - a * a)
        self.covariance = _symmetric(p)

    # --- Measurements --------------------------------------------------

    def heading(self, imu_yaw, map_pose):
        """Use an absolute (ENU) heading; return True if it changed the estimate."""
        c = self.config
        if not c.use_imu_heading:
            return False
        if not all(math.isfinite(v) for v in (imu_yaw, *map_pose)):
            raise ValueError('heading and map pose must be finite')
        measured = wrap_angle(imu_yaw - map_pose[2])
        if not self.initialised:
            self._heading_yaw = measured
            return False
        self.predict(map_pose)
        p = self.covariance
        if math.sqrt(p[YAW][YAW]) <= c.imu_yaw_sigma:
            return False
        innovation = wrap_angle(measured - self.transform.yaw)
        s = p[YAW][YAW] + c.imu_yaw_sigma ** 2
        if innovation * innovation / s > c.heading_gate:
            return False
        k = [p[i][YAW] / s for i in range(N)]
        self._correct([ki * innovation for ki in k])
        self.covariance = _symmetric([[p[i][j] - k[i] * k[j] * s for j in range(N)]
                                      for i in range(N)])
        return True

    def gps(self, now, east, north, sigma, map_pose):
        """Use one GPS fix (ENU metres, reported sigma) taken at a map pose."""
        if not all(math.isfinite(v) for v in (now, east, north, sigma, *map_pose)):
            raise ValueError('GPS fix and map pose must be finite')
        c = self.config
        map_pose = Pose2D(*map_pose)
        self._bias_sigma = sigma * c.gps_bias_fraction
        white = max(sigma * c.gps_white_fraction, c.min_gps_sigma)
        if not self.initialised:
            return self._initialise(now, east, north, white, map_pose)
        self.predict(map_pose, now)
        if self._last_fused_time is not None and \
                now - self._last_fused_time < c.min_fix_interval:
            return FixUpdate(FixResult.SKIPPED)
        self._last_fused_time = now

        result = FixResult.ACCEPTED
        innovation, s, h, d2 = self._innovation(east, north, white, map_pose)
        if d2 > c.gps_gate:
            if self._rejecting_since is None:
                self._rejecting_since = now
            if now - self._rejecting_since < c.reacquire_time:
                return FixUpdate(FixResult.REJECTED, math.sqrt(d2))
            # Fixes have disagreed for a long time: trust them again.
            self.covariance[X][X] += c.reacquire_sigma ** 2
            self.covariance[Y][Y] += c.reacquire_sigma ** 2
            innovation, s, h, d2 = self._innovation(east, north, white, map_pose)
            result = FixResult.REACQUIRED
        self._rejecting_since = None

        # Kalman gain K = P H^T S^-1, then state += K y and the Joseph-form update
        # P = (I - K H) P (I - K H)^T + K R K^T, which stays symmetric and positive.
        p = self.covariance
        k = _mat_mul(_mat_mul(p, _transpose(h)), _inverse_2x2(s))
        self._correct([k[i][0] * innovation[0] + k[i][1] * innovation[1] for i in range(N)])
        i_kh = _sub(_identity(N), _mat_mul(k, h))
        r = [[white ** 2, 0.0], [0.0, white ** 2]]
        self.covariance = _symmetric(_add(_quadratic(i_kh, p), _quadratic(k, r)))
        self.last_fix_time = now
        self.distance_since_fix = 0.0
        return FixUpdate(result, math.sqrt(d2))

    def _correct(self, dx):
        """Add a state correction (5 values) to T and the bias."""
        t = self.transform
        self.transform = Pose2D(t.x + dx[X], t.y + dx[Y], wrap_angle(t.yaw + dx[YAW]))
        self.bias = (self.bias[0] + dx[BE], self.bias[1] + dx[BN])

    def _innovation(self, east, north, white, map_pose):
        """Return (innovation, S, H, squared Mahalanobis distance) for a fix."""
        rx, ry, _ = self.robot_pose(map_pose)
        y = [east - (rx + self.bias[0]), north - (ry + self.bias[1])]
        h = self._position_jacobian(map_pose)
        h[0][BE] = h[1][BN] = 1.0
        s = _quadratic(h, self.covariance)
        s[0][0] += white ** 2
        s[1][1] += white ** 2
        s_inv = _inverse_2x2(s)
        d2 = sum(y[i] * s_inv[i][j] * y[j] for i in range(2) for j in range(2))
        return y, s, h, d2

    # --- Initialisation ------------------------------------------------

    def _initialise(self, now, east, north, white, map_pose):
        """Start the filter from a fix plus a heading (IMU, or travel direction)."""
        self._last_map_pose = map_pose
        self._last_time = now
        if self._heading_yaw is not None:
            self._start(now, east, north, map_pose, self._heading_yaw, white,
                        self.config.imu_yaw_sigma)
            return FixUpdate(FixResult.INITIALISED)
        if self._first_fix is None:
            self._first_fix = (east, north, map_pose, white)
            return FixUpdate(FixResult.WAITING)
        e0, n0, p0, w0 = self._first_fix
        baseline = math.hypot(map_pose.x - p0.x, map_pose.y - p0.y)
        if baseline < self.config.init_baseline:
            return FixUpdate(FixResult.WAITING)
        # Direction of travel in ENU minus direction of travel in the map frame. Both
        # fixes share almost the same bias, so only the white noise blurs the heading.
        yaw = wrap_angle(math.atan2(north - n0, east - e0)
                         - math.atan2(map_pose.y - p0.y, map_pose.x - p0.x))
        yaw_sigma = 2.0 * math.hypot(white, w0) / baseline
        self._start(now, east, north, map_pose, yaw, white, yaw_sigma)
        return FixUpdate(FixResult.INITIALISED)

    def _start(self, now, east, north, map_pose, yaw, white, yaw_sigma):
        """Set T so the robot is at the fix (bias 0), with matching covariance."""
        c, s = math.cos(yaw), math.sin(yaw)
        px, py = map_pose.x, map_pose.y
        self.transform = Pose2D(east - (c * px - s * py), north - (s * px + c * py), yaw)
        self.bias = (0.0, 0.0)
        # The fix = origin + R(yaw) p + bias + noise, so the origin's error is the
        # bias and noise (minus) plus the yaw error times the lever arm j:
        # d(origin)/d(yaw) = j.
        j = [s * px + c * py, -c * px + s * py]
        v = yaw_sigma ** 2
        b = self._bias_sigma ** 2
        w = white ** 2
        p = _zeros(N, N)
        p[X][X] = w + b + j[0] * j[0] * v
        p[Y][Y] = w + b + j[1] * j[1] * v
        p[X][Y] = p[Y][X] = j[0] * j[1] * v
        p[X][YAW] = p[YAW][X] = j[0] * v
        p[Y][YAW] = p[YAW][Y] = j[1] * v
        p[YAW][YAW] = v
        p[BE][BE] = p[BN][BN] = b
        p[X][BE] = p[BE][X] = -b
        p[Y][BN] = p[BN][Y] = -b
        self.covariance = p
        self.last_fix_time = now
        self._last_fused_time = now
        self.distance_since_fix = 0.0
        self._first_fix = None
