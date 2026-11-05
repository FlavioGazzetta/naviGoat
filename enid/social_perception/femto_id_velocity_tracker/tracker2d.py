"""Camera-independent multi-person tracker: constant-velocity Kalman + assignment.

Inputs must be (x, y) metres in a CONSISTENT Cartesian reference frame.
For a moving robot, transform detections into odom before calling update().
This module requires only numpy and scipy; no ROS, YOLO or camera imports.
"""

from dataclasses import dataclass
from typing import Sequence
import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class TrackView:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    hits: int
    age_since_seen_s: float
    observed: bool  # True only if a detection matched on this update
    confirmed: bool


class _KalmanTrack:
    H = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.]])
    I = np.eye(4)

    def __init__(self, identifier: int, position: np.ndarray, stamp: float):
        self.id = identifier
        self.state = np.array([position[0], position[1], 0., 0.], dtype=float)
        # Large initial velocity uncertainty because one frame cannot give velocity.
        self.cov = np.diag([0.15**2, 0.15**2, 1.5**2, 1.5**2])
        self.timestamp = float(stamp)
        self.last_seen = float(stamp)
        self.hits = 1
        self.observed = True

    def predict(self, stamp: float, accel_std: float):
        dt = float(stamp - self.timestamp)
        if dt <= 0:
            return
        f = np.eye(4)
        f[0, 2] = dt
        f[1, 3] = dt
        g = np.array([[0.5 * dt * dt, 0.],
                      [0., 0.5 * dt * dt],
                      [dt, 0.],
                      [0., dt]])
        q = (accel_std ** 2) * (g @ g.T)
        self.state = f @ self.state
        self.cov = f @ self.cov @ f.T + q
        self.timestamp = float(stamp)
        self.observed = False

    def correct(self, measurement: np.ndarray, stamp: float, position_std: float):
        r = np.eye(2) * position_std**2
        innovation = measurement - self.H @ self.state
        s = self.H @ self.cov @ self.H.T + r
        k = np.linalg.solve(s, self.H @ self.cov).T
        self.state = self.state + k @ innovation
        # Joseph form maintains numerical stability/positive semidefiniteness.
        a = self.I - k @ self.H
        self.cov = a @ self.cov @ a.T + k @ r @ k.T
        self.last_seen = float(stamp)
        self.hits += 1
        self.observed = True


class Tracker2D:
    def __init__(self, gate_m=1.0, max_age_s=1.0, min_hits=2,
                 measurement_std_m=0.12, acceleration_std_mps2=1.8):
        if gate_m <= 0 or max_age_s <= 0 or min_hits < 1:
            raise ValueError("gate_m/max_age_s must be positive; min_hits >= 1")
        self.gate_m = float(gate_m)
        self.max_age_s = float(max_age_s)
        self.min_hits = int(min_hits)
        self.measurement_std_m = float(measurement_std_m)
        self.acceleration_std_mps2 = float(acceleration_std_mps2)
        self.tracks = []
        self.next_id = 1
        self.last_stamp = None

    def update(self, positions: Sequence[Sequence[float]], stamp: float):
        """Return (IDs per detection, snapshot). Uses SENSOR time in seconds.

        New detections get a tentative ID immediately. Tracks become confirmed
        after min_hits observations. Missing tracks are predicted for max_age_s.
        """
        stamp = float(stamp)
        measurements = np.asarray(positions, dtype=float).reshape(-1, 2)
        if not np.isfinite(stamp) or not np.all(np.isfinite(measurements)):
            raise ValueError("Non-finite measurements/timestamp")
        if self.last_stamp is not None and stamp <= self.last_stamp:
            # Camera messages occasionally arrive out of order. Do not corrupt state.
            return [None] * len(measurements), self.snapshot(self.last_stamp)
        self.last_stamp = stamp

        # Expire tracks BEFORE association. Avoid resurrecting very old IDs.
        self.tracks = [t for t in self.tracks
                       if stamp - t.last_seen <= self.max_age_s]
        for track in self.tracks:
            track.predict(stamp, self.acceleration_std_mps2)

        ids = [None] * len(measurements)
        matched_detections = set()
        if self.tracks and len(measurements):
            predicted = np.stack([t.state[:2] for t in self.tracks])
            cost = np.linalg.norm(
                predicted[:, None, :] - measurements[None, :, :], axis=2
            )
            # Hungarian assignment WITH explicit unmatched columns: matches outside
            # the distance gate are impossible, rather than accepted then discarded.
            num_t = len(self.tracks)
            dummy_penalty = self.gate_m + 0.001
            augmented = np.full((num_t, len(measurements) + num_t),
                                dummy_penalty, dtype=float)
            augmented[:, :len(measurements)] = np.where(
                cost <= self.gate_m, cost, 1e6
            )
            rows, cols = linear_sum_assignment(augmented)
            for row, col in zip(rows, cols):
                if col >= len(measurements) or cost[row, col] > self.gate_m:
                    continue
                track = self.tracks[row]
                track.correct(measurements[col], stamp, self.measurement_std_m)
                ids[col] = track.id
                matched_detections.add(col)

        for idx, position in enumerate(measurements):
            if idx not in matched_detections:
                track = _KalmanTrack(self.next_id, position, stamp)
                self.tracks.append(track)
                ids[idx] = track.id
                self.next_id += 1

        return ids, self.snapshot(stamp)

    def snapshot(self, stamp: float):
        return [TrackView(
            id=t.id, x=float(t.state[0]), y=float(t.state[1]),
            vx=float(t.state[2]), vy=float(t.state[3]),
            hits=t.hits, age_since_seen_s=max(0., float(stamp - t.last_seen)),
            observed=t.observed, confirmed=(t.hits >= self.min_hits)
        ) for t in self.tracks]
