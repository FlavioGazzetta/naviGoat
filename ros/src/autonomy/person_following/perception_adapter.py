"""Validate Enid's tracking JSON and translate sensor time to monotonic time.

No ROS imports. The caller supplies the current ROS/source clock and monotonic
clock sampled at receipt. Never substitute receipt time for capture time: that
would make delayed camera results look fresh.
"""

from __future__ import annotations

import json
import math

from state_machine import PerceptionFrame, PersonObservation


COORDINATE_CONVENTION = (
    "stationary_level_camera: x_forward_m,y_left_m; NOT odom-compensated"
)


class PerceptionError(ValueError):
    """Malformed, stale, or incompatible perception input."""


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PerceptionError(f"{name} must be a finite number")
    try:
        number = float(value)
    except OverflowError as exc:
        raise PerceptionError(f"{name} is too large") from exc
    if not math.isfinite(number):
        raise PerceptionError(f"{name} must be a finite number")
    return number


class PerceptionAdapter:
    def __init__(self, max_age_s=0.5):
        self.max_age_s = finite_number(max_age_s, "max_age_s")
        if self.max_age_s <= 0:
            raise PerceptionError("max_age_s must be positive")
        self._last_source_stamp = None
        self._last_monotonic_stamp = None

    def parse(self, text: str, *, source_now_s: float,
              monotonic_now_s: float) -> PerceptionFrame | None:
        """Return a new frame; return None for duplicate/out-of-order results.

        source_now_s must use the SAME clock as Enid's camera stamp_sec (normally
        the live ROS system clock). A future timestamp or stale frame raises
        PerceptionError. Predicted positions are preserved with observed=False.
        """
        source_now = finite_number(source_now_s, "source_now_s")
        monotonic_now = finite_number(monotonic_now_s, "monotonic_now_s")
        try:
            payload = json.loads(text)
        except (ValueError, TypeError, RecursionError) as exc:
            raise PerceptionError("Invalid tracking JSON") from exc
        if not isinstance(payload, dict):
            raise PerceptionError("Tracking payload must be an object")
        if payload.get("coordinate_convention") != COORDINATE_CONVENTION:
            raise PerceptionError("Expected Enid's stationary, level camera coordinates")
        stamp = finite_number(payload.get("stamp_sec"), "stamp_sec")
        if stamp < 0:
            raise PerceptionError("stamp_sec cannot be negative")
        age = source_now - stamp
        if age < 0:
            raise PerceptionError("Camera timestamp is in the future; check ROS clocks")
        if age > self.max_age_s:
            raise PerceptionError(f"Tracking result is stale ({age:.3f} s)")
        tracks = payload.get("tracks")
        if not isinstance(tracks, list):
            raise PerceptionError("tracks must be a list")
        people = []
        seen_ids = set()
        for track in tracks:
            if not isinstance(track, dict):
                raise PerceptionError("Each track must be an object")
            identifier = track.get("id")
            if (not isinstance(identifier, int) or isinstance(identifier, bool)
                    or identifier < 1 or identifier in seen_ids):
                raise PerceptionError("Track IDs must be positive, unique integers")
            seen_ids.add(identifier)
            observed = track.get("observed")
            confirmed = track.get("confirmed")
            if not isinstance(observed, bool) or not isinstance(confirmed, bool):
                raise PerceptionError("observed and confirmed must be booleans")
            age_since_seen = finite_number(track.get("age_since_seen_s"), "age_since_seen_s")
            if age_since_seen < 0 or (observed and age_since_seen != 0):
                raise PerceptionError("Track age is inconsistent with observed status")
            forward = finite_number(track.get("x"), "x")
            lateral = finite_number(track.get("y"), "y")
            if observed and forward <= 0:
                raise PerceptionError("Observed person must be in front of the camera")
            people.append(PersonObservation(identifier, forward, lateral, observed, confirmed))

        # Reject replay in source time even if the receiver's clock mapping moves.
        if self._last_source_stamp is not None and stamp <= self._last_source_stamp:
            return None
        mapped_stamp = monotonic_now - age
        if (self._last_monotonic_stamp is not None and
                mapped_stamp <= self._last_monotonic_stamp):
            raise PerceptionError("Clock mapping moved backwards; check clock synchronisation")
        self._last_source_stamp = stamp
        self._last_monotonic_stamp = mapped_stamp
        return PerceptionFrame(mapped_stamp, tuple(people), stamp)
