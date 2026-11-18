"""Plain-Python following lifecycle; no ROS, camera or wall-clock dependencies.

Call enable() explicitly, then update() regularly even when no frames arrive.
Pass a PerceptionFrame for each NEW perception result; an empty people tuple
means a working camera saw nobody. frame=None means no new result has arrived.
All lifecycle timestamps use the same monotonic clock, in seconds. The perception
adapter converts sensor timestamps into this domain and keeps source capture
time separately for the ROS node's transform lookups.

Predicted tracks do not count as sightings. ID continuity is trusted only for
the short loss window; callers must provide reliable association. This module
cannot establish a person's identity from their position or track ID alone.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class FollowingState(str, Enum):
    IDLE = "IDLE"
    ACQUIRING = "ACQUIRING"
    FOLLOWING = "FOLLOWING"
    TEMPORARILY_LOST = "TEMPORARILY_LOST"


@dataclass(frozen=True)
class PersonObservation:
    track_id: int
    forward_m: float
    lateral_m: float
    observed: bool = True
    confirmed: bool = True


@dataclass(frozen=True)
class PerceptionFrame:
    timestamp_s: float
    people: tuple[PersonObservation, ...] = ()
    source_timestamp_s: float | None = None  # original ROS capture time for exact-time TF


@dataclass(frozen=True)
class FollowingConfig:
    loss_timeout_s: float = 2.0
    input_timeout_s: float = 0.5

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class FollowingOutput:
    state: FollowingState
    target_id: int | None
    # One-shot machine-readable event published through the mode bridge.
    event: str | None = None


class FollowingStateMachine:
    def __init__(self, config: FollowingConfig | None = None):
        self.config = config if config is not None else FollowingConfig()
        self._reset()
        self._last_tick = None

    def _reset(self):
        self.state = FollowingState.IDLE
        self.target_id = None
        self._last_frame = None
        self._last_seen = None

    def _output(self, event=None):
        return FollowingOutput(self.state, self.target_id, event)

    def _stop(self, reason):
        self._reset()
        return self._output(reason)

    def enable(self) -> FollowingOutput:
        """Start acquisition; repeated enable calls do not reset an active target."""
        if self.state != FollowingState.IDLE:
            return self._output()
        self._reset()
        self.state = FollowingState.ACQUIRING
        self._last_tick = None
        return self._output("enabled")

    def disable(self) -> FollowingOutput:
        """End the session; the ROS adapter must cancel its navigation goal."""
        return self._stop("disabled")

    def update(self, now_s: float, frame: PerceptionFrame | None = None,
               *, input_healthy: bool = True) -> FollowingOutput:
        """Advance one tracking tick and return state, target and optional event.

        Camera/depth failures should set input_healthy=False. Stale input also
        immediately exits. Missed detections retain the selected ID briefly for
        reacquisition. The adapter cancels navigation whenever not FOLLOWING;
        this module neither calculates velocities nor controls deceleration.
        """
        if (not math.isfinite(now_s) or
                (self._last_tick is not None and now_s < self._last_tick)):
            self._stop("input_fault")
            raise ValueError("Control time must be finite and monotonic")
        self._last_tick = now_s
        if self.state == FollowingState.IDLE:
            return self._output()
        if not input_healthy:
            return self._stop("input_fault")

        # Enforce the deadline BEFORE accepting a returning target. A late
        # reappearance must not silently reactivate a completed following mode.
        if (self._last_seen is not None and
                now_s - self._last_seen >= self.config.loss_timeout_s):
            return self._stop("target_lost")

        event = None
        if frame is not None:
            if not math.isfinite(frame.timestamp_s) or frame.timestamp_s > now_s:
                return self._stop("input_fault")
            # Repeated/out-of-order frames cannot refresh the watchdog.
            is_new = self._last_frame is None or frame.timestamp_s > self._last_frame
            if is_new:
                if now_s - frame.timestamp_s > self.config.input_timeout_s:
                    return self._stop("input_stale")
                self._last_frame = frame.timestamp_s
                observed = [p for p in frame.people if p.observed]
                ids = [p.track_id for p in observed]
                if (any(not isinstance(i, int) or isinstance(i, bool) or i < 1 for i in ids)
                        or len(set(ids)) != len(ids)):
                    return self._stop("input_fault")
                for person in observed:
                    if (not math.isfinite(person.forward_m) or
                            not math.isfinite(person.lateral_m) or person.forward_m <= 0):
                        return self._stop("input_fault")
                candidates = [p for p in observed if p.confirmed]
                target = next((p for p in candidates if p.track_id == self.target_id), None)
                if self.state == FollowingState.ACQUIRING and candidates:
                    # Choose once at acquisition, then use only the selected ID.
                    # Horizontal camera range includes sideways displacement;
                    # use ID as a deterministic tie-break for equal distances.
                    target = min(candidates, key=lambda p: (
                        math.hypot(p.forward_m, p.lateral_m), p.track_id))
                    self.target_id = target.track_id
                    event = "target_acquired"
                if target is not None:
                    if self.state == FollowingState.TEMPORARILY_LOST:
                        event = "target_reacquired"
                    self.state = FollowingState.FOLLOWING
                    self._last_seen = frame.timestamp_s
                elif self.state == FollowingState.FOLLOWING:
                    self.state = FollowingState.TEMPORARILY_LOST
                    event = "target_missing"

        if (self._last_frame is not None and
                now_s - self._last_frame > self.config.input_timeout_s):
            return self._stop("input_stale")

        return self._output(event)
