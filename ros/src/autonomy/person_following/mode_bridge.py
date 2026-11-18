"""Translate middleman JSON commands and correlate following status; no ROS.

The middleman supplies a unique request_id on each mode command. Echo that ID
in status so a late failure cannot end a newer following request.
"""

from __future__ import annotations

from dataclasses import replace
import json

from state_machine import FollowingOutput, FollowingState, FollowingStateMachine


class ModeCommandError(ValueError):
    pass


class FollowingModeBridge:
    def __init__(self, machine: FollowingStateMachine):
        self.machine = machine
        self.request_id = None
        self._seen_follow_requests = set()

    def _snapshot(self):
        return FollowingOutput(self.machine.state, self.machine.target_id)

    def handle_command(self, text: str) -> FollowingOutput:
        try:
            payload = json.loads(text)
        except (ValueError, TypeError, RecursionError) as exc:
            raise ModeCommandError("Mode command must be valid JSON") from exc
        if not isinstance(payload, dict):
            raise ModeCommandError("Mode command must be an object")
        mode = payload.get("mode")
        stop = payload.get("stop")
        if not isinstance(mode, str) or not isinstance(stop, bool):
            raise ModeCommandError("mode must be text and stop must be boolean")
        if stop or mode != "follow":
            self.request_id = None
            return self.machine.disable()
        target = payload.get("target")
        request_id = payload.get("request_id")
        if not isinstance(target, str) or not isinstance(request_id, str) or not request_id:
            raise ModeCommandError("Follow needs a text target and non-empty request_id")
        if request_id in self._seen_follow_requests:
            # A repeated command must not restart a session after timeout.
            return self._snapshot()
        self._seen_follow_requests.add(request_id)
        self.request_id = request_id
        if target.strip():
            return replace(self.machine.disable(), event="unsupported_target")
        # Repeating 'follow' while active keeps the selected person. A stopped
        # session acquires anew only after a genuinely new command.
        return self.machine.enable()

    def reject_command(self) -> FollowingOutput:
        # Preserve the active ID so the middleman can correlate this failure.
        return replace(self.machine.disable(), event="command_invalid")

    def set_enabled(self, enabled: bool) -> FollowingOutput:
        """Manual enable service; new manual sessions have no voice ID."""
        if enabled:
            if self.machine.state == FollowingState.IDLE:
                self.request_id = None
            return self.machine.enable()
        return self.machine.disable()

    def status_payload(self, output: FollowingOutput) -> dict:
        return {"request_id": self.request_id, "state": output.state.value,
                "event": output.event, "target_id": output.target_id}
