"""Testable asynchronous NavigateToPose lifecycle; transport supplied by ROS.

Only goals sent by this client are cancelled. New destinations preempt previous
ones through Nav2. Stops invalidate pending sends; late acceptances are cancelled
as soon as their handles arrive. A new session waits for old cancellations.
"""

from dataclasses import dataclass
import math


@dataclass
class _Request:
    token: int
    generation: int
    session: str
    deadline: float
    handle: object = None
    cancelling: bool = False
    failed: bool = False


class Nav2GoalClient:
    def __init__(self, transport, make_message, timeout_s=2.0, min_interval_s=0.0):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("Action timeout must be finite and positive")
        if not math.isfinite(min_interval_s) or min_interval_s < 0:
            raise ValueError("Goal interval must be finite and non-negative")
        self.transport = transport
        self.make_message = make_message
        self.timeout_s = timeout_s
        self.min_interval_s = min_interval_s
        self._requests = {}
        self._generation = 0
        self._serial = 0
        self._latest = None
        self._desired = None
        self._waiting_since = None
        self._last_dispatch = None
        self._session = None
        self._now = 0.0
        self._failed = False
        self.events = []  # (session, event); consumed by the owning control loop

    @property
    def busy(self):
        return bool(self._requests)

    def request_goal(self, goal, session, now_s):
        if session != self._session:
            self.cancel(now_s)
            self._session, self._failed = session, False
        if not self._failed:
            if self._desired is None:
                self._waiting_since = now_s
            self._desired = goal
            self.tick(now_s)

    def cancel(self, now_s):
        self._now = now_s
        self._desired = None
        self._waiting_since = None
        self._generation += 1
        for record in list(self._requests.values()):
            self._cancel_record(record)

    def _fail(self, event, session):
        if not self._failed:
            # Failure to stop an old session also prevents the current one from
            # starting, so notify the session which is now waiting for control.
            self.events.append((self._session or session, event))
            self._failed = True
        self.cancel(self._now)

    def tick(self, now_s):
        self._now = now_s
        for record in list(self._requests.values()):
            if now_s > record.deadline and not record.failed:
                # Current active goals have no deadline after acceptance.
                record.failed = True
                event = "navigation_cancel_failed" if record.cancelling else "navigation_failed"
                self._fail(event, record.session)
                return
        if self._desired is None or self._failed:
            return
        # Never start over while a stop/old session is still being resolved.
        if any(r.generation != self._generation or r.cancelling or r.handle is None
               for r in self._requests.values()):
            if now_s - self._waiting_since > self.timeout_s:
                self._fail("navigation_cancel_failed", self._session)
            return
        if self._last_dispatch is not None and now_s - self._last_dispatch < self.min_interval_s:
            return
        if not self.transport.server_is_ready():
            self._fail("navigation_unavailable", self._session)
            return
        self._serial += 1
        record = _Request(self._serial, self._generation, self._session, now_s + self.timeout_s)
        self._latest = record.token
        self._requests[record.token] = record
        goal, self._desired = self._desired, None
        self._waiting_since = None
        self._last_dispatch = now_s
        try:
            future = self.transport.send_goal_async(self.make_message(goal))
            future.add_done_callback(lambda result: self._accepted(record, result))
        except Exception:
            self._requests.pop(record.token, None)
            self._fail("navigation_failed", record.session)

    def _accepted(self, record, future):
        try:
            handle = future.result()
        except Exception:
            self._requests.pop(record.token, None)
            if record.generation == self._generation:
                self._fail("navigation_failed", record.session)
            return
        if not handle.accepted:
            self._requests.pop(record.token, None)
            if record.generation == self._generation:
                self._fail("navigation_failed", record.session)
            return
        record.handle = handle
        record.deadline = float("inf")
        try:
            handle.get_result_async().add_done_callback(lambda result: self._finished(record, result))
        except Exception:
            self._fail("navigation_failed", record.session)
            return
        if record.generation != self._generation:
            self._cancel_record(record)
        else:
            # Nav2 preempts older goals; bound how long their results may linger.
            for old in self._requests.values():
                if old.token != record.token and not old.cancelling:
                    old.deadline = self._now + self.timeout_s

    def _finished(self, record, future):
        if self._requests.pop(record.token, None) is None:
            return
        try:
            status = future.result().status
        except Exception:
            status = None
        if record.generation != self._generation or record.token != self._latest:
            return  # stopped/preempted goals must not end a newer request
        if status == 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            self.events.append((record.session, "goal_reached"))
        else:
            self._fail("navigation_failed", record.session)

    def _cancel_record(self, record):
        if record.handle is None or record.cancelling:
            return
        record.cancelling = True
        record.deadline = self._now + self.timeout_s
        try:
            future = record.handle.cancel_goal_async()
            future.add_done_callback(lambda response: self._cancelled(record, response))
        except Exception:
            record.failed = True
            self._fail("navigation_cancel_failed", record.session)

    def _cancelled(self, record, future):
        if record.token not in self._requests:
            return
        try:
            response = future.result()
            accepted = response.return_code == 0 and bool(response.goals_canceling)
        except Exception:
            accepted = False
        if not accepted:
            if record.handle.status in (4, 5, 6):  # already terminal
                self._requests.pop(record.token, None)
            else:
                record.failed = True
                self._fail("navigation_cancel_failed", record.session)
        # An accepted cancellation still waits for its terminal action result.
