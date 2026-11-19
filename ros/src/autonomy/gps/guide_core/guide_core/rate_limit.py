"""Request budget for the public Nominatim server.

The usage policy (https://operations.osmfoundation.org/policies/nominatim/)
allows at most 1 request per second for the whole application and can
block clients that misbehave. RateLimiter keeps us well inside that:

- at least min_interval seconds (1.1 s) between requests: the caller waits;
- at most max_per_hour requests in any hour: a safety net so a bug that
  loops cannot hammer the server; further searches are refused until the
  hour has passed;
- after the server refuses us (HTTP 403 or 429), no requests at all for
  block_cooldown seconds (1 hour). There are never automatic retries.

check() only looks; record_request() / record_refused() change the state.
Times are seconds from a monotonic clock (time.monotonic() in the node), so
changing the system clock cannot break the limits. No ROS.
"""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RateLimitConfig:
    """Limits (see the module docstring)."""

    min_interval: float = 1.1
    max_per_hour: int = 30
    block_cooldown: float = 3600.0

    def __post_init__(self):
        if not (math.isfinite(self.min_interval) and self.min_interval >= 1.0):
            raise ValueError(f'min_interval must be >= 1.0 s (policy), got {self.min_interval}')
        if isinstance(self.max_per_hour, bool) or not isinstance(self.max_per_hour, int) \
                or self.max_per_hour < 1:
            raise ValueError(f'max_per_hour must be an integer >= 1, got {self.max_per_hour}')
        if not (math.isfinite(self.block_cooldown) and self.block_cooldown >= 0.0):
            raise ValueError(f'block_cooldown must be >= 0, got {self.block_cooldown}')


@dataclass(frozen=True)
class Decision:
    """What check() allows.

    send: True -> wait 'wait' seconds (0 <= wait <= min_interval), then send.
    send: False -> do not send; 'reason' says why (for logs and the user).
    """

    send: bool
    wait: float = 0.0
    reason: str = ''


HOUR = 3600.0


class RateLimiter:
    """Keeps track of requests sent and refusals received."""

    def __init__(self, config=None):
        self.config = config or RateLimitConfig()
        self._sent = deque()          # monotonic times of requests in the last hour
        self._blocked_until = None

    def check(self, now):
        """Decide whether a request may be sent at time now, and after how long a wait."""
        if self._blocked_until is not None and now < self._blocked_until:
            minutes = math.ceil((self._blocked_until - now) / 60.0)
            return Decision(False, reason='Nominatim refused our requests; online search '
                                          f'is paused for {minutes} more min')
        recent = [t for t in self._sent if t > now - HOUR]
        if len(recent) >= self.config.max_per_hour:
            minutes = math.ceil((recent[0] + HOUR - now) / 60.0)
            return Decision(False, reason=f'limit of {self.config.max_per_hour} online '
                                          f'searches per hour reached; try again in {minutes} min')
        if not self._sent:
            return Decision(True)
        wait = self._sent[-1] + self.config.min_interval - now
        return Decision(True, wait=min(max(0.0, wait), self.config.min_interval))

    def record_request(self, now):
        """Note that a request is being sent (call it right before sending)."""
        self._sent.append(now)
        while self._sent and self._sent[0] <= now - HOUR:
            self._sent.popleft()

    def record_refused(self, now):
        """Note that the server answered 403 or 429: stop for block_cooldown seconds."""
        self._blocked_until = now + self.config.block_cooldown
