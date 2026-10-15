"""Injectable clocks so timing-dependent safety logic is testable without sleeping."""

from __future__ import annotations

import time
from typing import Protocol


class Clock(Protocol):
    def now_ms(self) -> int: ...

    def sleep_ms(self, ms: int) -> None: ...


class SystemClock:
    """Monotonic wall clock for real runs."""

    def now_ms(self) -> int:
        return int(time.monotonic() * 1000)

    def sleep_ms(self, ms: int) -> None:
        time.sleep(ms / 1000.0)


class Deadline:
    """Timeout helper. Boundary is inclusive: expired when elapsed >= timeout_ms.

    The Arduino watchdog must use the same rule, with unsigned subtraction so that
    millis() rollover is safe: ``(uint32_t)(millis() - start) >= timeout``.
    """

    def __init__(self, clock: Clock, timeout_ms: int) -> None:
        if timeout_ms < 0:
            raise ValueError("timeout_ms must be >= 0")
        self._clock = clock
        self._timeout = int(timeout_ms)
        self._start = clock.now_ms()

    def reset(self) -> None:
        self._start = self._clock.now_ms()

    def elapsed_ms(self) -> int:
        return self._clock.now_ms() - self._start

    def remaining_ms(self) -> int:
        return max(0, self._timeout - self.elapsed_ms())

    def expired(self) -> bool:
        return self.elapsed_ms() >= self._timeout


class FakeClock:
    """Deterministic clock: time only moves when told to (or when something 'sleeps')."""

    def __init__(self, start_ms: int = 0) -> None:
        self._now = int(start_ms)

    def now_ms(self) -> int:
        return self._now

    def advance(self, ms: int) -> None:
        if ms < 0:
            raise ValueError("cannot move time backwards")
        self._now += int(ms)

    def sleep_ms(self, ms: int) -> None:
        self.advance(ms)
