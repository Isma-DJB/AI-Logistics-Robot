"""Public clock contract for deterministic execution."""

from datetime import datetime
from typing import Protocol, runtime_checkable


@runtime_checkable
class ClockPort(Protocol):
    """Make real time, simulated time, and tests interchangeable."""

    def now(self) -> datetime:
        """Return the current timezone-aware wall-clock time."""

        ...

    def monotonic(self) -> float:
        """Return the current monotonic time in seconds."""

        ...

    def wait_until(self, deadline: float) -> None:
        """Wait until a monotonic deadline is reached."""

        ...