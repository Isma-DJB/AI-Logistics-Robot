"""Real UTC and monotonic clock for physical robot execution."""

from collections.abc import Callable
from datetime import UTC, datetime
from math import isfinite
from time import monotonic as system_monotonic
from time import sleep as system_sleep

from ai_logistics_robot.domain.errors import DomainValidationError

WallClockProvider = Callable[[], datetime]
MonotonicProvider = Callable[[], float]
SleepProvider = Callable[[float], None]


def _utc_now() -> datetime:
    """Return the current timezone-aware UTC datetime."""

    return datetime.now(UTC)


class SystemClock:
    """Expose real system time through the public clock contract."""

    __slots__ = (
        "_monotonic_provider",
        "_sleep_provider",
        "_wall_clock_provider",
    )

    def __init__(
        self,
        *,
        wall_clock_provider: WallClockProvider = _utc_now,
        monotonic_provider: MonotonicProvider = (
            system_monotonic
        ),
        sleep_provider: SleepProvider = system_sleep,
    ) -> None:
        """Validate and retain replaceable system-time providers."""

        if not callable(wall_clock_provider):
            raise DomainValidationError(
                "wall_clock_provider must be callable."
            )

        if not callable(monotonic_provider):
            raise DomainValidationError(
                "monotonic_provider must be callable."
            )

        if not callable(sleep_provider):
            raise DomainValidationError(
                "sleep_provider must be callable."
            )

        self._wall_clock_provider = wall_clock_provider
        self._monotonic_provider = monotonic_provider
        self._sleep_provider = sleep_provider

    def now(self) -> datetime:
        """Return one validated timezone-aware wall-clock instant."""

        current_datetime = self._wall_clock_provider()

        if (
            not isinstance(current_datetime, datetime)
            or current_datetime.tzinfo is None
            or current_datetime.utcoffset() is None
        ):
            raise DomainValidationError(
                "wall_clock_provider must return a "
                "timezone-aware datetime."
            )

        return current_datetime

    def monotonic(self) -> float:
        """Return one validated real monotonic-clock value."""

        current_value = self._monotonic_provider()

        if (
            isinstance(current_value, bool)
            or not isinstance(current_value, (int, float))
            or not isfinite(float(current_value))
            or float(current_value) < 0.0
        ):
            raise DomainValidationError(
                "monotonic_provider must return a finite "
                "non-negative number."
            )

        return float(current_value)

    def wait_until(self, deadline: float) -> None:
        """Block until one finite non-past monotonic deadline."""

        if (
            isinstance(deadline, bool)
            or not isinstance(deadline, (int, float))
            or not isfinite(float(deadline))
        ):
            raise DomainValidationError(
                "deadline must be a finite number."
            )

        normalized_deadline = float(deadline)
        current_time = self.monotonic()

        if normalized_deadline < current_time:
            raise DomainValidationError(
                "deadline must not precede current "
                "monotonic time."
            )

        self._sleep_provider(
            normalized_deadline - current_time
        )