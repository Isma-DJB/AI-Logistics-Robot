"""Deterministic clock backed by confirmed GridWorld time."""

from datetime import datetime, timedelta
from math import isfinite

from ai_logistics_robot.adapters.simulation.grid_world import GridWorld
from ai_logistics_robot.domain.errors import DomainValidationError


class SimulatedClock:
    """Expose GridWorld time through the public clock contract."""

    __slots__ = (
        "_epoch",
        "_simulation",
    )

    def __init__(
        self,
        *,
        simulation: GridWorld,
        epoch: datetime,
    ) -> None:
        """Validate and retain one simulation and wall-clock epoch."""

        if not isinstance(simulation, GridWorld):
            raise DomainValidationError(
                "simulation must be a GridWorld instance."
            )

        if (
            not isinstance(epoch, datetime)
            or epoch.tzinfo is None
            or epoch.utcoffset() is None
        ):
            raise DomainValidationError(
                "epoch must be a timezone-aware datetime."
            )

        self._simulation = simulation
        self._epoch = epoch

    def now(self) -> datetime:
        """Return wall time derived from confirmed simulation time."""

        return self._epoch + timedelta(
            seconds=self.monotonic()
        )

    def monotonic(self) -> float:
        """Return the confirmed GridWorld elapsed time."""

        return self._simulation.elapsed_time_seconds

    def wait_until(self, deadline: float) -> None:
        """Advance GridWorld to one finite non-past deadline."""

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
                "deadline must not precede current monotonic time."
            )

        try:
            self._epoch + timedelta(
                seconds=normalized_deadline
            )
        except OverflowError as error:
            raise DomainValidationError(
                "deadline exceeds the supported datetime range."
            ) from error

        self._simulation.advance_time(
            normalized_deadline - current_time
        )
