"""Public perception contract for normalized observations."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.domain.perception import PerceptionSnapshot


@runtime_checkable
class PerceptionPort(Protocol):
    """Produce consistent platform-independent perception snapshots."""

    def observe(self) -> PerceptionSnapshot:
        """Return one immutable, timestamped perception snapshot."""

        ...