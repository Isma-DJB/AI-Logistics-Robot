"""Public simulation contract for deterministic world execution."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.world import GridMap


@runtime_checkable
class SimulationPort(Protocol):
    """Expose a replaceable deterministic simulation platform."""

    def reset(self) -> None:
        """Restore the configured initial simulation state."""

        ...

    def read_world(self) -> GridMap:
        """Return the current immutable world snapshot."""

        ...

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Apply one command and return its confirmed outcome."""

        ...

    def advance_time(self, seconds: float) -> None:
        """Advance simulated time by the requested duration."""

        ...