"""Public control contract for command execution and local safety."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import FailureReason
from ai_logistics_robot.domain.safety import SafetyStatus


@runtime_checkable
class ControlPort(Protocol):
    """Execute robot commands while owning the local safety latch."""

    def execute_step(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Execute one command and return its confirmed outcome."""

        ...

    def stop(self) -> None:
        """Request a normal controlled stop."""

        ...

    def emergency_stop(
        self,
        reason: FailureReason,
    ) -> SafetyStatus:
        """Latch the priority stop and return the confirmed status."""

        ...

    def get_safety_status(self) -> SafetyStatus:
        """Return the current confirmed safety status."""

        ...

    def reset_safety_latch(self) -> SafetyStatus:
        """Perform manual rearm and return the confirmed safe status."""

        ...