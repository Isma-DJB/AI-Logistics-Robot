"""Public brain contract for deterministic orchestration cycles."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.domain.status import SystemStatus


@runtime_checkable
class BrainPort(Protocol):
    """Advance and inspect the platform-independent robot brain."""

    def update(self) -> None:
        """Perform one deterministic orchestration cycle."""

        ...

    def get_status(self) -> SystemStatus:
        """Return the current read-only system status."""

        ...

    def reset(self) -> None:
        """Restore the configured initial brain state."""

        ...