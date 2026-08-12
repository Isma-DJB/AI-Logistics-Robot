"""Public renderer contract for effect-free visualization."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap


@runtime_checkable
class RendererPort(Protocol):
    """Visualize read-only state without affecting robot control."""

    def render(
        self,
        world: GridMap,
        status: SystemStatus,
    ) -> None:
        """Render one immutable world and system-status snapshot."""

        ...

    def display_event(self, event: MissionEvent) -> None:
        """Display one normalized mission event."""

        ...