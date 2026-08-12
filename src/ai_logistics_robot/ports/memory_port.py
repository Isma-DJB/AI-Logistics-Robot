"""Public memory contract for mission reconstruction."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.domain.enums import PathPhase
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.domain.paths import PathRecord


@runtime_checkable
class MemoryPort(Protocol):
    """Store confirmed mission state independently of its backend."""

    def start(self, mission: Mission) -> None:
        """Start recording one mission."""

        ...

    def record_pose(
        self,
        phase: PathPhase,
        pose: RobotPose,
    ) -> None:
        """Record one confirmed pose for a navigation phase."""

        ...

    def record_event(self, event: MissionEvent) -> None:
        """Record one ordered mission event."""

        ...

    def build_return_path(self) -> PathRecord:
        """Build the confirmed path record used for return navigation."""

        ...

    def complete(self, mission: Mission) -> None:
        """Store the final immutable mission state."""

        ...

    def reset(self) -> None:
        """Clear the active mission recording state."""

        ...