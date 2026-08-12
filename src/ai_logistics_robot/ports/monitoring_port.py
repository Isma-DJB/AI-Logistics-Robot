"""Public monitoring contract for ordered mission events."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.domain.events import MissionEvent


@runtime_checkable
class MonitoringPort(Protocol):
    """Publish and retrieve immutable ordered mission events."""

    def publish(self, event: MissionEvent) -> None:
        """Publish one normalized mission event."""

        ...

    def events_for(
        self,
        mission_id: str,
    ) -> tuple[MissionEvent, ...]:
        """Return the mission events in sequence-number order."""

        ...