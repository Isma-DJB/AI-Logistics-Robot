"""Passive in-memory rendering for headless execution."""

from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap

RenderedFrame = tuple[GridMap, SystemStatus]


class HeadlessRenderer:
    """Retain immutable display inputs without graphical effects."""

    __slots__ = (
        "_displayed_events",
        "_rendered_frames",
    )

    def __init__(self) -> None:
        """Initialize empty passive rendering history."""

        self._rendered_frames: tuple[RenderedFrame, ...] = ()
        self._displayed_events: tuple[MissionEvent, ...] = ()

    @property
    def rendered_frames(self) -> tuple[RenderedFrame, ...]:
        """Return immutable rendered world and status snapshots."""

        return self._rendered_frames

    @property
    def displayed_events(self) -> tuple[MissionEvent, ...]:
        """Return immutable displayed event history."""

        return self._displayed_events

    def render(
        self,
        world: GridMap,
        status: SystemStatus,
    ) -> None:
        """Retain one validated passive display frame."""

        if not isinstance(world, GridMap):
            raise DomainValidationError(
                "world must be a GridMap instance."
            )

        if not isinstance(status, SystemStatus):
            raise DomainValidationError(
                "status must be a SystemStatus instance."
            )

        self._rendered_frames = (
            *self._rendered_frames,
            (world, status),
        )

    def display_event(
        self,
        event: MissionEvent,
    ) -> None:
        """Retain one validated event without interpreting it."""

        if not isinstance(event, MissionEvent):
            raise DomainValidationError(
                "event must be a MissionEvent instance."
            )

        self._displayed_events = (
            *self._displayed_events,
            event,
        )
