"""Deterministic in-memory mission-event monitoring."""

from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.events import MissionEvent


class InMemoryMonitoring:
    """Publish immutable events in validated mission order."""

    __slots__ = (
        "_event_ids",
        "_events",
        "_last_sequences",
    )

    def __init__(self) -> None:
        """Initialize empty multi-mission monitoring history."""

        self._events: tuple[MissionEvent, ...] = ()
        self._event_ids: set[str] = set()
        self._last_sequences: dict[str, int] = {}

    def publish(
        self,
        event: MissionEvent,
    ) -> None:
        """Publish one unique event in increasing mission order."""

        if not isinstance(event, MissionEvent):
            raise DomainValidationError(
                "event must be a MissionEvent instance."
            )

        if event.event_id in self._event_ids:
            raise InvariantViolationError(
                "event_id must be unique across monitoring history."
            )

        previous_sequence = self._last_sequences.get(
            event.mission_id
        )

        if (
            previous_sequence is not None
            and event.sequence_number <= previous_sequence
        ):
            raise InvariantViolationError(
                "event sequence numbers must increase "
                "within each mission."
            )

        self._events = (*self._events, event)
        self._event_ids.add(event.event_id)
        self._last_sequences[event.mission_id] = (
            event.sequence_number
        )

    def events_for(
        self,
        mission_id: str,
    ) -> tuple[MissionEvent, ...]:
        """Return one mission's events in publication order."""

        if (
            not isinstance(mission_id, str)
            or not mission_id.strip()
        ):
            raise DomainValidationError(
                "mission_id must be a non-empty string."
            )

        return tuple(
            event
            for event in self._events
            if event.mission_id == mission_id
        )
