"""Immutable, ordered mission-event objects."""

from dataclasses import dataclass
from datetime import datetime
from math import isfinite

from ai_logistics_robot.domain.enums import BrainState
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)

EventValue = str | int | float | bool | None


def _validate_text(name: str, value: object) -> None:
    """Require a non-empty string."""

    if not isinstance(value, str) or not value.strip():
        raise DomainValidationError(
            f"{name} must be a non-empty string."
        )


def _validate_timestamp(value: object) -> None:
    """Require a timezone-aware datetime."""

    if (
        not isinstance(value, datetime)
        or value.tzinfo is None
        or value.utcoffset() is None
    ):
        raise DomainValidationError(
            "occurred_at must be a timezone-aware datetime."
        )


@dataclass(frozen=True, slots=True)
class EventDetail:
    """One immutable, serializable event-detail field."""

    name: str
    value: EventValue

    def __post_init__(self) -> None:
        """Validate the event-detail contract."""

        _validate_text("name", self.name)

        if not isinstance(
            self.value,
            (str, int, float, bool, type(None)),
        ):
            raise DomainValidationError(
                "event detail values must be scalar or None."
            )

        if isinstance(self.value, float) and not isfinite(self.value):
            raise DomainValidationError(
                "floating event detail values must be finite."
            )


@dataclass(frozen=True, slots=True)
class MissionEvent:
    """One ordered event used to reconstruct a mission."""

    event_id: str
    sequence_number: int
    mission_id: str
    robot_id: str
    occurred_at: datetime
    source: str
    name: str
    brain_state: BrainState
    details: tuple[EventDetail, ...] = ()

    def __post_init__(self) -> None:
        """Validate ordering, identity, and immutable details."""

        _validate_text("event_id", self.event_id)
        _validate_text("mission_id", self.mission_id)
        _validate_text("robot_id", self.robot_id)
        _validate_text("source", self.source)
        _validate_text("name", self.name)
        _validate_timestamp(self.occurred_at)

        if (
            isinstance(self.sequence_number, bool)
            or not isinstance(self.sequence_number, int)
            or self.sequence_number < 1
        ):
            raise DomainValidationError(
                "sequence_number must be a positive integer."
            )

        if not isinstance(self.brain_state, BrainState):
            raise DomainValidationError(
                "brain_state must be a BrainState instance."
            )

        if not isinstance(self.details, tuple):
            raise DomainValidationError(
                "details must be an immutable tuple."
            )

        if not all(
            isinstance(detail, EventDetail)
            for detail in self.details
        ):
            raise DomainValidationError(
                "every detail must be an EventDetail instance."
            )

        names = tuple(detail.name for detail in self.details)

        if len(names) != len(set(names)):
            raise InvariantViolationError(
                "event detail names must be unique."
            )

    def detail_value(self, name: str) -> EventValue:
        """Return a detail value by name."""

        _validate_text("name", name)

        for detail in self.details:
            if detail.name == name:
                return detail.value

        raise KeyError(name)