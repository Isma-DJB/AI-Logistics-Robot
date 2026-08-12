"""Immutable safety-event and safety-status objects."""

from dataclasses import dataclass
from datetime import datetime

from ai_logistics_robot.domain.enums import (
    FailureReason,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)


def _validate_text(name: str, value: object) -> None:
    """Require a non-empty string."""

    if not isinstance(value, str) or not value.strip():
        raise DomainValidationError(
            f"{name} must be a non-empty string."
        )


def _validate_timestamp(name: str, value: object) -> None:
    """Require a timezone-aware datetime."""

    if (
        not isinstance(value, datetime)
        or value.tzinfo is None
        or value.utcoffset() is None
    ):
        raise DomainValidationError(
            f"{name} must be a timezone-aware datetime."
        )


@dataclass(frozen=True, slots=True)
class SafetyEvent:
    """One normalized event relevant to local robot safety."""

    event_id: str
    robot_id: str
    occurred_at: datetime
    severity: SafetySeverity
    source: str
    message: str
    reason: FailureReason | None = None

    def __post_init__(self) -> None:
        """Validate the safety-event contract."""

        _validate_text("event_id", self.event_id)
        _validate_text("robot_id", self.robot_id)
        _validate_timestamp("occurred_at", self.occurred_at)
        _validate_text("source", self.source)
        _validate_text("message", self.message)

        if not isinstance(self.severity, SafetySeverity):
            raise DomainValidationError(
                "severity must be a SafetySeverity instance."
            )

        if (
            self.reason is not None
            and not isinstance(self.reason, FailureReason)
        ):
            raise DomainValidationError(
                "reason must be a FailureReason or None."
            )

        if (
            self.severity is SafetySeverity.CRITICAL
            and self.reason is None
        ):
            raise InvariantViolationError(
                "a critical safety event requires an explicit reason."
            )

    @property
    def requires_stop(self) -> bool:
        """Report whether the event requires the priority stop chain."""

        return self.severity is SafetySeverity.CRITICAL


@dataclass(frozen=True, slots=True)
class SafetyStatus:
    """Confirmed local safety-latch state."""

    robot_id: str
    updated_at: datetime
    latched: bool
    severity: SafetySeverity
    reason: FailureReason | None = None

    def __post_init__(self) -> None:
        """Validate latch and rearm invariants."""

        _validate_text("robot_id", self.robot_id)
        _validate_timestamp("updated_at", self.updated_at)

        if not isinstance(self.latched, bool):
            raise DomainValidationError(
                "latched must be a boolean."
            )

        if not isinstance(self.severity, SafetySeverity):
            raise DomainValidationError(
                "severity must be a SafetySeverity instance."
            )

        if (
            self.reason is not None
            and not isinstance(self.reason, FailureReason)
        ):
            raise DomainValidationError(
                "reason must be a FailureReason or None."
            )

        if self.latched:
            if self.severity is not SafetySeverity.CRITICAL:
                raise InvariantViolationError(
                    "a latched safety status must be CRITICAL."
                )

            if self.reason is None:
                raise InvariantViolationError(
                    "a latched safety status requires a reason."
                )

        else:
            if self.severity is not SafetySeverity.INFO:
                raise InvariantViolationError(
                    "an unlatched safety status must be INFO."
                )

            if self.reason is not None:
                raise InvariantViolationError(
                    "an unlatched safety status cannot retain a reason."
                )

    @property
    def manual_rearm_required(self) -> bool:
        """Expose the mandatory manual-rearm condition."""

        return self.latched