"""Immutable mission model and mission-level invariants."""

from dataclasses import dataclass

from ai_logistics_robot.domain.enums import FailureReason, MissionStatus
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position


def _validate_identifier(name: str, value: object) -> None:
    """Require a non-empty string identifier."""

    if not isinstance(value, str) or not value.strip():
        raise DomainValidationError(
            f"{name} must be a non-empty string."
        )


@dataclass(frozen=True, slots=True)
class Mission:
    """Platform-independent state of one robot mission."""

    mission_id: str
    robot_id: str
    target_id: str
    target_position: Position
    base_position: Position
    status: MissionStatus = MissionStatus.CREATED
    collection_completed: bool = False
    base_arrival_confirmed: bool = False
    terminal_reason: FailureReason | None = None

    def __post_init__(self) -> None:
        """Validate mission data and terminal-state invariants."""

        _validate_identifier("mission_id", self.mission_id)
        _validate_identifier("robot_id", self.robot_id)
        _validate_identifier("target_id", self.target_id)

        if not isinstance(self.target_position, Position):
            raise DomainValidationError(
                "target_position must be a Position instance."
            )

        if not isinstance(self.base_position, Position):
            raise DomainValidationError(
                "base_position must be a Position instance."
            )

        if not isinstance(self.status, MissionStatus):
            raise DomainValidationError(
                "status must be a MissionStatus instance."
            )

        if not isinstance(self.collection_completed, bool):
            raise DomainValidationError(
                "collection_completed must be a boolean."
            )

        if not isinstance(self.base_arrival_confirmed, bool):
            raise DomainValidationError(
                "base_arrival_confirmed must be a boolean."
            )

        if (
            self.terminal_reason is not None
            and not isinstance(self.terminal_reason, FailureReason)
        ):
            raise DomainValidationError(
                "terminal_reason must be a FailureReason or None."
            )

        if (
            self.base_arrival_confirmed
            and not self.collection_completed
        ):
            raise InvariantViolationError(
                "base arrival cannot be confirmed before collection."
            )

        if self.status is MissionStatus.SUCCESS:
            if (
                not self.collection_completed
                or not self.base_arrival_confirmed
            ):
                raise InvariantViolationError(
                    "a successful mission requires completed collection "
                    "and confirmed base arrival."
                )

            if self.terminal_reason is not None:
                raise InvariantViolationError(
                    "a successful mission cannot have a terminal reason."
                )

        if self.status in (
            MissionStatus.FAILED,
            MissionStatus.ABORTED,
        ):
            if self.terminal_reason is None:
                raise InvariantViolationError(
                    "a failed or aborted mission requires an explicit "
                    "terminal reason."
                )

        if self.status in (
            MissionStatus.CREATED,
            MissionStatus.ACTIVE,
        ):
            if self.terminal_reason is not None:
                raise InvariantViolationError(
                    "a non-terminal mission cannot have a terminal reason."
                )