"""Immutable read-only system-status model."""

from dataclasses import dataclass
from datetime import datetime

from ai_logistics_robot.domain.enums import (
    BrainState,
    FailureReason,
    MissionStatus,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.safety import SafetyStatus


def _validate_identifier(name: str, value: object) -> None:
    """Require a non-empty string identifier."""

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
            "observed_at must be a timezone-aware datetime."
        )


@dataclass(frozen=True, slots=True)
class SystemStatus:
    """Read-only snapshot assembled from confirmed system state."""

    robot_id: str
    observed_at: datetime
    brain_state: BrainState
    robot_pose: RobotPose
    safety_status: SafetyStatus
    mission_id: str | None = None
    mission_status: MissionStatus | None = None
    active_plan: PathPlan | None = None
    latest_error: FailureReason | None = None

    def __post_init__(self) -> None:
        """Validate status consistency without causing control effects."""

        _validate_identifier("robot_id", self.robot_id)
        _validate_timestamp(self.observed_at)

        if not isinstance(self.brain_state, BrainState):
            raise DomainValidationError(
                "brain_state must be a BrainState instance."
            )

        if not isinstance(self.robot_pose, RobotPose):
            raise DomainValidationError(
                "robot_pose must be a RobotPose instance."
            )

        if not isinstance(self.safety_status, SafetyStatus):
            raise DomainValidationError(
                "safety_status must be a SafetyStatus instance."
            )

        if self.safety_status.robot_id != self.robot_id:
            raise InvariantViolationError(
                "safety status must belong to the same robot."
            )

        if self.mission_id is not None:
            _validate_identifier("mission_id", self.mission_id)

        if (
            self.mission_status is not None
            and not isinstance(self.mission_status, MissionStatus)
        ):
            raise DomainValidationError(
                "mission_status must be a MissionStatus or None."
            )

        if (self.mission_id is None) != (self.mission_status is None):
            raise InvariantViolationError(
                "mission_id and mission_status must be present together."
            )

        if (
            self.active_plan is not None
            and not isinstance(self.active_plan, PathPlan)
        ):
            raise DomainValidationError(
                "active_plan must be a PathPlan or None."
            )

        if self.active_plan is not None:
            if self.mission_id is None:
                raise InvariantViolationError(
                    "an active plan requires an active mission reference."
                )

            if self.active_plan.mission_id != self.mission_id:
                raise InvariantViolationError(
                    "active plan and system status must reference "
                    "the same mission."
                )

            if self.active_plan.robot_id != self.robot_id:
                raise InvariantViolationError(
                    "active plan and system status must reference "
                    "the same robot."
                )

        if (
            self.latest_error is not None
            and not isinstance(self.latest_error, FailureReason)
        ):
            raise DomainValidationError(
                "latest_error must be a FailureReason or None."
            )