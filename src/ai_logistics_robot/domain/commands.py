"""Immutable motion-command and execution-result objects."""

from dataclasses import dataclass

from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    FailureReason,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import RobotPose


@dataclass(frozen=True, slots=True)
class MotionCommand:
    """A platform-independent motion instruction for one robot."""

    robot_id: str
    command_type: CommandType

    def __post_init__(self) -> None:
        """Validate the command contract."""

        if not isinstance(self.robot_id, str) or not self.robot_id.strip():
            raise DomainValidationError(
                "robot_id must be a non-empty string."
            )

        if not isinstance(self.command_type, CommandType):
            raise DomainValidationError(
                "command_type must be a CommandType instance."
            )


@dataclass(frozen=True, slots=True)
class CommandResult:
    """Confirmed outcome of executing one motion command."""

    command: MotionCommand
    status: CommandStatus
    pose_before: RobotPose
    pose_after: RobotPose
    failure_reason: FailureReason | None = None

    def __post_init__(self) -> None:
        """Validate execution data and movement invariants."""

        if not isinstance(self.command, MotionCommand):
            raise DomainValidationError(
                "command must be a MotionCommand instance."
            )

        if not isinstance(self.status, CommandStatus):
            raise DomainValidationError(
                "status must be a CommandStatus instance."
            )

        if not isinstance(self.pose_before, RobotPose):
            raise DomainValidationError(
                "pose_before must be a RobotPose instance."
            )

        if not isinstance(self.pose_after, RobotPose):
            raise DomainValidationError(
                "pose_after must be a RobotPose instance."
            )

        if (
            self.failure_reason is not None
            and not isinstance(self.failure_reason, FailureReason)
        ):
            raise DomainValidationError(
                "failure_reason must be a FailureReason or None."
            )

        if (
            self.status is CommandStatus.SUCCESS
            and self.failure_reason is not None
        ):
            raise DomainValidationError(
                "a successful command cannot have a failure reason."
            )

        if (
            self.status is not CommandStatus.SUCCESS
            and self.failure_reason is None
        ):
            raise DomainValidationError(
                "an unsuccessful command requires a failure reason."
            )

        if (
            self.status is CommandStatus.FAILED
            and self.pose_after != self.pose_before
        ):
            raise InvariantViolationError(
                "a failed command must preserve the confirmed pose."
            )