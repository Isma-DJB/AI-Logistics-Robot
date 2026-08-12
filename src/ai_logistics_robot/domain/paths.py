"""Immutable planned-path and confirmed-path-record objects."""

from dataclasses import dataclass

from ai_logistics_robot.domain.enums import PathPhase
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose


def _validate_identifier(name: str, value: object) -> None:
    """Require a non-empty string identifier."""

    if not isinstance(value, str) or not value.strip():
        raise DomainValidationError(
            f"{name} must be a non-empty string."
        )


@dataclass(frozen=True, slots=True)
class PathPlan:
    """A versioned path ending at an authorized goal position."""

    mission_id: str
    robot_id: str
    phase: PathPhase
    version: int
    positions: tuple[Position, ...]
    goal: Position

    def __post_init__(self) -> None:
        """Validate the planned path contract."""

        _validate_identifier("mission_id", self.mission_id)
        _validate_identifier("robot_id", self.robot_id)

        if not isinstance(self.phase, PathPhase):
            raise DomainValidationError(
                "phase must be a PathPhase instance."
            )

        if (
            isinstance(self.version, bool)
            or not isinstance(self.version, int)
            or self.version < 1
        ):
            raise DomainValidationError(
                "version must be a positive integer."
            )

        if not isinstance(self.positions, tuple):
            raise DomainValidationError(
                "positions must be an immutable tuple."
            )

        if not self.positions:
            raise DomainValidationError(
                "a path plan must contain at least one position."
            )

        if not all(
            isinstance(position, Position)
            for position in self.positions
        ):
            raise DomainValidationError(
                "every planned position must be a Position instance."
            )

        if not isinstance(self.goal, Position):
            raise DomainValidationError(
                "goal must be a Position instance."
            )

        if self.positions[-1] != self.goal:
            raise InvariantViolationError(
                "the final planned position must equal the goal."
            )


@dataclass(frozen=True, slots=True)
class PathRecord:
    """Confirmed robot poses recorded during one navigation phase."""

    mission_id: str
    robot_id: str
    phase: PathPhase
    confirmed_poses: tuple[RobotPose, ...]

    def __post_init__(self) -> None:
        """Validate the confirmed path record."""

        _validate_identifier("mission_id", self.mission_id)
        _validate_identifier("robot_id", self.robot_id)

        if not isinstance(self.phase, PathPhase):
            raise DomainValidationError(
                "phase must be a PathPhase instance."
            )

        if self.phase not in (
            PathPhase.OUTBOUND,
            PathPhase.RETURN,
        ):
            raise DomainValidationError(
                "a path record phase must be OUTBOUND or RETURN."
            )

        if not isinstance(self.confirmed_poses, tuple):
            raise DomainValidationError(
                "confirmed_poses must be an immutable tuple."
            )

        if not all(
            isinstance(pose, RobotPose)
            for pose in self.confirmed_poses
        ):
            raise DomainValidationError(
                "every confirmed pose must be a RobotPose instance."
            )

    @property
    def confirmed_positions(self) -> tuple[Position, ...]:
        """Expose positions without changing the confirmed history."""

        return tuple(
            pose.position for pose in self.confirmed_poses
        )