"""Immutable geometry objects for the V1 domain model."""

from dataclasses import dataclass

from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvalidCoordinateError,
)


def _validate_coordinate(name: str, value: object) -> None:
    """Require an integer coordinate while explicitly rejecting booleans."""

    if isinstance(value, bool) or not isinstance(value, int):
        raise InvalidCoordinateError(
            f"{name} must be an integer, got {type(value).__name__}."
        )


@dataclass(frozen=True, slots=True)
class Position:
    """Platform-independent coordinate whose bounds belong to GridMap."""

    x: int
    y: int

    def __post_init__(self) -> None:
        """Validate the coordinate representation."""

        _validate_coordinate("x", self.x)
        _validate_coordinate("y", self.y)


@dataclass(frozen=True, slots=True)
class RobotPose:
    """Confirmed robot position and cardinal heading."""

    position: Position
    heading: Heading

    def __post_init__(self) -> None:
        """Validate the composed geometry values."""

        if not isinstance(self.position, Position):
            raise DomainValidationError(
                "position must be a Position instance."
            )

        if not isinstance(self.heading, Heading):
            raise DomainValidationError(
                "heading must be a Heading instance."
            )