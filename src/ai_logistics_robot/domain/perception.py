"""Immutable normalized perception objects."""

from dataclasses import dataclass
from datetime import datetime
from math import isfinite

from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose


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
            "captured_at must be a timezone-aware datetime."
        )


@dataclass(frozen=True, slots=True)
class Observation:
    """One normalized spatial observation."""

    kind: str
    position: Position
    confidence: float

    def __post_init__(self) -> None:
        """Validate and normalize observation data."""

        if not isinstance(self.kind, str) or not self.kind.strip():
            raise DomainValidationError(
                "kind must be a non-empty string."
            )

        if not isinstance(self.position, Position):
            raise DomainValidationError(
                "position must be a Position instance."
            )

        if (
            isinstance(self.confidence, bool)
            or not isinstance(self.confidence, (int, float))
        ):
            raise DomainValidationError(
                "confidence must be a numeric value."
            )

        confidence = float(self.confidence)

        if not isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise DomainValidationError(
                "confidence must lie within [0.0, 1.0]."
            )

        object.__setattr__(self, "confidence", confidence)


@dataclass(frozen=True, slots=True)
class PerceptionSnapshot:
    """Consistent perception result produced at one instant."""

    robot_id: str
    captured_at: datetime
    robot_pose: RobotPose
    observations: tuple[Observation, ...]
    target_active: bool
    hazard_detected: bool

    def __post_init__(self) -> None:
        """Validate snapshot consistency and immutability."""

        _validate_identifier("robot_id", self.robot_id)
        _validate_timestamp(self.captured_at)

        if not isinstance(self.robot_pose, RobotPose):
            raise DomainValidationError(
                "robot_pose must be a RobotPose instance."
            )

        if not isinstance(self.observations, tuple):
            raise DomainValidationError(
                "observations must be an immutable tuple."
            )

        if not all(
            isinstance(observation, Observation)
            for observation in self.observations
        ):
            raise DomainValidationError(
                "every observation must be an Observation instance."
            )

        if not isinstance(self.target_active, bool):
            raise DomainValidationError(
                "target_active must be a boolean."
            )

        if not isinstance(self.hazard_detected, bool):
            raise DomainValidationError(
                "hazard_detected must be a boolean."
            )