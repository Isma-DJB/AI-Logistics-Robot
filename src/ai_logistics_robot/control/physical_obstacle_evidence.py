"""Validated obstacle evidence for assisted physical missions."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum, unique
from math import isfinite

from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position
from ai_logistics_robot.domain.world import GridMap


@unique
class ObstacleEvidenceSource(StrEnum):
    """Origin of one physical obstacle statement."""

    CAMERA = "CAMERA"
    OPERATOR = "OPERATOR"
    LOCAL_SENSOR = "LOCAL_SENSOR"


@unique
class ObstacleEvidenceStatus(StrEnum):
    """Validation state of one physical obstacle statement."""

    DETECTED = "DETECTED"
    CONFIRMED = "CONFIRMED"
    CLEARED = "CLEARED"


_ALLOWED_STATUSES = {
    ObstacleEvidenceSource.CAMERA: frozenset(
        {
            ObstacleEvidenceStatus.DETECTED,
        }
    ),
    ObstacleEvidenceSource.OPERATOR: frozenset(
        {
            ObstacleEvidenceStatus.CONFIRMED,
            ObstacleEvidenceStatus.CLEARED,
        }
    ),
    ObstacleEvidenceSource.LOCAL_SENSOR: frozenset(
        {
            ObstacleEvidenceStatus.CONFIRMED,
        }
    ),
}


@dataclass(frozen=True, slots=True)
class ObstacleEvidence:
    """Latest validated statement about one physical grid cell."""

    position: Position
    source: ObstacleEvidenceSource
    status: ObstacleEvidenceStatus
    confidence: float

    def __post_init__(self) -> None:
        """Validate evidence type, source policy, and confidence."""

        if not isinstance(self.position, Position):
            raise DomainValidationError(
                "position must be a Position instance."
            )

        if not isinstance(
            self.source,
            ObstacleEvidenceSource,
        ):
            raise DomainValidationError(
                "source must be an ObstacleEvidenceSource instance."
            )

        if not isinstance(
            self.status,
            ObstacleEvidenceStatus,
        ):
            raise DomainValidationError(
                "status must be an ObstacleEvidenceStatus instance."
            )

        if self.status not in _ALLOWED_STATUSES[self.source]:
            raise InvariantViolationError(
                "obstacle evidence status is not authorized "
                "for its source."
            )

        if (
            isinstance(self.confidence, bool)
            or not isinstance(
                self.confidence,
                (int, float),
            )
        ):
            raise DomainValidationError(
                "confidence must be numeric."
            )

        normalized_confidence = float(
            self.confidence
        )
        if (
            not isfinite(normalized_confidence)
            or not 0.0
            <= normalized_confidence
            <= 1.0
        ):
            raise DomainValidationError(
                "confidence must be finite and between zero and one."
            )

        object.__setattr__(
            self,
            "confidence",
            normalized_confidence,
        )


@dataclass(frozen=True, slots=True)
class PhysicalObstacleMap:
    """Immutable obstacle evidence over one base grid."""

    base_world: GridMap
    evidence: tuple[ObstacleEvidence, ...] = ()

    def __post_init__(self) -> None:
        """Validate evidence uniqueness and spatial safety."""

        if not isinstance(self.base_world, GridMap):
            raise DomainValidationError(
                "base_world must be a GridMap instance."
            )

        if not isinstance(self.evidence, tuple):
            raise DomainValidationError(
                "evidence must be an immutable tuple."
            )

        if not all(
            isinstance(item, ObstacleEvidence)
            for item in self.evidence
        ):
            raise DomainValidationError(
                "every evidence item must be ObstacleEvidence."
            )

        positions = tuple(
            item.position
            for item in self.evidence
        )
        if len(positions) != len(set(positions)):
            raise InvariantViolationError(
                "physical obstacle evidence positions must be unique."
            )

        for item in self.evidence:
            if not self.base_world.contains(
                item.position
            ):
                raise InvariantViolationError(
                    "physical obstacle evidence must lie "
                    "within the grid."
                )

            if item.position in (
                self.base_world.base_position,
                self.base_world.target_position,
            ):
                raise InvariantViolationError(
                    "base and target cells cannot contain "
                    "physical obstacle evidence."
                )

    @property
    def detected_positions(
        self,
    ) -> frozenset[Position]:
        """Return unconfirmed camera proposals."""

        return self._positions_with_status(
            ObstacleEvidenceStatus.DETECTED
        )

    @property
    def confirmed_positions(
        self,
    ) -> frozenset[Position]:
        """Return cells authorized to block planning."""

        return self._positions_with_status(
            ObstacleEvidenceStatus.CONFIRMED
        )

    @property
    def cleared_positions(
        self,
    ) -> frozenset[Position]:
        """Return cells explicitly confirmed as clear."""

        return self._positions_with_status(
            ObstacleEvidenceStatus.CLEARED
        )

    @property
    def confirmed_world(self) -> GridMap:
        """Apply only confirmed and cleared evidence to the grid."""

        obstacles = (
            self.base_world.obstacles
            - self.cleared_positions
        ) | self.confirmed_positions

        return replace(
            self.base_world,
            obstacles=obstacles,
        )

    def _positions_with_status(
        self,
        status: ObstacleEvidenceStatus,
    ) -> frozenset[Position]:
        """Return positions carrying one exact status."""

        return frozenset(
            item.position
            for item in self.evidence
            if item.status is status
        )
