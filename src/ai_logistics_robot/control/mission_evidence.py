"""Immutable evidence contract for assisted physical missions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum, unique

from ai_logistics_robot.control.physical_obstacle_evidence import (
    PhysicalObstacleMap,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.world import GridMap


@unique
class MissionEvidenceStatus(StrEnum):
    """Visible outcome represented by one mission evidence frame."""

    PLANNED = "PLANNED"
    CONFIRMED = "CONFIRMED"
    REJECTED = "REJECTED"
    REPLANNED = "REPLANNED"
    COMPLETED = "COMPLETED"


@dataclass(frozen=True, slots=True)
class MissionEvidence:
    """Consistent mission state used to generate one visual proof."""

    scenario_id: str
    step_number: int
    stage: str
    status: MissionEvidenceStatus
    world: GridMap
    perception: PerceptionSnapshot
    obstacle_map: PhysicalObstacleMap | None = None
    plan: PathPlan | None = None
    confirmed_positions: tuple[Position, ...] = ()
    next_position: Position | None = None

    def __post_init__(self) -> None:
        """Validate evidence identity and spatial consistency."""

        _validate_identifier(
            "scenario_id",
            self.scenario_id,
        )
        _validate_identifier(
            "stage",
            self.stage,
        )

        if (
            isinstance(self.step_number, bool)
            or not isinstance(self.step_number, int)
            or self.step_number < 1
        ):
            raise DomainValidationError(
                "step_number must be a positive integer."
            )

        if not isinstance(
            self.status,
            MissionEvidenceStatus,
        ):
            raise DomainValidationError(
                "status must be a MissionEvidenceStatus instance."
            )

        if not isinstance(self.world, GridMap):
            raise DomainValidationError(
                "world must be a GridMap instance."
            )

        if not isinstance(
            self.perception,
            PerceptionSnapshot,
        ):
            raise DomainValidationError(
                "perception must be a PerceptionSnapshot instance."
            )

        if (
            self.obstacle_map is not None
            and not isinstance(
                self.obstacle_map,
                PhysicalObstacleMap,
            )
        ):
            raise DomainValidationError(
                "obstacle_map must be a PhysicalObstacleMap "
                "instance or None."
            )

        if (
            self.obstacle_map is not None
            and self.obstacle_map.confirmed_world
            != self.world
        ):
            raise InvariantViolationError(
                "evidence world must equal the confirmed "
                "physical obstacle world."
            )

        if (
            self.plan is not None
            and not isinstance(self.plan, PathPlan)
        ):
            raise DomainValidationError(
                "plan must be a PathPlan instance or None."
            )

        if not isinstance(
            self.confirmed_positions,
            tuple,
        ):
            raise DomainValidationError(
                "confirmed_positions must be an immutable tuple."
            )

        if not all(
            isinstance(position, Position)
            for position in self.confirmed_positions
        ):
            raise DomainValidationError(
                "every confirmed position must be a Position instance."
            )

        if (
            self.next_position is not None
            and not isinstance(
                self.next_position,
                Position,
            )
        ):
            raise DomainValidationError(
                "next_position must be a Position instance or None."
            )

        robot_position = (
            self.perception.robot_pose.position
        )
        self._require_traversable(
            "perceived robot position",
            robot_position,
        )

        for position in self.confirmed_positions:
            self._require_traversable(
                "confirmed position",
                position,
            )

        if self.plan is None:
            if self.next_position is not None:
                raise InvariantViolationError(
                    "next_position requires a path plan."
                )
            return

        if (
            self.plan.robot_id
            != self.perception.robot_id
        ):
            raise InvariantViolationError(
                "plan and perception robot identifiers must match."
            )

        for position in self.plan.positions:
            self._require_traversable(
                "planned position",
                position,
            )

        if (
            self.next_position is not None
            and self.next_position
            not in self.plan.positions
        ):
            raise InvariantViolationError(
                "next_position must belong to the path plan."
            )

    def _require_traversable(
        self,
        label: str,
        position: Position,
    ) -> None:
        """Require one evidence position to be safely occupiable."""

        if not self.world.contains(position):
            raise InvariantViolationError(
                f"{label} must lie within the grid."
            )

        if not self.world.is_traversable(position):
            raise InvariantViolationError(
                f"{label} must be traversable."
            )


def _validate_identifier(
    name: str,
    value: object,
) -> None:
    """Require one non-empty textual identifier."""

    if (
        not isinstance(value, str)
        or not value.strip()
    ):
        raise DomainValidationError(
            f"{name} must be a non-empty string."
        )
