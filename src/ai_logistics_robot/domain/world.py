"""Immutable grid-map model and spatial validation rules."""

from dataclasses import dataclass
from math import isfinite

from ai_logistics_robot.domain.enums import CellLayer
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position


def _validate_positive_integer(name: str, value: object) -> None:
    """Require a strictly positive integer."""

    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < 1
    ):
        raise DomainValidationError(
            f"{name} must be a positive integer."
        )


def _validate_position(name: str, value: object) -> None:
    """Require a Position instance."""

    if not isinstance(value, Position):
        raise DomainValidationError(
            f"{name} must be a Position instance."
        )


@dataclass(frozen=True, slots=True)
class GridMap:
    """Platform-independent immutable representation of the V1 grid."""

    width: int
    height: int
    cell_size_cm: float
    origin: Position
    base_position: Position
    target_position: Position
    obstacles: frozenset[Position] = frozenset()
    layers: tuple[CellLayer, ...] = (
        CellLayer.TERRAIN,
        CellLayer.SEMANTIC,
        CellLayer.DYNAMIC,
    )

    def __post_init__(self) -> None:
        """Validate dimensions, layers, and all spatial invariants."""

        _validate_positive_integer("width", self.width)
        _validate_positive_integer("height", self.height)

        if (
            isinstance(self.cell_size_cm, bool)
            or not isinstance(self.cell_size_cm, (int, float))
        ):
            raise DomainValidationError(
                "cell_size_cm must be a numeric value."
            )

        cell_size_cm = float(self.cell_size_cm)

        if not isfinite(cell_size_cm) or cell_size_cm <= 0:
            raise DomainValidationError(
                "cell_size_cm must be finite and greater than zero."
            )

        object.__setattr__(self, "cell_size_cm", cell_size_cm)

        _validate_position("origin", self.origin)
        _validate_position("base_position", self.base_position)
        _validate_position("target_position", self.target_position)

        if not isinstance(self.obstacles, frozenset):
            raise DomainValidationError(
                "obstacles must be an immutable frozenset."
            )

        if not all(
            isinstance(position, Position)
            for position in self.obstacles
        ):
            raise DomainValidationError(
                "every obstacle must be a Position instance."
            )

        if not isinstance(self.layers, tuple):
            raise DomainValidationError(
                "layers must be an immutable tuple."
            )

        if not self.layers:
            raise DomainValidationError(
                "at least one grid layer is required."
            )

        if not all(
            isinstance(layer, CellLayer)
            for layer in self.layers
        ):
            raise DomainValidationError(
                "every layer must be a CellLayer instance."
            )

        if len(self.layers) != len(set(self.layers)):
            raise InvariantViolationError(
                "grid layers must be unique."
            )

        if not self.contains(self.base_position):
            raise InvariantViolationError(
                "base_position must lie within the grid."
            )

        if not self.contains(self.target_position):
            raise InvariantViolationError(
                "target_position must lie within the grid."
            )

        for obstacle in self.obstacles:
            if not self.contains(obstacle):
                raise InvariantViolationError(
                    "every obstacle must lie within the grid."
                )

        if self.base_position in self.obstacles:
            raise InvariantViolationError(
                "base_position cannot contain an obstacle."
            )

        if self.target_position in self.obstacles:
            raise InvariantViolationError(
                "target_position cannot contain an obstacle."
            )

        if self.base_position == self.target_position:
            raise InvariantViolationError(
                "base_position and target_position must differ."
            )

    def contains(self, position: Position) -> bool:
        """Return whether a position lies inside configured bounds."""

        _validate_position("position", position)

        return (
            self.origin.x
            <= position.x
            < self.origin.x + self.width
            and self.origin.y
            <= position.y
            < self.origin.y + self.height
        )

    def is_obstacle(self, position: Position) -> bool:
        """Return whether a grid position is occupied by an obstacle."""

        _validate_position("position", position)
        return position in self.obstacles

    def is_traversable(self, position: Position) -> bool:
        """Return whether the robot may occupy a position."""

        _validate_position("position", position)

        return (
            self.contains(position)
            and position not in self.obstacles
            and position != self.target_position
        )

    def adjacent_positions(
        self,
        position: Position,
    ) -> tuple[Position, ...]:
        """Return in-bounds cardinal neighbors in deterministic order."""

        _validate_position("position", position)

        candidates = (
            Position(x=position.x, y=position.y + 1),
            Position(x=position.x + 1, y=position.y),
            Position(x=position.x, y=position.y - 1),
            Position(x=position.x - 1, y=position.y),
        )

        return tuple(
            candidate
            for candidate in candidates
            if self.contains(candidate)
        )

    @property
    def authorized_arrival_positions(self) -> tuple[Position, ...]:
        """Return safe traversable cells adjacent to the target."""

        return tuple(
            position
            for position in self.adjacent_positions(
                self.target_position
            )
            if self.is_traversable(position)
        )