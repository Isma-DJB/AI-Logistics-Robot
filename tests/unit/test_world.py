"""Unit tests for the immutable V1 grid map."""

import unittest
from dataclasses import FrozenInstanceError

from ai_logistics_robot.domain.enums import CellLayer
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position
from ai_logistics_robot.domain.world import GridMap


class GridMapTests(unittest.TestCase):
    """Verify grid bounds, occupancy, and arrival rules."""

    def reference_grid(self) -> GridMap:
        """Build the approved V1 reference environment."""

        return GridMap(
            width=10,
            height=10,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=Position(x=8, y=7),
            obstacles=frozenset(
                {
                    Position(x=1, y=4),
                    Position(x=2, y=4),
                    Position(x=3, y=4),
                    Position(x=5, y=6),
                }
            ),
        )

    def test_reference_grid_accepts_approved_configuration(self) -> None:
        grid = self.reference_grid()

        self.assertEqual(grid.width, 10)
        self.assertEqual(grid.height, 10)
        self.assertEqual(grid.cell_size_cm, 20.0)
        self.assertEqual(
            grid.layers,
            (
                CellLayer.TERRAIN,
                CellLayer.SEMANTIC,
                CellLayer.DYNAMIC,
            ),
        )

    def test_reference_grid_owns_coordinate_bounds(self) -> None:
        grid = self.reference_grid()

        self.assertTrue(grid.contains(Position(x=0, y=0)))
        self.assertTrue(grid.contains(Position(x=9, y=9)))
        self.assertFalse(grid.contains(Position(x=-1, y=0)))
        self.assertFalse(grid.contains(Position(x=10, y=9)))

    def test_shifted_origin_changes_grid_bounds(self) -> None:
        grid = GridMap(
            width=2,
            height=2,
            cell_size_cm=20,
            origin=Position(x=5, y=5),
            base_position=Position(x=5, y=5),
            target_position=Position(x=6, y=6),
        )

        self.assertTrue(grid.contains(Position(x=5, y=5)))
        self.assertTrue(grid.contains(Position(x=6, y=6)))
        self.assertFalse(grid.contains(Position(x=4, y=5)))
        self.assertFalse(grid.contains(Position(x=7, y=6)))

    def test_target_and_obstacles_are_not_traversable(self) -> None:
        grid = self.reference_grid()

        self.assertFalse(
            grid.is_traversable(grid.target_position)
        )
        self.assertFalse(
            grid.is_traversable(Position(x=1, y=4))
        )
        self.assertTrue(
            grid.is_traversable(Position(x=1, y=2))
        )

    def test_authorized_arrival_positions_are_adjacent_and_safe(
        self,
    ) -> None:
        grid = self.reference_grid()

        self.assertEqual(
            set(grid.authorized_arrival_positions),
            {
                Position(x=8, y=8),
                Position(x=9, y=7),
                Position(x=8, y=6),
                Position(x=7, y=7),
            },
        )

    def test_arrival_positions_exclude_obstacles_and_outside_cells(
        self,
    ) -> None:
        grid = GridMap(
            width=3,
            height=3,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=2, y=2),
            target_position=Position(x=0, y=0),
            obstacles=frozenset({Position(x=1, y=0)}),
        )

        self.assertEqual(
            grid.authorized_arrival_positions,
            (Position(x=0, y=1),),
        )

    def test_dimensions_must_be_positive_integers(self) -> None:
        for field in ("width", "height"):
            for value in (0, -1, 1.5, True):
                with self.subTest(field=field, value=value):
                    data: dict[str, object] = {
                        "width": 10,
                        "height": 10,
                        "cell_size_cm": 20,
                        "origin": Position(x=0, y=0),
                        "base_position": Position(x=1, y=1),
                        "target_position": Position(x=8, y=7),
                    }
                    data[field] = value

                    with self.assertRaises(
                        DomainValidationError
                    ):
                        GridMap(**data)

    def test_cell_size_must_be_positive_and_finite(self) -> None:
        invalid_values = (
            0,
            -1,
            float("nan"),
            float("inf"),
            True,
            "20",
        )

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    GridMap(  # type: ignore[arg-type]
                        width=10,
                        height=10,
                        cell_size_cm=value,
                        origin=Position(x=0, y=0),
                        base_position=Position(x=1, y=1),
                        target_position=Position(x=8, y=7),
                    )

    def test_base_and_target_must_be_inside_grid(self) -> None:
        for field in ("base_position", "target_position"):
            with self.subTest(field=field):
                data: dict[str, object] = {
                    "width": 10,
                    "height": 10,
                    "cell_size_cm": 20,
                    "origin": Position(x=0, y=0),
                    "base_position": Position(x=1, y=1),
                    "target_position": Position(x=8, y=7),
                }
                data[field] = Position(x=10, y=10)

                with self.assertRaises(InvariantViolationError):
                    GridMap(**data)

    def test_obstacles_must_be_immutable(self) -> None:
        with self.assertRaises(DomainValidationError):
            GridMap(  # type: ignore[arg-type]
                width=10,
                height=10,
                cell_size_cm=20,
                origin=Position(x=0, y=0),
                base_position=Position(x=1, y=1),
                target_position=Position(x=8, y=7),
                obstacles={Position(x=1, y=4)},
            )

    def test_obstacles_must_lie_inside_grid(self) -> None:
        with self.assertRaises(InvariantViolationError):
            GridMap(
                width=10,
                height=10,
                cell_size_cm=20,
                origin=Position(x=0, y=0),
                base_position=Position(x=1, y=1),
                target_position=Position(x=8, y=7),
                obstacles=frozenset(
                    {Position(x=10, y=10)}
                ),
            )

    def test_base_and_target_cannot_contain_obstacles(self) -> None:
        for obstacle in (
            Position(x=1, y=1),
            Position(x=8, y=7),
        ):
            with self.subTest(obstacle=obstacle):
                with self.assertRaises(
                    InvariantViolationError
                ):
                    GridMap(
                        width=10,
                        height=10,
                        cell_size_cm=20,
                        origin=Position(x=0, y=0),
                        base_position=Position(x=1, y=1),
                        target_position=Position(x=8, y=7),
                        obstacles=frozenset({obstacle}),
                    )

    def test_base_and_target_must_differ(self) -> None:
        with self.assertRaises(InvariantViolationError):
            GridMap(
                width=10,
                height=10,
                cell_size_cm=20,
                origin=Position(x=0, y=0),
                base_position=Position(x=1, y=1),
                target_position=Position(x=1, y=1),
            )

    def test_layers_must_be_unique_immutable_values(self) -> None:
        with self.assertRaises(InvariantViolationError):
            GridMap(
                width=10,
                height=10,
                cell_size_cm=20,
                origin=Position(x=0, y=0),
                base_position=Position(x=1, y=1),
                target_position=Position(x=8, y=7),
                layers=(
                    CellLayer.TERRAIN,
                    CellLayer.TERRAIN,
                ),
            )

        with self.assertRaises(DomainValidationError):
            GridMap(  # type: ignore[arg-type]
                width=10,
                height=10,
                cell_size_cm=20,
                origin=Position(x=0, y=0),
                base_position=Position(x=1, y=1),
                target_position=Position(x=8, y=7),
                layers=["terrain"],
            )

    def test_grid_map_is_immutable(self) -> None:
        grid = self.reference_grid()

        with self.assertRaises(FrozenInstanceError):
            grid.width = 20  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()