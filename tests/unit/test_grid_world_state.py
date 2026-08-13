"""Unit tests for deterministic GridWorld state management."""

import unittest

from ai_logistics_robot.adapters.simulation.grid_world import GridWorld
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap


class GridWorldStateTests(unittest.TestCase):
    """Verify construction, time, world access, and reset behavior."""

    def setUp(self) -> None:
        self.world = GridMap(
            width=4,
            height=4,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=Position(x=3, y=3),
            obstacles=frozenset({Position(x=1, y=2)}),
        )
        self.initial_pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.simulation = GridWorld(
            world=self.world,
            robot_id="robot_1",
            initial_pose=self.initial_pose,
        )

    def test_initial_state_matches_configuration(self) -> None:
        self.assertEqual(
            self.simulation.current_pose,
            self.initial_pose,
        )
        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            0.0,
        )

    def test_read_world_returns_configured_immutable_map(self) -> None:
        self.assertIs(
            self.simulation.read_world(),
            self.world,
        )

    def test_public_state_properties_are_read_only(self) -> None:
        with self.assertRaises(AttributeError):
            self.simulation.current_pose = self.initial_pose  # type: ignore[misc]

        with self.assertRaises(AttributeError):
            self.simulation.elapsed_time_seconds = 1.0  # type: ignore[misc]

    def test_constructor_rejects_invalid_world(self) -> None:
        with self.assertRaises(DomainValidationError):
            GridWorld(  # type: ignore[arg-type]
                world="not-a-grid-map",
                robot_id="robot_1",
                initial_pose=self.initial_pose,
            )

    def test_constructor_rejects_invalid_robot_id(self) -> None:
        invalid_values = ("", "   ", None, 1)

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    GridWorld(  # type: ignore[arg-type]
                        world=self.world,
                        robot_id=value,
                        initial_pose=self.initial_pose,
                    )

    def test_constructor_rejects_invalid_pose_type(self) -> None:
        with self.assertRaises(DomainValidationError):
            GridWorld(  # type: ignore[arg-type]
                world=self.world,
                robot_id="robot_1",
                initial_pose=Position(x=1, y=1),
            )

    def test_constructor_rejects_non_traversable_pose(self) -> None:
        invalid_positions = (
            Position(x=1, y=2),
            self.world.target_position,
            Position(x=4, y=1),
        )

        for position in invalid_positions:
            with self.subTest(position=position):
                with self.assertRaises(DomainValidationError):
                    GridWorld(
                        world=self.world,
                        robot_id="robot_1",
                        initial_pose=RobotPose(
                            position=position,
                            heading=Heading.NORTH,
                        ),
                    )

    def test_advance_time_accumulates_valid_durations(self) -> None:
        self.simulation.advance_time(0)
        self.simulation.advance_time(1.25)
        self.simulation.advance_time(0.75)

        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            2.0,
        )

    def test_invalid_duration_preserves_elapsed_time(self) -> None:
        invalid_values = (
            -1,
            float("nan"),
            float("inf"),
            True,
            "1",
        )

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    self.simulation.advance_time(  # type: ignore[arg-type]
                        value
                    )

                self.assertEqual(
                    self.simulation.elapsed_time_seconds,
                    0.0,
                )

    def test_time_overflow_is_rejected_atomically(self) -> None:
        self.simulation.advance_time(1e308)

        with self.assertRaises(DomainValidationError):
            self.simulation.advance_time(1e308)

        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            1e308,
        )

    def test_reset_restores_initial_state(self) -> None:
        self.simulation.advance_time(3)

        self.simulation.reset()

        self.assertEqual(
            self.simulation.current_pose,
            self.initial_pose,
        )
        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            0.0,
        )


if __name__ == "__main__":
    unittest.main()