"""Unit tests for immutable domain geometry objects."""

import unittest
from dataclasses import FrozenInstanceError

from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvalidCoordinateError,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose


class PositionTests(unittest.TestCase):
    """Verify coordinate validation and immutability."""

    def test_position_accepts_integer_coordinates(self) -> None:
        position = Position(x=1, y=7)

        self.assertEqual(position.x, 1)
        self.assertEqual(position.y, 7)

    def test_position_does_not_hard_code_grid_bounds(self) -> None:
        position = Position(x=-1, y=10)

        self.assertEqual(position, Position(x=-1, y=10))

    def test_position_rejects_non_integer_coordinates(self) -> None:
        invalid_values = (1.5, "1", None, True)

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(InvalidCoordinateError):
                    Position(x=value, y=0)  # type: ignore[arg-type]

    def test_position_is_immutable(self) -> None:
        position = Position(x=1, y=1)

        with self.assertRaises(FrozenInstanceError):
            position.x = 2  # type: ignore[misc]


class RobotPoseTests(unittest.TestCase):
    """Verify pose composition and immutability."""

    def test_robot_pose_contains_position_and_heading(self) -> None:
        position = Position(x=1, y=1)
        pose = RobotPose(position=position, heading=Heading.NORTH)

        self.assertEqual(pose.position, position)
        self.assertIs(pose.heading, Heading.NORTH)

    def test_robot_pose_rejects_invalid_position(self) -> None:
        with self.assertRaises(DomainValidationError):
            RobotPose(  # type: ignore[arg-type]
                position=(1, 1),
                heading=Heading.NORTH,
            )

    def test_robot_pose_rejects_invalid_heading(self) -> None:
        with self.assertRaises(DomainValidationError):
            RobotPose(  # type: ignore[arg-type]
                position=Position(x=1, y=1),
                heading="NORTH",
            )

    def test_robot_pose_is_immutable(self) -> None:
        pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )

        with self.assertRaises(FrozenInstanceError):
            pose.heading = Heading.SOUTH  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()