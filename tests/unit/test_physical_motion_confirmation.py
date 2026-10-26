"""Unit tests for camera-confirmed physical motion geometry."""

import unittest

from ai_logistics_robot.control.physical_motion_confirmation import (
    expected_pose_after_command,
    pose_confirms_command,
)
from ai_logistics_robot.domain.enums import CommandType, Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose


class PhysicalMotionConfirmationTests(unittest.TestCase):
    """Verify exact pose expectations for every command."""

    def pose(
        self,
        *,
        x: int = 3,
        y: int = 3,
        heading: Heading = Heading.NORTH,
    ) -> RobotPose:
        """Build one deterministic pose."""

        return RobotPose(
            position=Position(x=x, y=y),
            heading=heading,
        )

    def test_stop_preserves_pose(self) -> None:
        pose_before = self.pose()

        pose_after = expected_pose_after_command(
            pose_before=pose_before,
            command_type=CommandType.STOP,
        )

        self.assertIs(pose_after, pose_before)

    def test_left_turn_rotates_each_cardinal_heading(self) -> None:
        cases = (
            (Heading.NORTH, Heading.WEST),
            (Heading.WEST, Heading.SOUTH),
            (Heading.SOUTH, Heading.EAST),
            (Heading.EAST, Heading.NORTH),
        )

        for heading_before, heading_after in cases:
            with self.subTest(heading=heading_before):
                result = expected_pose_after_command(
                    pose_before=self.pose(
                        heading=heading_before
                    ),
                    command_type=CommandType.TURN_LEFT,
                )

                self.assertEqual(result.position, Position(3, 3))
                self.assertIs(result.heading, heading_after)

    def test_right_turn_rotates_each_cardinal_heading(self) -> None:
        cases = (
            (Heading.NORTH, Heading.EAST),
            (Heading.EAST, Heading.SOUTH),
            (Heading.SOUTH, Heading.WEST),
            (Heading.WEST, Heading.NORTH),
        )

        for heading_before, heading_after in cases:
            with self.subTest(heading=heading_before):
                result = expected_pose_after_command(
                    pose_before=self.pose(
                        heading=heading_before
                    ),
                    command_type=CommandType.TURN_RIGHT,
                )

                self.assertEqual(result.position, Position(3, 3))
                self.assertIs(result.heading, heading_after)

    def test_forward_uses_domain_grid_orientation(self) -> None:
        cases = (
            (Heading.NORTH, Position(3, 4)),
            (Heading.EAST, Position(4, 3)),
            (Heading.SOUTH, Position(3, 2)),
            (Heading.WEST, Position(2, 3)),
        )

        for heading, expected_position in cases:
            with self.subTest(heading=heading):
                result = expected_pose_after_command(
                    pose_before=self.pose(heading=heading),
                    command_type=CommandType.MOVE_FORWARD,
                )

                self.assertEqual(
                    result.position,
                    expected_position,
                )
                self.assertIs(result.heading, heading)

    def test_exact_expected_pose_confirms_command(self) -> None:
        pose_before = self.pose()
        observed_pose = self.pose(
            heading=Heading.WEST
        )

        confirmed = pose_confirms_command(
            pose_before=pose_before,
            command_type=CommandType.TURN_LEFT,
            observed_pose=observed_pose,
        )

        self.assertTrue(confirmed)

    def test_wrong_heading_does_not_confirm_turn(self) -> None:
        confirmed = pose_confirms_command(
            pose_before=self.pose(),
            command_type=CommandType.TURN_LEFT,
            observed_pose=self.pose(
                heading=Heading.NORTH
            ),
        )

        self.assertFalse(confirmed)

    def test_displaced_turn_does_not_confirm_command(self) -> None:
        confirmed = pose_confirms_command(
            pose_before=self.pose(),
            command_type=CommandType.TURN_LEFT,
            observed_pose=self.pose(
                x=2,
                y=3,
                heading=Heading.WEST,
            ),
        )

        self.assertFalse(confirmed)

    def test_invalid_values_are_rejected(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "pose_before",
        ):
            expected_pose_after_command(
                pose_before="invalid",  # type: ignore[arg-type]
                command_type=CommandType.STOP,
            )

        with self.assertRaisesRegex(
            DomainValidationError,
            "command_type",
        ):
            expected_pose_after_command(
                pose_before=self.pose(),
                command_type="STOP",  # type: ignore[arg-type]
            )

        with self.assertRaisesRegex(
            DomainValidationError,
            "observed_pose",
        ):
            pose_confirms_command(
                pose_before=self.pose(),
                command_type=CommandType.STOP,
                observed_pose="invalid",  # type: ignore[arg-type]
            )


if __name__ == "__main__":
    unittest.main()
