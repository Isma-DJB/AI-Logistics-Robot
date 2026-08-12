"""Unit tests for motion commands and execution results."""

import unittest
from dataclasses import FrozenInstanceError

from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    FailureReason,
    Heading,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose


class MotionCommandTests(unittest.TestCase):
    """Verify command validation and immutability."""

    def test_motion_command_accepts_valid_data(self) -> None:
        command = MotionCommand(
            robot_id="robot_1",
            command_type=CommandType.MOVE_FORWARD,
        )

        self.assertEqual(command.robot_id, "robot_1")
        self.assertIs(command.command_type, CommandType.MOVE_FORWARD)

    def test_motion_command_rejects_invalid_robot_id(self) -> None:
        invalid_values = ("", "   ", None, 1)

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    MotionCommand(  # type: ignore[arg-type]
                        robot_id=value,
                        command_type=CommandType.STOP,
                    )

    def test_motion_command_rejects_invalid_command_type(self) -> None:
        with self.assertRaises(DomainValidationError):
            MotionCommand(  # type: ignore[arg-type]
                robot_id="robot_1",
                command_type="STOP",
            )

    def test_motion_command_is_immutable(self) -> None:
        command = MotionCommand(
            robot_id="robot_1",
            command_type=CommandType.STOP,
        )

        with self.assertRaises(FrozenInstanceError):
            command.command_type = CommandType.TURN_LEFT  # type: ignore[misc]


class CommandResultTests(unittest.TestCase):
    """Verify result validation and execution invariants."""

    def setUp(self) -> None:
        self.command = MotionCommand(
            robot_id="robot_1",
            command_type=CommandType.MOVE_FORWARD,
        )
        self.pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )

    def test_successful_result_accepts_confirmed_movement(self) -> None:
        pose_after = RobotPose(
            position=Position(x=1, y=2),
            heading=Heading.NORTH,
        )
        result = CommandResult(
            command=self.command,
            status=CommandStatus.SUCCESS,
            pose_before=self.pose,
            pose_after=pose_after,
        )

        self.assertEqual(result.pose_after, pose_after)
        self.assertIsNone(result.failure_reason)

    def test_successful_result_rejects_failure_reason(self) -> None:
        with self.assertRaises(DomainValidationError):
            CommandResult(
                command=self.command,
                status=CommandStatus.SUCCESS,
                pose_before=self.pose,
                pose_after=self.pose,
                failure_reason=FailureReason.BLOCKED,
            )

    def test_unsuccessful_result_requires_failure_reason(self) -> None:
        with self.assertRaises(DomainValidationError):
            CommandResult(
                command=self.command,
                status=CommandStatus.FAILED,
                pose_before=self.pose,
                pose_after=self.pose,
            )

    def test_failed_result_cannot_change_confirmed_pose(self) -> None:
        changed_pose = RobotPose(
            position=Position(x=1, y=2),
            heading=Heading.NORTH,
        )

        with self.assertRaises(InvariantViolationError):
            CommandResult(
                command=self.command,
                status=CommandStatus.FAILED,
                pose_before=self.pose,
                pose_after=changed_pose,
                failure_reason=FailureReason.BLOCKED,
            )


if __name__ == "__main__":
    unittest.main()