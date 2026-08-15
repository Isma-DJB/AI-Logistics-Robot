"""Unit tests for deterministic safe command control."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.control.safe_robot_control import (
    SafeRobotControl,
)
from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    FailureReason,
    Heading,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports.control_port import ControlPort


class DeterministicClock:
    """Provide fixed deterministic time for Control tests."""

    def __init__(self) -> None:
        self.current_time = datetime(
            2026,
            8,
            15,
            tzinfo=UTC,
        )
        self.now_calls = 0
        self.waited_deadline: float | None = None

    def now(self) -> datetime:
        """Return the fixed timezone-aware timestamp."""

        self.now_calls += 1
        return self.current_time

    def monotonic(self) -> float:
        """Return deterministic monotonic time."""

        return 10.0

    def wait_until(self, deadline: float) -> None:
        """Record a requested deadline without waiting."""

        self.waited_deadline = deadline


class RecordingSimulation:
    """Return scripted command outcomes and record port calls."""

    def __init__(
        self,
        *,
        world: GridMap,
        initial_pose: RobotPose,
        outcomes: list[
            tuple[RobotPose, FailureReason | None]
        ] | None = None,
    ) -> None:
        self.world = world
        self.initial_pose = initial_pose
        self.current_pose = initial_pose
        self.outcomes = [] if outcomes is None else outcomes
        self.applied_commands: list[MotionCommand] = []
        self.advanced_seconds = 0.0
        self.reset_calls = 0

    def reset(self) -> None:
        """Restore the scripted initial simulation state."""

        self.current_pose = self.initial_pose
        self.applied_commands.clear()
        self.advanced_seconds = 0.0
        self.reset_calls += 1

    def read_world(self) -> GridMap:
        """Return the configured immutable world."""

        return self.world

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Return the next scripted result for the supplied command."""

        self.applied_commands.append(command)
        pose_before = self.current_pose

        if self.outcomes:
            pose_after, failure_reason = self.outcomes.pop(0)
        else:
            pose_after = pose_before
            failure_reason = None

        status = (
            CommandStatus.SUCCESS
            if failure_reason is None
            else CommandStatus.FAILED
        )

        result = CommandResult(
            command=command,
            status=status,
            pose_before=pose_before,
            pose_after=pose_after,
            failure_reason=failure_reason,
        )

        if result.status is CommandStatus.SUCCESS:
            self.current_pose = result.pose_after

        return result

    def advance_time(self, seconds: float) -> None:
        """Record simulated-time advancement."""

        self.advanced_seconds += seconds


class MismatchedPoseSimulation(RecordingSimulation):
    """Return a result that disagrees with Control's confirmed pose."""

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Return a structurally valid but inconsistent result."""

        self.applied_commands.append(command)
        unexpected_pose = RobotPose(
            position=Position(x=2, y=2),
            heading=Heading.NORTH,
        )

        return CommandResult(
            command=command,
            status=CommandStatus.SUCCESS,
            pose_before=unexpected_pose,
            pose_after=unexpected_pose,
        )


class SafeRobotControlNominalTests(unittest.TestCase):
    """Verify nominal platform-independent Control behavior."""

    def setUp(self) -> None:
        self.robot_id = "robot_1"
        self.timestamp = datetime(
            2026,
            8,
            15,
            tzinfo=UTC,
        )
        self.initial_pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.world = GridMap(
            width=5,
            height=5,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=Position(x=4, y=4),
        )
        self.clock = DeterministicClock()

    def build_control(
        self,
        *,
        outcomes: list[
            tuple[RobotPose, FailureReason | None]
        ] | None = None,
        simulation: RecordingSimulation | None = None,
    ) -> tuple[SafeRobotControl, RecordingSimulation]:
        """Build Control and its inspectable simulation fake."""

        selected_simulation = (
            RecordingSimulation(
                world=self.world,
                initial_pose=self.initial_pose,
                outcomes=outcomes,
            )
            if simulation is None
            else simulation
        )
        control = SafeRobotControl(
            robot_id=self.robot_id,
            initial_pose=self.initial_pose,
            simulation=selected_simulation,
            clock=self.clock,
        )

        return control, selected_simulation

    def command(
        self,
        command_type: CommandType,
    ) -> MotionCommand:
        """Build one command for the configured robot."""

        return MotionCommand(
            robot_id=self.robot_id,
            command_type=command_type,
        )

    def test_control_satisfies_runtime_protocol_and_starts_safe(
        self,
    ) -> None:
        control, simulation = self.build_control()

        self.assertIsInstance(control, ControlPort)

        status = control.get_safety_status()

        self.assertEqual(status.robot_id, self.robot_id)
        self.assertEqual(status.updated_at, self.timestamp)
        self.assertFalse(status.latched)
        self.assertIs(status.severity, SafetySeverity.INFO)
        self.assertIsNone(status.reason)
        self.assertEqual(self.clock.now_calls, 1)
        self.assertEqual(simulation.applied_commands, [])

    def test_execute_step_forwards_exact_command_and_result(
        self,
    ) -> None:
        pose_after = RobotPose(
            position=Position(x=1, y=2),
            heading=Heading.NORTH,
        )
        control, simulation = self.build_control(
            outcomes=[(pose_after, None)]
        )
        command = self.command(CommandType.MOVE_FORWARD)

        result = control.execute_step(command)

        self.assertIs(result.command, command)
        self.assertIs(result.status, CommandStatus.SUCCESS)
        self.assertEqual(result.pose_before, self.initial_pose)
        self.assertEqual(result.pose_after, pose_after)
        self.assertEqual(
            simulation.applied_commands,
            [command],
        )

    def test_successive_commands_use_latest_confirmed_pose(
        self,
    ) -> None:
        turned_pose = RobotPose(
            position=self.initial_pose.position,
            heading=Heading.EAST,
        )
        moved_pose = RobotPose(
            position=Position(x=2, y=1),
            heading=Heading.EAST,
        )
        control, _ = self.build_control(
            outcomes=[
                (turned_pose, None),
                (moved_pose, None),
            ]
        )

        turn_result = control.execute_step(
            self.command(CommandType.TURN_RIGHT)
        )
        move_result = control.execute_step(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertEqual(
            move_result.pose_before,
            turn_result.pose_after,
        )
        self.assertEqual(move_result.pose_after, moved_pose)

    def test_failed_result_preserves_confirmed_pose(
        self,
    ) -> None:
        recovered_pose = RobotPose(
            position=Position(x=1, y=2),
            heading=Heading.NORTH,
        )
        control, _ = self.build_control(
            outcomes=[
                (self.initial_pose, FailureReason.BLOCKED),
                (recovered_pose, None),
            ]
        )

        failed_result = control.execute_step(
            self.command(CommandType.MOVE_FORWARD)
        )
        recovered_result = control.execute_step(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertIs(
            failed_result.status,
            CommandStatus.FAILED,
        )
        self.assertIs(
            failed_result.failure_reason,
            FailureReason.BLOCKED,
        )
        self.assertEqual(
            failed_result.pose_after,
            self.initial_pose,
        )
        self.assertEqual(
            recovered_result.pose_before,
            self.initial_pose,
        )

    def test_normal_stop_sends_stop_without_advancing_time(
        self,
    ) -> None:
        control, simulation = self.build_control()

        result = control.stop()

        self.assertIsNone(result)
        self.assertEqual(len(simulation.applied_commands), 1)
        self.assertIs(
            simulation.applied_commands[0].command_type,
            CommandType.STOP,
        )
        self.assertEqual(
            simulation.applied_commands[0].robot_id,
            self.robot_id,
        )
        self.assertEqual(simulation.advanced_seconds, 0.0)

    def test_status_inspection_has_no_control_effect(
        self,
    ) -> None:
        control, simulation = self.build_control()
        first_status = control.get_safety_status()

        second_status = control.get_safety_status()

        self.assertIs(first_status, second_status)
        self.assertEqual(self.clock.now_calls, 1)
        self.assertEqual(simulation.applied_commands, [])

    def test_execute_step_rejects_invalid_or_foreign_command(
        self,
    ) -> None:
        control, simulation = self.build_control()

        with self.assertRaises(DomainValidationError):
            control.execute_step(  # type: ignore[arg-type]
                "MOVE_FORWARD"
            )

        with self.assertRaises(DomainValidationError):
            control.execute_step(
                MotionCommand(
                    robot_id="robot_2",
                    command_type=CommandType.STOP,
                )
            )

        self.assertEqual(simulation.applied_commands, [])

    def test_platform_pose_mismatch_is_rejected(
        self,
    ) -> None:
        simulation = MismatchedPoseSimulation(
            world=self.world,
            initial_pose=self.initial_pose,
        )
        control, _ = self.build_control(
            simulation=simulation
        )

        with self.assertRaises(InvariantViolationError):
            control.execute_step(
                self.command(CommandType.STOP)
            )


if __name__ == "__main__":
    unittest.main()
