"""Unit tests for SafeRobotControl safety-latch rules."""

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


class MutableClock:
    """Provide deterministic controllable wall-clock time."""

    def __init__(self) -> None:
        self.current_time = datetime(
            2026,
            8,
            15,
            10,
            0,
            tzinfo=UTC,
        )
        self.now_calls = 0

    def now(self) -> datetime:
        """Return the configured timestamp."""

        self.now_calls += 1
        return self.current_time

    def monotonic(self) -> float:
        """Return deterministic monotonic time."""

        return 10.0

    def wait_until(self, deadline: float) -> None:
        """Accept a deadline without blocking."""


class SafetySimulation:
    """Record commands and return scripted confirmed outcomes."""

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
        self.reset_calls = 0
        self.advanced_seconds = 0.0

    def reset(self) -> None:
        """Restore the initial fake-platform state."""

        self.current_pose = self.initial_pose
        self.applied_commands.clear()
        self.reset_calls += 1
        self.advanced_seconds = 0.0

    def read_world(self) -> GridMap:
        """Return the configured immutable world."""

        return self.world

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Return the next scripted command result."""

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
        """Record simulated time without using it for commands."""

        self.advanced_seconds += seconds


class RaisingStopSimulation(SafetySimulation):
    """Raise while attempting to apply the priority STOP."""

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Record the command and simulate platform failure."""

        self.applied_commands.append(command)
        raise RuntimeError("platform stop failed")


class InvalidResultSimulation(SafetySimulation):
    """Return an invalid value from the simulation port."""

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Return a value that violates SimulationPort."""

        self.applied_commands.append(command)
        return "invalid"  # type: ignore[return-value]


class CopiedCommandSimulation(SafetySimulation):
    """Return a result containing a copied command value."""

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Replace the supplied command with an equal copy."""

        self.applied_commands.append(command)
        copied_command = MotionCommand(
            robot_id=command.robot_id,
            command_type=command.command_type,
        )

        return CommandResult(
            command=copied_command,
            status=CommandStatus.SUCCESS,
            pose_before=self.current_pose,
            pose_after=self.current_pose,
        )


class SafeRobotControlSafetyTests(unittest.TestCase):
    """Verify priority stop, latch, and manual-rearm behavior."""

    def setUp(self) -> None:
        self.robot_id = "robot_1"
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
        self.clock = MutableClock()

    def build_control(
        self,
        *,
        simulation: SafetySimulation | None = None,
        outcomes: list[
            tuple[RobotPose, FailureReason | None]
        ] | None = None,
    ) -> tuple[SafeRobotControl, SafetySimulation]:
        """Build Control with an inspectable simulation fake."""

        selected_simulation = (
            SafetySimulation(
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

    def test_emergency_stop_latches_and_sends_priority_stop(
        self,
    ) -> None:
        control, simulation = self.build_control()
        emergency_time = datetime(
            2026,
            8,
            15,
            10,
            1,
            tzinfo=UTC,
        )
        self.clock.current_time = emergency_time

        status = control.emergency_stop(
            FailureReason.EMERGENCY_STOP
        )

        self.assertIs(control.get_safety_status(), status)
        self.assertTrue(status.latched)
        self.assertIs(status.severity, SafetySeverity.CRITICAL)
        self.assertIs(
            status.reason,
            FailureReason.EMERGENCY_STOP,
        )
        self.assertEqual(status.updated_at, emergency_time)
        self.assertEqual(len(simulation.applied_commands), 1)
        self.assertIs(
            simulation.applied_commands[0].command_type,
            CommandType.STOP,
        )
        self.assertEqual(simulation.advanced_seconds, 0.0)

    def test_latched_control_rejects_command_without_platform_call(
        self,
    ) -> None:
        control, simulation = self.build_control()
        control.emergency_stop(FailureReason.EMERGENCY_STOP)
        simulation.applied_commands.clear()
        command = self.command(CommandType.MOVE_FORWARD)

        result = control.execute_step(command)

        self.assertIs(result.command, command)
        self.assertIs(result.status, CommandStatus.ABORTED)
        self.assertIs(
            result.failure_reason,
            FailureReason.SAFETY_LATCHED,
        )
        self.assertEqual(result.pose_before, self.initial_pose)
        self.assertEqual(result.pose_after, self.initial_pose)
        self.assertEqual(simulation.applied_commands, [])

    def test_normal_stop_remains_available_while_latched(
        self,
    ) -> None:
        control, simulation = self.build_control()
        control.emergency_stop(FailureReason.EMERGENCY_STOP)
        simulation.applied_commands.clear()

        control.stop()

        self.assertEqual(len(simulation.applied_commands), 1)
        self.assertIs(
            simulation.applied_commands[0].command_type,
            CommandType.STOP,
        )
        self.assertTrue(
            control.get_safety_status().latched
        )

    def test_manual_rearm_does_not_reset_or_move_platform(
        self,
    ) -> None:
        control, simulation = self.build_control()
        control.emergency_stop(FailureReason.EMERGENCY_STOP)
        simulation.applied_commands.clear()
        rearm_time = datetime(
            2026,
            8,
            15,
            10,
            2,
            tzinfo=UTC,
        )
        self.clock.current_time = rearm_time

        status = control.reset_safety_latch()

        self.assertFalse(status.latched)
        self.assertIs(status.severity, SafetySeverity.INFO)
        self.assertIsNone(status.reason)
        self.assertEqual(status.updated_at, rearm_time)
        self.assertEqual(simulation.reset_calls, 0)
        self.assertEqual(simulation.applied_commands, [])

        result = control.execute_step(
            self.command(CommandType.STOP)
        )

        self.assertIs(result.status, CommandStatus.SUCCESS)
        self.assertEqual(len(simulation.applied_commands), 1)

    def test_safety_cycle_preserves_latest_confirmed_pose(
        self,
    ) -> None:
        moved_pose = RobotPose(
            position=Position(x=1, y=2),
            heading=Heading.NORTH,
        )
        recovered_pose = RobotPose(
            position=Position(x=1, y=3),
            heading=Heading.NORTH,
        )
        control, _ = self.build_control(
            outcomes=[
                (moved_pose, None),
                (moved_pose, None),
                (recovered_pose, None),
            ]
        )

        first_result = control.execute_step(
            self.command(CommandType.MOVE_FORWARD)
        )
        control.emergency_stop(FailureReason.EMERGENCY_STOP)
        control.reset_safety_latch()
        recovered_result = control.execute_step(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertEqual(first_result.pose_after, moved_pose)
        self.assertEqual(
            recovered_result.pose_before,
            moved_pose,
        )
        self.assertEqual(
            recovered_result.pose_after,
            recovered_pose,
        )

    def test_invalid_emergency_reason_is_atomic(
        self,
    ) -> None:
        control, simulation = self.build_control()

        with self.assertRaises(DomainValidationError):
            control.emergency_stop(  # type: ignore[arg-type]
                "EMERGENCY_STOP"
            )

        status = control.get_safety_status()

        self.assertFalse(status.latched)
        self.assertIsNone(status.reason)
        self.assertEqual(simulation.applied_commands, [])
        self.assertEqual(self.clock.now_calls, 1)

    def test_platform_stop_failure_leaves_safety_latched(
        self,
    ) -> None:
        simulation = RaisingStopSimulation(
            world=self.world,
            initial_pose=self.initial_pose,
        )
        control, _ = self.build_control(
            simulation=simulation
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "platform stop failed",
        ):
            control.emergency_stop(
                FailureReason.COMMUNICATION_LOSS
            )

        status = control.get_safety_status()

        self.assertTrue(status.latched)
        self.assertIs(
            status.reason,
            FailureReason.COMMUNICATION_LOSS,
        )
        self.assertEqual(len(simulation.applied_commands), 1)
        self.assertIs(
            simulation.applied_commands[0].command_type,
            CommandType.STOP,
        )

    def test_constructor_rejects_invalid_dependencies(
        self,
    ) -> None:
        simulation = SafetySimulation(
            world=self.world,
            initial_pose=self.initial_pose,
        )

        with self.assertRaises(DomainValidationError):
            SafeRobotControl(
                robot_id=" ",
                initial_pose=self.initial_pose,
                simulation=simulation,
                clock=self.clock,
            )

        with self.assertRaises(DomainValidationError):
            SafeRobotControl(  # type: ignore[arg-type]
                robot_id=self.robot_id,
                initial_pose=Position(x=1, y=1),
                simulation=simulation,
                clock=self.clock,
            )

        with self.assertRaises(DomainValidationError):
            SafeRobotControl(  # type: ignore[arg-type]
                robot_id=self.robot_id,
                initial_pose=self.initial_pose,
                simulation=object(),
                clock=self.clock,
            )

        with self.assertRaises(DomainValidationError):
            SafeRobotControl(  # type: ignore[arg-type]
                robot_id=self.robot_id,
                initial_pose=self.initial_pose,
                simulation=simulation,
                clock=object(),
            )

    def test_invalid_platform_result_is_rejected(
        self,
    ) -> None:
        simulation = InvalidResultSimulation(
            world=self.world,
            initial_pose=self.initial_pose,
        )
        control, _ = self.build_control(
            simulation=simulation
        )

        with self.assertRaises(DomainValidationError):
            control.execute_step(
                self.command(CommandType.STOP)
            )

    def test_copied_result_command_is_rejected(
        self,
    ) -> None:
        simulation = CopiedCommandSimulation(
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
