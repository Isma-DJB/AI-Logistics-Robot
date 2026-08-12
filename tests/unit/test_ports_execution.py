"""Unit tests for Control and Simulation ports."""

import unittest
from datetime import UTC, datetime

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
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports.control_port import ControlPort
from ai_logistics_robot.ports.simulation_port import SimulationPort


class CompatibleControl:
    """Minimal structural implementation of ControlPort."""

    def __init__(
        self,
        result: CommandResult,
        safety_status: SafetyStatus,
    ) -> None:
        self.result = result
        self.safety_status = safety_status
        self.executed_command: MotionCommand | None = None
        self.stop_called = False

    def execute_step(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Record and confirm one command execution."""

        self.executed_command = command
        return self.result

    def stop(self) -> None:
        """Record a normal stop request."""

        self.stop_called = True

    def emergency_stop(
        self,
        reason: FailureReason,
    ) -> SafetyStatus:
        """Latch and return a deterministic critical status."""

        self.safety_status = SafetyStatus(
            robot_id=self.safety_status.robot_id,
            updated_at=self.safety_status.updated_at,
            latched=True,
            severity=SafetySeverity.CRITICAL,
            reason=reason,
        )
        return self.safety_status

    def get_safety_status(self) -> SafetyStatus:
        """Return the current deterministic safety status."""

        return self.safety_status

    def reset_safety_latch(self) -> SafetyStatus:
        """Clear and return the deterministic safety status."""

        self.safety_status = SafetyStatus(
            robot_id=self.safety_status.robot_id,
            updated_at=self.safety_status.updated_at,
            latched=False,
            severity=SafetySeverity.INFO,
        )
        return self.safety_status


class IncompleteControl:
    """Object intentionally missing ControlPort operations."""


class CompatibleSimulation:
    """Minimal structural implementation of SimulationPort."""

    def __init__(
        self,
        world: GridMap,
        result: CommandResult,
    ) -> None:
        self.world = world
        self.result = result
        self.applied_command: MotionCommand | None = None
        self.advanced_seconds = 0.0
        self.reset_called = False

    def reset(self) -> None:
        """Reset recorded deterministic simulation state."""

        self.applied_command = None
        self.advanced_seconds = 0.0
        self.reset_called = True

    def read_world(self) -> GridMap:
        """Return the configured immutable world."""

        return self.world

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Record the command and return its configured result."""

        self.applied_command = command
        return self.result

    def advance_time(self, seconds: float) -> None:
        """Accumulate deterministic simulated time."""

        self.advanced_seconds += seconds


class IncompleteSimulation:
    """Object intentionally missing SimulationPort operations."""


class ExecutionPortTests(unittest.TestCase):
    """Verify execution-port compatibility and typed results."""

    def setUp(self) -> None:
        self.pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.command = MotionCommand(
            robot_id="robot_1",
            command_type=next(iter(CommandType)),
        )
        self.result = CommandResult(
            command=self.command,
            status=CommandStatus.SUCCESS,
            pose_before=self.pose,
            pose_after=self.pose,
        )
        self.safe_status = SafetyStatus(
            robot_id="robot_1",
            updated_at=datetime(2026, 8, 12, tzinfo=UTC),
            latched=False,
            severity=SafetySeverity.INFO,
        )
        self.world = GridMap(
            width=10,
            height=10,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=Position(x=8, y=7),
        )

    def test_control_executes_command_and_normal_stop(self) -> None:
        control = CompatibleControl(
            result=self.result,
            safety_status=self.safe_status,
        )

        self.assertIsInstance(control, ControlPort)
        self.assertIs(control.execute_step(self.command), self.result)
        self.assertIs(control.executed_command, self.command)

        control.stop()

        self.assertTrue(control.stop_called)

    def test_control_confirms_emergency_latch_and_reset(self) -> None:
        control = CompatibleControl(
            result=self.result,
            safety_status=self.safe_status,
        )
        reason = next(iter(FailureReason))

        latched_status = control.emergency_stop(reason)

        self.assertTrue(latched_status.latched)
        self.assertIs(latched_status.reason, reason)
        self.assertIs(control.get_safety_status(), latched_status)

        reset_status = control.reset_safety_latch()

        self.assertFalse(reset_status.latched)
        self.assertIs(reset_status.severity, SafetySeverity.INFO)
        self.assertIsNone(reset_status.reason)

    def test_incomplete_control_is_rejected(self) -> None:
        self.assertNotIsInstance(IncompleteControl(), ControlPort)

    def test_simulation_reads_world_and_applies_command(self) -> None:
        simulation = CompatibleSimulation(
            world=self.world,
            result=self.result,
        )

        self.assertIsInstance(simulation, SimulationPort)
        self.assertIs(simulation.read_world(), self.world)
        self.assertIs(
            simulation.apply_command(self.command),
            self.result,
        )
        self.assertIs(simulation.applied_command, self.command)

    def test_simulation_advances_time_and_resets(self) -> None:
        simulation = CompatibleSimulation(
            world=self.world,
            result=self.result,
        )

        simulation.advance_time(0.25)
        simulation.advance_time(0.75)

        self.assertEqual(simulation.advanced_seconds, 1.0)

        simulation.reset()

        self.assertTrue(simulation.reset_called)
        self.assertEqual(simulation.advanced_seconds, 0.0)
        self.assertIsNone(simulation.applied_command)

    def test_incomplete_simulation_is_rejected(self) -> None:
        self.assertNotIsInstance(
            IncompleteSimulation(),
            SimulationPort,
        )


if __name__ == "__main__":
    unittest.main()