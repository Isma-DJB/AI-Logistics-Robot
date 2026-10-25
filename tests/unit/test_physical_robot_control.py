"""Offline tests for camera-confirmed physical robot control."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoCommandOutcome,
    ArduinoCommandRejectedError,
    ArduinoControllerStatus,
    ArduinoSerialSessionError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoCompletionCode,
    ArduinoDeviceState,
    ArduinoErrorCode,
    ArduinoSafetyState,
    ArduinoSerialProtocolError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialTimeoutError,
    ArduinoSerialTransportError,
)
from ai_logistics_robot.adapters.hardware.physical_robot_control import (
    PhysicalRobotControl,
)
from ai_logistics_robot.domain.commands import MotionCommand
from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    FailureReason,
    Heading,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.ports.control_port import ControlPort


class DeterministicClock:
    """Provide deterministic wall and monotonic time."""

    def __init__(self) -> None:
        self.current_time = datetime(
            2026,
            10,
            25,
            18,
            0,
            tzinfo=UTC,
        )
        self.now_calls = 0

    def now(self) -> datetime:
        """Return the configured wall-clock time."""

        self.now_calls += 1
        return self.current_time

    def monotonic(self) -> float:
        """Return deterministic monotonic time."""

        return 10.0

    def wait_until(self, deadline: float) -> None:
        """Accept a deadline without waiting."""


class ScriptedPerception:
    """Return scripted immutable physical observations."""

    def __init__(
        self,
        snapshots: list[PerceptionSnapshot],
    ) -> None:
        self.snapshots = snapshots
        self.observe_calls = 0

    def observe(self) -> PerceptionSnapshot:
        """Return the next scripted observation."""

        self.observe_calls += 1

        if not self.snapshots:
            raise AssertionError(
                "No scripted perception snapshot remains."
            )

        return self.snapshots.pop(0)


class RecordingArduino:
    """Record supervised calls and return scripted completions."""

    def __init__(
        self,
        completion_codes: list[
            ArduinoCompletionCode
        ] | None = None,
    ) -> None:
        self.completion_codes = (
            []
            if completion_codes is None
            else completion_codes
        )
        self.executed_commands: list[CommandType] = []
        self.rearm_calls = 0
        self.emergency_stop_calls = 0
        self.next_sequence_id = 1

    def execute_command(
        self,
        command_type: CommandType,
    ) -> ArduinoCommandOutcome:
        """Return one matching scripted command outcome."""

        self.executed_commands.append(command_type)
        completion_code = (
            self.completion_codes.pop(0)
            if self.completion_codes
            else ArduinoCompletionCode.OK
        )
        outcome = ArduinoCommandOutcome(
            sequence_id=self.next_sequence_id,
            command_type=command_type,
            completion_code=completion_code,
        )
        self.next_sequence_id += 1
        return outcome

    def emergency_stop(self) -> ArduinoControllerStatus:
        """Record and confirm ESTOPPED/LATCHED."""

        self.emergency_stop_calls += 1
        status = ArduinoControllerStatus(
            sequence_id=self.next_sequence_id,
            device_state=(
                ArduinoDeviceState.EMERGENCY_STOPPED
            ),
            safety_state=ArduinoSafetyState.LATCHED,
        )
        self.next_sequence_id += 1
        return status

    def rearm(self) -> ArduinoControllerStatus:
        """Record and confirm IDLE/SAFE."""

        self.rearm_calls += 1
        status = ArduinoControllerStatus(
            sequence_id=self.next_sequence_id,
            device_state=ArduinoDeviceState.IDLE,
            safety_state=ArduinoSafetyState.SAFE,
        )
        self.next_sequence_id += 1
        return status


class RaisingArduino(RecordingArduino):
    """Raise one configured error for every motion request."""

    def __init__(self, error: Exception) -> None:
        super().__init__()
        self.error = error

    def execute_command(
        self,
        command_type: CommandType,
    ) -> ArduinoCommandOutcome:
        """Record the command and raise the configured error."""

        self.executed_commands.append(command_type)
        raise self.error


class InvalidStatusArduino(RecordingArduino):
    """Return structurally valid but unsafe controller states."""

    def emergency_stop(self) -> ArduinoControllerStatus:
        """Return an invalid unlatched emergency-stop status."""

        self.emergency_stop_calls += 1
        return ArduinoControllerStatus(
            sequence_id=self.next_sequence_id,
            device_state=ArduinoDeviceState.IDLE,
            safety_state=ArduinoSafetyState.SAFE,
        )

    def rearm(self) -> ArduinoControllerStatus:
        """Return an invalid latched rearm status."""

        self.rearm_calls += 1
        return ArduinoControllerStatus(
            sequence_id=self.next_sequence_id,
            device_state=(
                ArduinoDeviceState.EMERGENCY_STOPPED
            ),
            safety_state=ArduinoSafetyState.LATCHED,
        )


class PhysicalRobotControlTests(unittest.TestCase):
    """Verify physical confirmation and safety behavior offline."""

    def setUp(self) -> None:
        self.robot_id = "robot-1"
        self.clock = DeterministicClock()
        self.initial_pose = self.pose(
            heading=Heading.NORTH
        )

    def pose(
        self,
        *,
        x: int = 3,
        y: int = 3,
        heading: Heading = Heading.NORTH,
    ) -> RobotPose:
        """Build one deterministic robot pose."""

        return RobotPose(
            position=Position(x=x, y=y),
            heading=heading,
        )

    def snapshot(
        self,
        pose: RobotPose,
        *,
        robot_id: str | None = None,
        hazard_detected: bool = False,
    ) -> PerceptionSnapshot:
        """Build one deterministic perception snapshot."""

        return PerceptionSnapshot(
            robot_id=(
                self.robot_id
                if robot_id is None
                else robot_id
            ),
            captured_at=self.clock.current_time,
            robot_pose=pose,
            observations=(),
            target_active=False,
            hazard_detected=hazard_detected,
        )

    def command(
        self,
        command_type: CommandType,
        *,
        robot_id: str | None = None,
    ) -> MotionCommand:
        """Build one motion command."""

        return MotionCommand(
            robot_id=(
                self.robot_id
                if robot_id is None
                else robot_id
            ),
            command_type=command_type,
        )

    def build_control(
        self,
        *,
        snapshots: list[PerceptionSnapshot] | None = None,
        completion_codes: list[
            ArduinoCompletionCode
        ] | None = None,
        arduino: RecordingArduino | None = None,
    ) -> tuple[
        PhysicalRobotControl,
        RecordingArduino,
        ScriptedPerception,
    ]:
        """Build Control with inspectable hardware doubles."""

        selected_arduino = (
            RecordingArduino(completion_codes)
            if arduino is None
            else arduino
        )
        perception = ScriptedPerception(
            [] if snapshots is None else snapshots
        )
        control = PhysicalRobotControl(
            robot_id=self.robot_id,
            initial_pose=self.initial_pose,
            arduino=selected_arduino,
            perception=perception,
            clock=self.clock,
        )
        return control, selected_arduino, perception

    def rearm(
        self,
        control: PhysicalRobotControl,
    ) -> None:
        """Explicitly rearm one physical Control."""

        status = control.reset_safety_latch()

        self.assertFalse(status.latched)
        self.assertIs(status.severity, SafetySeverity.INFO)

    def test_satisfies_control_port_and_starts_latched(
        self,
    ) -> None:
        control, arduino, perception = self.build_control()

        self.assertIsInstance(control, ControlPort)
        status = control.get_safety_status()

        self.assertTrue(status.latched)
        self.assertIs(status.severity, SafetySeverity.CRITICAL)
        self.assertIs(
            status.reason,
            FailureReason.SAFETY_LATCHED,
        )
        self.assertEqual(arduino.rearm_calls, 0)
        self.assertEqual(perception.observe_calls, 0)

    def test_latched_control_rejects_motion_without_io(
        self,
    ) -> None:
        control, arduino, perception = self.build_control()
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
        self.assertEqual(arduino.executed_commands, [])
        self.assertEqual(perception.observe_calls, 0)

    def test_rearm_is_confirmed_by_arduino(self) -> None:
        control, arduino, _ = self.build_control()

        status = control.reset_safety_latch()

        self.assertEqual(arduino.rearm_calls, 1)
        self.assertFalse(status.latched)
        self.assertIs(status.severity, SafetySeverity.INFO)
        self.assertIsNone(status.reason)

    def test_confirmed_left_turn_succeeds(self) -> None:
        pose_after = self.pose(heading=Heading.WEST)
        control, arduino, perception = self.build_control(
            snapshots=[
                self.snapshot(self.initial_pose),
                self.snapshot(pose_after),
            ]
        )
        self.rearm(control)
        command = self.command(CommandType.TURN_LEFT)

        result = control.execute_step(command)

        self.assertIs(result.command, command)
        self.assertIs(result.status, CommandStatus.SUCCESS)
        self.assertEqual(result.pose_before, self.initial_pose)
        self.assertEqual(result.pose_after, pose_after)
        self.assertEqual(
            arduino.executed_commands,
            [CommandType.TURN_LEFT],
        )
        self.assertEqual(perception.observe_calls, 2)
        self.assertFalse(
            control.get_safety_status().latched
        )

    def test_confirmed_forward_cell_succeeds(self) -> None:
        pose_after = self.pose(y=4)
        control, arduino, _ = self.build_control(
            snapshots=[
                self.snapshot(self.initial_pose),
                self.snapshot(pose_after),
            ]
        )
        self.rearm(control)

        result = control.execute_step(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertIs(result.status, CommandStatus.SUCCESS)
        self.assertEqual(result.pose_after, pose_after)
        self.assertEqual(
            arduino.executed_commands,
            [CommandType.MOVE_FORWARD],
        )

    def test_stop_remains_available_while_latched(
        self,
    ) -> None:
        control, arduino, perception = self.build_control()

        result = control.execute_step(
            self.command(CommandType.STOP)
        )

        self.assertIs(result.status, CommandStatus.ABORTED)
        self.assertEqual(arduino.executed_commands, [])
        self.assertEqual(perception.observe_calls, 0)

        control.stop()

        self.assertEqual(
            arduino.executed_commands,
            [CommandType.STOP],
        )
        self.assertEqual(perception.observe_calls, 0)
        self.assertTrue(
            control.get_safety_status().latched
        )

    def test_wrong_pose_triggers_emergency_stop(
        self,
    ) -> None:
        unexpected_pose = self.pose(
            x=2,
            heading=Heading.NORTH,
        )
        control, arduino, _ = self.build_control(
            snapshots=[
                self.snapshot(self.initial_pose),
                self.snapshot(unexpected_pose),
            ]
        )
        self.rearm(control)

        result = control.execute_step(
            self.command(CommandType.TURN_LEFT)
        )

        self.assertIs(result.status, CommandStatus.ABORTED)
        self.assertIs(
            result.failure_reason,
            FailureReason.EMERGENCY_STOP,
        )
        self.assertEqual(result.pose_after, unexpected_pose)
        self.assertEqual(arduino.emergency_stop_calls, 1)
        self.assertTrue(
            control.get_safety_status().latched
        )

    def test_preflight_hazard_prevents_motion(
        self,
    ) -> None:
        control, arduino, perception = self.build_control(
            snapshots=[
                self.snapshot(
                    self.initial_pose,
                    hazard_detected=True,
                )
            ]
        )
        self.rearm(control)

        result = control.execute_step(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertIs(result.status, CommandStatus.ABORTED)
        self.assertIs(
            result.failure_reason,
            FailureReason.EMERGENCY_STOP,
        )
        self.assertEqual(arduino.executed_commands, [])
        self.assertEqual(arduino.emergency_stop_calls, 1)
        self.assertEqual(perception.observe_calls, 1)

    def test_completion_codes_are_normalized(self) -> None:
        cases = (
            (
                ArduinoCompletionCode.BLOCKED,
                CommandStatus.FAILED,
                FailureReason.BLOCKED,
                False,
            ),
            (
                ArduinoCompletionCode.TIMEOUT,
                CommandStatus.TIMEOUT,
                FailureReason.TIMEOUT,
                False,
            ),
            (
                ArduinoCompletionCode.COMMUNICATION_LOSS,
                CommandStatus.ABORTED,
                FailureReason.COMMUNICATION_LOSS,
                True,
            ),
            (
                ArduinoCompletionCode.EMERGENCY_STOP,
                CommandStatus.ABORTED,
                FailureReason.EMERGENCY_STOP,
                True,
            ),
        )

        for code, expected_status, reason, latched in cases:
            with self.subTest(code=code):
                control, _, perception = self.build_control(
                    snapshots=[
                        self.snapshot(self.initial_pose)
                    ],
                    completion_codes=[code],
                )
                self.rearm(control)

                result = control.execute_step(
                    self.command(
                        CommandType.MOVE_FORWARD
                    )
                )

                self.assertIs(
                    result.status,
                    expected_status,
                )
                self.assertIs(
                    result.failure_reason,
                    reason,
                )
                self.assertEqual(
                    result.pose_after,
                    self.initial_pose,
                )
                self.assertEqual(
                    perception.observe_calls,
                    1,
                )
                self.assertEqual(
                    control.get_safety_status().latched,
                    latched,
                )

    def test_transport_timeout_is_normalized_and_latched(
        self,
    ) -> None:
        arduino = RaisingArduino(
            ArduinoSerialTimeoutError("timed out")
        )
        control, _, perception = self.build_control(
            snapshots=[
                self.snapshot(self.initial_pose)
            ],
            arduino=arduino,
        )
        self.rearm(control)

        result = control.execute_step(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertIs(result.status, CommandStatus.TIMEOUT)
        self.assertIs(
            result.failure_reason,
            FailureReason.TIMEOUT,
        )
        self.assertEqual(result.pose_after, self.initial_pose)
        self.assertEqual(perception.observe_calls, 1)

        safety = control.get_safety_status()
        self.assertTrue(safety.latched)
        self.assertIs(
            safety.reason,
            FailureReason.TIMEOUT,
        )

    def test_serial_failures_become_communication_loss(
        self,
    ) -> None:
        errors = (
            ArduinoSerialTransportError("connection lost"),
            ArduinoSerialSessionError("invalid exchange"),
            ArduinoSerialProtocolError("invalid message"),
        )

        for error in errors:
            with self.subTest(error=type(error).__name__):
                arduino = RaisingArduino(error)
                control, _, perception = self.build_control(
                    snapshots=[
                        self.snapshot(self.initial_pose)
                    ],
                    arduino=arduino,
                )
                self.rearm(control)

                result = control.execute_step(
                    self.command(
                        CommandType.MOVE_FORWARD
                    )
                )

                self.assertIs(
                    result.status,
                    CommandStatus.ABORTED,
                )
                self.assertIs(
                    result.failure_reason,
                    FailureReason.COMMUNICATION_LOSS,
                )
                self.assertEqual(
                    perception.observe_calls,
                    1,
                )

                safety = control.get_safety_status()
                self.assertTrue(safety.latched)
                self.assertIs(
                    safety.reason,
                    FailureReason.COMMUNICATION_LOSS,
                )

    def test_safety_latched_rejection_is_preserved(
        self,
    ) -> None:
        arduino = RaisingArduino(
            ArduinoCommandRejectedError(
                sequence_id=9,
                error_code=(
                    ArduinoErrorCode.SAFETY_LATCHED
                ),
            )
        )
        control, _, _ = self.build_control(
            snapshots=[
                self.snapshot(self.initial_pose)
            ],
            arduino=arduino,
        )
        self.rearm(control)

        result = control.execute_step(
            self.command(CommandType.TURN_LEFT)
        )

        self.assertIs(result.status, CommandStatus.ABORTED)
        self.assertIs(
            result.failure_reason,
            FailureReason.SAFETY_LATCHED,
        )
        self.assertTrue(
            control.get_safety_status().latched
        )

    def test_other_rejection_becomes_internal_error(
        self,
    ) -> None:
        arduino = RaisingArduino(
            ArduinoCommandRejectedError(
                sequence_id=10,
                error_code=ArduinoErrorCode.BAD_COMMAND,
            )
        )
        control, _, _ = self.build_control(
            snapshots=[
                self.snapshot(self.initial_pose)
            ],
            arduino=arduino,
        )
        self.rearm(control)

        result = control.execute_step(
            self.command(CommandType.TURN_RIGHT)
        )

        self.assertIs(result.status, CommandStatus.FAILED)
        self.assertIs(
            result.failure_reason,
            FailureReason.INTERNAL_ERROR,
        )
        self.assertFalse(
            control.get_safety_status().latched
        )

    def test_explicit_emergency_stop_latches_before_io(
        self,
    ) -> None:
        control, arduino, _ = self.build_control()
        self.rearm(control)

        status = control.emergency_stop(
            FailureReason.COMMUNICATION_LOSS
        )

        self.assertTrue(status.latched)
        self.assertIs(
            status.reason,
            FailureReason.COMMUNICATION_LOSS,
        )
        self.assertEqual(arduino.emergency_stop_calls, 1)

    def test_invalid_rearm_confirmation_preserves_latch(
        self,
    ) -> None:
        arduino = InvalidStatusArduino()
        control, _, _ = self.build_control(
            arduino=arduino
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "IDLE/SAFE",
        ):
            control.reset_safety_latch()

        safety = control.get_safety_status()
        self.assertTrue(safety.latched)
        self.assertIs(
            safety.reason,
            FailureReason.SAFETY_LATCHED,
        )

    def test_invalid_estop_confirmation_preserves_latch(
        self,
    ) -> None:
        arduino = InvalidStatusArduino()
        control, _, _ = self.build_control(
            arduino=arduino
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "ESTOPPED/LATCHED",
        ):
            control.emergency_stop(
                FailureReason.EMERGENCY_STOP
            )

        safety = control.get_safety_status()
        self.assertTrue(safety.latched)
        self.assertIs(
            safety.reason,
            FailureReason.EMERGENCY_STOP,
        )

    def test_invalid_and_foreign_commands_are_atomic(
        self,
    ) -> None:
        control, arduino, perception = self.build_control()
        self.rearm(control)

        with self.assertRaises(DomainValidationError):
            control.execute_step(
                "MOVE_FORWARD"  # type: ignore[arg-type]
            )

        with self.assertRaises(DomainValidationError):
            control.execute_step(
                self.command(
                    CommandType.STOP,
                    robot_id="robot-2",
                )
            )

        self.assertEqual(arduino.executed_commands, [])
        self.assertEqual(perception.observe_calls, 0)


if __name__ == "__main__":
    unittest.main()