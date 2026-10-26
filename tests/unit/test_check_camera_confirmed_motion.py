"""Offline guards for the camera-confirmed motion diagnostic."""

import unittest
from contextlib import redirect_stderr
from datetime import UTC, datetime
from io import StringIO
from unittest.mock import patch

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoControllerStatus,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoDeviceState,
    ArduinoSafetyState,
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
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.safety import SafetyStatus
from tools.check_camera_confirmed_motion import (
    CameraConfirmedMotionDiagnosticError,
    _execute_control_cycle,
    _require_explicit_authorization,
    _require_motion_command,
    _run_diagnostic,
    build_parser,
    main,
)


class FakeTransportContext:
    """Record deterministic serial context entry and exit."""

    def __init__(self) -> None:
        self.transport = object()
        self.entered = False
        self.exited = False

    def __enter__(self) -> object:
        """Return one inert transport token."""

        self.entered = True
        return self.transport

    def __exit__(
        self,
        exception_type: object,
        exception: object,
        traceback: object,
    ) -> None:
        """Record context cleanup."""

        self.exited = True


class FakeArduinoClient:
    """Expose only setup operations used by the live runner."""

    def __init__(
        self,
        statuses: list[ArduinoControllerStatus],
    ) -> None:
        self.statuses = statuses
        self.ping_calls = 0
        self.status_calls = 0

    def ping(self) -> int:
        """Return one deterministic ping sequence."""

        self.ping_calls += 1
        return 41

    def request_status(self) -> ArduinoControllerStatus:
        """Return the next scripted controller status."""

        self.status_calls += 1

        if not self.statuses:
            raise AssertionError(
                "No scripted controller status remains."
            )

        return self.statuses.pop(0)


class FakeFusedPerception:
    """Return one deterministic initial physical snapshot."""

    def __init__(
        self,
        snapshot: PerceptionSnapshot,
    ) -> None:
        self.snapshot = snapshot
        self.observe_calls = 0

    def observe(self) -> PerceptionSnapshot:
        """Return the configured snapshot."""

        self.observe_calls += 1
        return self.snapshot


class RecordingControl:
    """Record one guarded diagnostic control cycle."""

    def __init__(
        self,
        *,
        command_status: CommandStatus = (
            CommandStatus.SUCCESS
        ),
        failure_reason: FailureReason | None = None,
        raise_during_execute: bool = False,
    ) -> None:
        self.pose = RobotPose(
            position=Position(x=3, y=3),
            heading=Heading.NORTH,
        )
        self.command_status = command_status
        self.failure_reason = failure_reason
        self.raise_during_execute = raise_during_execute
        self.calls: list[str] = []
        self.latched = True

    def execute_step(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Return one configured command result."""

        self.calls.append("execute")

        if self.raise_during_execute:
            raise RuntimeError("execution failed")

        return CommandResult(
            command=command,
            status=self.command_status,
            pose_before=self.pose,
            pose_after=self.pose,
            failure_reason=self.failure_reason,
        )

    def stop(self) -> None:
        """Record normal stop."""

        self.calls.append("stop")

    def emergency_stop(
        self,
        reason: FailureReason,
    ) -> SafetyStatus:
        """Record and confirm final safety latch."""

        self.calls.append("estop")
        self.latched = True
        return self._status(
            latched=True,
            reason=reason,
        )

    def get_safety_status(self) -> SafetyStatus:
        """Return current deterministic safety state."""

        return self._status(
            latched=self.latched,
            reason=(
                FailureReason.EMERGENCY_STOP
                if self.latched
                else None
            ),
        )

    def reset_safety_latch(self) -> SafetyStatus:
        """Record and confirm safe rearm."""

        self.calls.append("rearm")
        self.latched = False
        return self._status(
            latched=False,
            reason=None,
        )

    def _status(
        self,
        *,
        latched: bool,
        reason: FailureReason | None,
    ) -> SafetyStatus:
        """Build one valid deterministic safety status."""

        return SafetyStatus(
            robot_id="robot-1",
            updated_at=datetime(
                2026,
                10,
                25,
                19,
                0,
                tzinfo=UTC,
            ),
            latched=latched,
            severity=(
                SafetySeverity.CRITICAL
                if latched
                else SafetySeverity.INFO
            ),
            reason=reason,
        )


class CameraConfirmedMotionDiagnosticTests(
    unittest.TestCase
):
    """Verify parser defaults and mandatory authorization."""

    def test_parser_exposes_guarded_defaults(self) -> None:
        arguments = build_parser().parse_args(
            [
                "--url",
                "http://camera.test",
                "--port",
                "COM3",
                "--command",
                "TURN_LEFT",
            ]
        )

        self.assertEqual(
            arguments.url,
            "http://camera.test",
        )
        self.assertEqual(arguments.port, "COM3")
        self.assertEqual(arguments.command, "TURN_LEFT")
        self.assertFalse(arguments.confirm_ground_test)
        self.assertIsNone(arguments.ambient_light_raw)
        self.assertEqual(
            arguments.ambient_calibration_samples,
            10,
        )
        self.assertEqual(
            arguments.maximum_ambient_light_range,
            10,
        )
        self.assertEqual(
            arguments.frontal_obstacle_distance_mm,
            200,
        )
        self.assertEqual(
            arguments.target_activation_delta,
            50,
        )
        self.assertEqual(
            arguments.target_release_delta,
            30,
        )
        self.assertEqual(
            arguments.required_stable_readings,
            5,
        )
        self.assertEqual(arguments.baud_rate, 115200)
        self.assertEqual(arguments.serial_timeout, 5.0)
        self.assertEqual(arguments.camera_timeout, 15.0)

    def test_accepts_supported_commands(self) -> None:
        cases = (
            ("MOVE_FORWARD", CommandType.MOVE_FORWARD),
            ("TURN_LEFT", CommandType.TURN_LEFT),
            ("TURN_RIGHT", CommandType.TURN_RIGHT),
        )

        for value, expected in cases:
            with self.subTest(value=value):
                self.assertIs(
                    _require_motion_command(value),
                    expected,
                )

    def test_rejects_stop_and_invalid_commands(self) -> None:
        for value in ("STOP", "BACKWARD", "", None):
            with self.subTest(value=value):
                with self.assertRaises(
                    DomainValidationError
                ):
                    _require_motion_command(value)

    def test_requires_exact_authorization(self) -> None:
        _require_explicit_authorization(True)

        for value in (False, None, 1, "true"):
            with self.subTest(value=value):
                with self.assertRaises(
                    CameraConfirmedMotionDiagnosticError
                ):
                    _require_explicit_authorization(value)

    def test_missing_authorization_fails_before_hardware(
        self,
    ) -> None:
        error_output = StringIO()

        with redirect_stderr(error_output):
            exit_code = main(
                [
                    "--url",
                    "http://camera.test",
                    "--port",
                    "COM3",
                    "--command",
                    "MOVE_FORWARD",
                ]
            )

        self.assertEqual(exit_code, 2)
        self.assertIn(
            "ground-test authorization is required",
            error_output.getvalue(),
        )

    def test_control_cycle_rearms_executes_and_estops(
        self,
    ) -> None:
        control = RecordingControl()
        command = MotionCommand(
            robot_id="robot-1",
            command_type=CommandType.TURN_LEFT,
        )

        result = _execute_control_cycle(
            control=control,
            command=command,
        )

        self.assertEqual(
            control.calls,
            ["rearm", "execute", "estop"],
        )
        self.assertIs(
            result.command_result.status,
            CommandStatus.SUCCESS,
        )
        self.assertTrue(
            result.final_safety_status.latched
        )

    def test_control_cycle_estops_after_execution_error(
        self,
    ) -> None:
        control = RecordingControl(
            raise_during_execute=True
        )
        command = MotionCommand(
            robot_id="robot-1",
            command_type=CommandType.MOVE_FORWARD,
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "execution failed",
        ):
            _execute_control_cycle(
                control=control,
                command=command,
            )

        self.assertEqual(
            control.calls,
            ["rearm", "execute", "estop"],
        )

    def test_unconfirmed_command_is_rejected_after_estop(
        self,
    ) -> None:
        control = RecordingControl(
            command_status=CommandStatus.FAILED,
            failure_reason=FailureReason.BLOCKED,
        )
        command = MotionCommand(
            robot_id="robot-1",
            command_type=CommandType.MOVE_FORWARD,
        )

        with self.assertRaisesRegex(
            CameraConfirmedMotionDiagnosticError,
            "status=FAILED, reason=BLOCKED",
        ):
            _execute_control_cycle(
                control=control,
                command=command,
            )

        self.assertEqual(
            control.calls,
            ["rearm", "execute", "estop"],
        )

    def test_authorized_runner_uses_guarded_cycle_offline(
        self,
    ) -> None:
        timestamp = datetime(
            2026,
            10,
            25,
            19,
            30,
            tzinfo=UTC,
        )
        pose = RobotPose(
            position=Position(x=3, y=3),
            heading=Heading.NORTH,
        )
        snapshot = PerceptionSnapshot(
            robot_id="robot-1",
            captured_at=timestamp,
            robot_pose=pose,
            observations=(),
            target_active=False,
            hazard_detected=False,
        )
        initial_status = ArduinoControllerStatus(
            sequence_id=2,
            device_state=(
                ArduinoDeviceState.EMERGENCY_STOPPED
            ),
            safety_state=ArduinoSafetyState.LATCHED,
        )
        final_status = ArduinoControllerStatus(
            sequence_id=9,
            device_state=(
                ArduinoDeviceState.EMERGENCY_STOPPED
            ),
            safety_state=ArduinoSafetyState.LATCHED,
        )
        client = FakeArduinoClient(
            [initial_status, final_status]
        )
        perception = FakeFusedPerception(snapshot)
        control = RecordingControl()
        transport_context = FakeTransportContext()
        arguments = build_parser().parse_args(
            [
                "--url",
                "http://camera.test",
                "--port",
                "COM3",
                "--command",
                "TURN_LEFT",
                "--confirm-ground-test",
                "--ambient-light-raw",
                "300",
            ]
        )

        with (
            patch(
                "tools.check_camera_confirmed_motion."
                "open_arduino_serial_transport",
                return_value=transport_context,
            ) as open_transport,
            patch(
                "tools.check_camera_confirmed_motion."
                "ArduinoSerialClient",
                return_value=client,
            ) as client_constructor,
            patch(
                "tools.check_camera_confirmed_motion."
                "_build_fused_perception",
                return_value=perception,
            ) as build_perception,
            patch(
                "tools.check_camera_confirmed_motion."
                "PhysicalRobotControl",
                return_value=control,
            ) as control_constructor,
        ):
            result = _run_diagnostic(arguments)

        self.assertTrue(transport_context.entered)
        self.assertTrue(transport_context.exited)
        self.assertEqual(client.ping_calls, 1)
        self.assertEqual(client.status_calls, 2)
        self.assertEqual(perception.observe_calls, 1)
        self.assertEqual(
            control.calls,
            ["rearm", "execute", "estop"],
        )
        self.assertEqual(result.ping_sequence, 41)
        self.assertIs(result.initial_status, initial_status)
        self.assertIs(result.final_status, final_status)
        self.assertEqual(result.ambient_light_raw, 300)
        self.assertIsNone(result.ambient_calibration)
        self.assertIs(
            result.cycle.command_result.status,
            CommandStatus.SUCCESS,
        )
        open_transport.assert_called_once_with(
            port="COM3",
            baud_rate=115200,
            read_timeout_seconds=5.0,
            write_timeout_seconds=5.0,
        )
        client_constructor.assert_called_once_with(
            transport=transport_context.transport,
        )
        build_perception.assert_called_once_with(
            arguments,
            client=client,
            ambient_light_raw=300,
        )
        control_constructor.assert_called_once()


if __name__ == "__main__":
    unittest.main()