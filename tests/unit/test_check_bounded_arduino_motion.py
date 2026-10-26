"""Offline tests for the bounded Arduino motion diagnostic."""

import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from unittest.mock import patch

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoCommandOutcome,
    ArduinoControllerStatus,
    ArduinoSensorSnapshot,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoCompletionCode,
    ArduinoDeviceState,
    ArduinoSafetyState,
    encode_emergency_stop,
    encode_motion_command,
    encode_ping,
    encode_rearm,
    encode_sensor_request,
    encode_status_request,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialTransport,
)
from ai_logistics_robot.domain.enums import CommandType
from ai_logistics_robot.domain.errors import DomainValidationError
from tools.check_bounded_arduino_motion import (
    BoundedArduinoMotionDiagnosticError,
    _require_explicit_authorization,
    _require_latched_status,
    _require_positive_distance_threshold,
    _require_safe_preflight,
    _require_successful_completion,
    build_parser,
    main,
)


class _ScriptedSerialPort:
    """Provide deterministic serial responses without hardware."""

    def __init__(self, *responses: bytes) -> None:
        """Retain scripted responses and transmitted requests."""

        self._responses = list(responses)
        self.requests: list[bytes] = []
        self._is_open = True

    @property
    def is_open(self) -> bool:
        """Return whether the fake connection remains open."""

        return self._is_open

    def write(self, data: bytes) -> int:
        """Record one complete request."""

        self.requests.append(data)
        return len(data)

    def flush(self) -> None:
        """Accept output flushing without side effects."""

    def readline(self) -> bytes:
        """Return the next scripted response."""

        if not self._responses:
            raise AssertionError(
                "No scripted serial response remains."
            )

        return self._responses.pop(0)

    def reset_input_buffer(self) -> None:
        """Accept input-buffer reset without side effects."""

    def close(self) -> None:
        """Record connection closure."""

        self._is_open = False


def _safe_snapshot(
    *,
    left_ir_raw: int = 1,
    right_ir_raw: int = 1,
    line_raw: int = 0,
    distance_mm: int | None = 350,
) -> ArduinoSensorSnapshot:
    """Create one configurable preflight sensor snapshot."""

    return ArduinoSensorSnapshot(
        sequence_id=3,
        left_ir_raw=left_ir_raw,
        right_ir_raw=right_ir_raw,
        line_raw=line_raw,
        distance_mm=distance_mm,
        light_raw=400,
    )


def _latched_status(
    sequence_id: int = 2,
) -> ArduinoControllerStatus:
    """Create one confirmed emergency-stop status."""

    return ArduinoControllerStatus(
        sequence_id=sequence_id,
        device_state=ArduinoDeviceState.EMERGENCY_STOPPED,
        safety_state=ArduinoSafetyState.LATCHED,
    )


def _successful_outcome() -> ArduinoCommandOutcome:
    """Create one successful bounded forward outcome."""

    return ArduinoCommandOutcome(
        sequence_id=5,
        command_type=CommandType.MOVE_FORWARD,
        completion_code=ArduinoCompletionCode.OK,
    )


def _successful_serial_port() -> _ScriptedSerialPort:
    """Create one complete successful diagnostic exchange."""

    return _ScriptedSerialPort(
        b"ALR|1|PONG|1\n",
        b"ALR|1|STATUS|2|ESTOPPED|LATCHED\n",
        b"ALR|1|SENSORS|3|1|1|0|350|400\n",
        b"ALR|1|STATUS|4|IDLE|SAFE\n",
        b"ALR|1|ACK|5|MOVE_FORWARD\n",
        b"ALR|1|DONE|5|OK\n",
        b"ALR|1|STATUS|6|ESTOPPED|LATCHED\n",
    )


class BoundedArduinoMotionDiagnosticTests(
    unittest.TestCase
):
    """Verify authorization, preflight, execution, and cleanup."""

    def test_parser_keeps_motion_unauthorized_by_default(
        self,
    ) -> None:
        arguments = build_parser().parse_args(
            [
                "--port",
                "COM3",
            ]
        )

        self.assertEqual(arguments.port, "COM3")
        self.assertFalse(arguments.confirm_wheels_lifted)
        self.assertEqual(
            arguments.frontal_obstacle_distance_mm,
            200,
        )
        self.assertEqual(arguments.baud_rate, 115200)
        self.assertEqual(arguments.timeout, 2.0)

    def test_explicit_authorization_is_required(self) -> None:
        _require_explicit_authorization(True)

        for value in (False, None, 1, "true"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(
                    BoundedArduinoMotionDiagnosticError,
                    "confirm-wheels-lifted",
                ):
                    _require_explicit_authorization(value)

    def test_positive_distance_threshold_is_required(
        self,
    ) -> None:
        self.assertEqual(
            _require_positive_distance_threshold(200),
            200,
        )

        for value in (True, 0, -1, 1.5, "200"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "positive integer",
                ):
                    _require_positive_distance_threshold(value)

    def test_latched_status_is_required(self) -> None:
        _require_latched_status(
            _latched_status(),
            stage="before",
        )

        unsafe_status = ArduinoControllerStatus(
            sequence_id=2,
            device_state=ArduinoDeviceState.IDLE,
            safety_state=ArduinoSafetyState.SAFE,
        )

        with self.assertRaisesRegex(
            BoundedArduinoMotionDiagnosticError,
            "ESTOPPED/LATCHED",
        ):
            _require_latched_status(
                unsafe_status,
                stage="before",
            )

    def test_safe_preflight_is_accepted(self) -> None:
        _require_safe_preflight(
            _safe_snapshot(),
            frontal_obstacle_distance_mm=200,
        )

    def test_unsafe_preflight_is_rejected(self) -> None:
        cases = (
            _safe_snapshot(left_ir_raw=0),
            _safe_snapshot(right_ir_raw=0),
            _safe_snapshot(line_raw=1),
            _safe_snapshot(distance_mm=None),
            _safe_snapshot(distance_mm=200),
        )

        for snapshot in cases:
            with self.subTest(snapshot=snapshot):
                with self.assertRaises(
                    BoundedArduinoMotionDiagnosticError
                ):
                    _require_safe_preflight(
                        snapshot,
                        frontal_obstacle_distance_mm=200,
                    )

    def test_successful_forward_completion_is_required(
        self,
    ) -> None:
        _require_successful_completion(
            _successful_outcome()
        )

        blocked = ArduinoCommandOutcome(
            sequence_id=5,
            command_type=CommandType.MOVE_FORWARD,
            completion_code=ArduinoCompletionCode.BLOCKED,
        )

        with self.assertRaisesRegex(
            BoundedArduinoMotionDiagnosticError,
            "BLOCKED",
        ):
            _require_successful_completion(blocked)

    def test_missing_authorization_fails_before_hardware_access(
        self,
    ) -> None:
        error_output = StringIO()

        with (
            patch(
                "tools.check_bounded_arduino_motion."
                "open_arduino_serial_transport"
            ) as open_transport,
            redirect_stderr(error_output),
        ):
            exit_code = main(
                [
                    "--port",
                    "COM3",
                ]
            )

        self.assertEqual(exit_code, 2)
        open_transport.assert_not_called()
        self.assertIn(
            "confirm-wheels-lifted",
            error_output.getvalue(),
        )

    def test_successful_diagnostic_uses_exact_exchange(
        self,
    ) -> None:
        serial_port = _successful_serial_port()
        transport = ArduinoSerialTransport(
            serial_port=serial_port
        )
        output = StringIO()

        with (
            patch(
                "tools.check_bounded_arduino_motion."
                "open_arduino_serial_transport",
                return_value=transport,
            ),
            redirect_stdout(output),
        ):
            exit_code = main(
                [
                    "--port",
                    "COM3",
                    "--confirm-wheels-lifted",
                ]
            )

        self.assertEqual(exit_code, 0)
        self.assertFalse(serial_port.is_open)
        self.assertEqual(
            serial_port.requests,
            [
                encode_ping(sequence_id=1),
                encode_status_request(sequence_id=2),
                encode_sensor_request(sequence_id=3),
                encode_rearm(sequence_id=4),
                encode_motion_command(
                    sequence_id=5,
                    command_type=CommandType.MOVE_FORWARD,
                ),
                encode_emergency_stop(sequence_id=6),
            ],
        )
        self.assertIn(
            "Bounded Arduino motion diagnostic succeeded.",
            output.getvalue(),
        )
        self.assertIn(
            "Emergency-stop cleanup was confirmed.",
            output.getvalue(),
        )

    def test_blocked_completion_still_requests_estop(
        self,
    ) -> None:
        serial_port = _ScriptedSerialPort(
            b"ALR|1|PONG|1\n",
            b"ALR|1|STATUS|2|ESTOPPED|LATCHED\n",
            b"ALR|1|SENSORS|3|1|1|0|350|400\n",
            b"ALR|1|STATUS|4|IDLE|SAFE\n",
            b"ALR|1|ACK|5|MOVE_FORWARD\n",
            b"ALR|1|DONE|5|BLOCKED\n",
            b"ALR|1|STATUS|6|ESTOPPED|LATCHED\n",
        )
        transport = ArduinoSerialTransport(
            serial_port=serial_port
        )
        error_output = StringIO()

        with (
            patch(
                "tools.check_bounded_arduino_motion."
                "open_arduino_serial_transport",
                return_value=transport,
            ),
            redirect_stderr(error_output),
        ):
            exit_code = main(
                [
                    "--port",
                    "COM3",
                    "--confirm-wheels-lifted",
                ]
            )

        self.assertEqual(exit_code, 2)
        self.assertFalse(serial_port.is_open)
        self.assertEqual(
            serial_port.requests[-1],
            encode_emergency_stop(sequence_id=6),
        )
        self.assertIn(
            "BLOCKED",
            error_output.getvalue(),
        )


if __name__ == "__main__":
    unittest.main()