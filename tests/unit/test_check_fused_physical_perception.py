"""Offline tests for the fused physical-perception diagnostic."""

import unittest
from contextlib import redirect_stderr
from io import StringIO

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoControllerStatus,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoDeviceState,
    ArduinoSafetyState,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from tools.check_fused_physical_perception import (
    FusedPhysicalPerceptionDiagnosticError,
    _require_latched_status,
    _require_positive_samples,
    build_parser,
    main,
)


class FusedPhysicalPerceptionDiagnosticTests(
    unittest.TestCase
):
    """Verify CLI defaults and passive-safety guards."""

    def test_parser_exposes_safe_defaults(self) -> None:
        arguments = build_parser().parse_args(
            [
                "--url",
                "http://camera.test",
                "--port",
                "COM3",
                "--ambient-light-raw",
                "300",
            ]
        )

        self.assertEqual(arguments.url, "http://camera.test")
        self.assertEqual(arguments.port, "COM3")
        self.assertEqual(arguments.ambient_light_raw, 300)
        self.assertEqual(arguments.samples, 5)
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

    def test_accepts_positive_sample_count(self) -> None:
        self.assertEqual(
            _require_positive_samples(3),
            3,
        )

    def test_rejects_invalid_sample_counts(self) -> None:
        invalid_values = (
            True,
            0,
            -1,
            1.5,
            "1",
        )

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "positive integer",
                ):
                    _require_positive_samples(value)

    def test_accepts_latched_emergency_stop_status(
        self,
    ) -> None:
        status = ArduinoControllerStatus(
            sequence_id=1,
            device_state=(
                ArduinoDeviceState.EMERGENCY_STOPPED
            ),
            safety_state=ArduinoSafetyState.LATCHED,
        )

        _require_latched_status(
            status,
            stage="before",
        )

    def test_rejects_unlatched_controller_status(
        self,
    ) -> None:
        status = ArduinoControllerStatus(
            sequence_id=1,
            device_state=ArduinoDeviceState.IDLE,
            safety_state=ArduinoSafetyState.SAFE,
        )

        with self.assertRaisesRegex(
            FusedPhysicalPerceptionDiagnosticError,
            "ESTOPPED/LATCHED",
        ):
            _require_latched_status(
                status,
                stage="before",
            )

    def test_invalid_samples_fail_before_hardware_access(
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
                    "--ambient-light-raw",
                    "300",
                    "--samples",
                    "0",
                ]
            )

        self.assertEqual(exit_code, 2)
        self.assertIn(
            "samples must be a positive integer",
            error_output.getvalue(),
        )


if __name__ == "__main__":
    unittest.main()