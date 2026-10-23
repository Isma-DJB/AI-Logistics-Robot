"""Offline tests for automatic fused ambient calibration."""

import unittest
from contextlib import redirect_stderr
from io import StringIO

from ai_logistics_robot.domain.errors import DomainValidationError
from tools.check_fused_physical_perception import (
    _require_ambient_calibration_samples,
    build_parser,
    main,
)


class AutomaticAmbientCalibrationDiagnosticTests(
    unittest.TestCase
):
    """Verify automatic calibration defaults and guards."""

    def test_parser_enables_automatic_calibration_by_default(
        self,
    ) -> None:
        arguments = build_parser().parse_args(
            [
                "--url",
                "http://camera.test",
                "--port",
                "COM3",
            ]
        )

        self.assertIsNone(arguments.ambient_light_raw)
        self.assertEqual(
            arguments.ambient_calibration_samples,
            10,
        )
        self.assertEqual(
            arguments.maximum_ambient_light_range,
            10,
        )

    def test_accepts_safe_calibration_sample_counts(
        self,
    ) -> None:
        self.assertEqual(
            _require_ambient_calibration_samples(5),
            5,
        )
        self.assertEqual(
            _require_ambient_calibration_samples(10),
            10,
        )

    def test_rejects_invalid_calibration_sample_counts(
        self,
    ) -> None:
        invalid_values = (
            True,
            0,
            4,
            -1,
            1.5,
            "5",
        )

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "integer of at least 5",
                ):
                    _require_ambient_calibration_samples(
                        value
                    )

    def test_invalid_count_fails_before_hardware_access(
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
                    "--ambient-calibration-samples",
                    "4",
                ]
            )

        self.assertEqual(exit_code, 2)
        self.assertIn(
            "ambient_calibration_samples must be "
            "an integer of at least 5",
            error_output.getvalue(),
        )


if __name__ == "__main__":
    unittest.main()