"""Unit tests for deterministic Arduino light calibration."""

import unittest

from ai_logistics_robot.adapters.hardware.arduino_light_calibration import (
    ArduinoAmbientLightCalibration,
    calibrate_ambient_light,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoSensorSnapshot,
)
from ai_logistics_robot.domain.errors import DomainValidationError


def _snapshot(
    *,
    sequence_id: int,
    light_raw: int,
) -> ArduinoSensorSnapshot:
    """Build one otherwise neutral raw sensor snapshot."""

    return ArduinoSensorSnapshot(
        sequence_id=sequence_id,
        left_ir_raw=1,
        right_ir_raw=1,
        line_raw=0,
        distance_mm=None,
        light_raw=light_raw,
    )


def _snapshots(
    *light_values: int,
) -> tuple[ArduinoSensorSnapshot, ...]:
    """Build immutable snapshots from ordered light values."""

    return tuple(
        _snapshot(
            sequence_id=index,
            light_raw=light_raw,
        )
        for index, light_raw in enumerate(
            light_values,
            start=1,
        )
    )


class ArduinoLightCalibrationTests(unittest.TestCase):
    """Validate stable ambient-baseline calculation."""

    def test_calibrates_stable_physical_ambient_samples(
        self,
    ) -> None:
        calibration = calibrate_ambient_light(
            _snapshots(
                557,
                557,
                557,
                557,
                557,
                557,
                557,
                557,
                557,
                557,
            )
        )

        self.assertEqual(
            calibration.ambient_light_raw,
            557,
        )
        self.assertEqual(calibration.sample_count, 10)
        self.assertEqual(
            calibration.minimum_light_raw,
            557,
        )
        self.assertEqual(
            calibration.maximum_light_raw,
            557,
        )
        self.assertEqual(calibration.light_range, 0)

    def test_rounds_the_arithmetic_mean_to_nearest_integer(
        self,
    ) -> None:
        calibration = calibrate_ambient_light(
            _snapshots(557, 557, 558, 558, 558)
        )

        self.assertEqual(
            calibration.ambient_light_raw,
            558,
        )

    def test_accepts_the_configured_maximum_range(
        self,
    ) -> None:
        calibration = calibrate_ambient_light(
            _snapshots(550, 552, 555, 558, 560),
            maximum_light_range=10,
        )

        self.assertEqual(calibration.light_range, 10)

    def test_rejects_unstable_ambient_samples(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "too unstable",
        ):
            calibrate_ambient_light(
                _snapshots(550, 552, 555, 558, 561),
                maximum_light_range=10,
            )

    def test_rejects_too_few_samples(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "at least 5",
        ):
            calibrate_ambient_light(
                _snapshots(557, 557, 557, 557)
            )

    def test_requires_an_immutable_tuple(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "immutable tuple",
        ):
            calibrate_ambient_light(
                [
                    _snapshot(
                        sequence_id=index,
                        light_raw=557,
                    )
                    for index in range(1, 6)
                ]
            )

    def test_rejects_an_invalid_snapshot_member(
        self,
    ) -> None:
        snapshots = (
            _snapshot(sequence_id=1, light_raw=557),
            _snapshot(sequence_id=2, light_raw=557),
            _snapshot(sequence_id=3, light_raw=557),
            _snapshot(sequence_id=4, light_raw=557),
            object(),
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "every snapshot",
        ):
            calibrate_ambient_light(snapshots)

    def test_rejects_invalid_maximum_ranges(self) -> None:
        snapshots = _snapshots(
            557,
            557,
            557,
            557,
            557,
        )
        invalid_values = (-1, 1024, True, 1.5)

        for invalid_value in invalid_values:
            with self.subTest(value=invalid_value):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "maximum_light_range",
                ):
                    calibrate_ambient_light(
                        snapshots,
                        maximum_light_range=invalid_value,
                    )

    def test_calibration_result_rejects_invalid_ranges(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "must not exceed",
        ):
            ArduinoAmbientLightCalibration(
                ambient_light_raw=555,
                sample_count=5,
                minimum_light_raw=560,
                maximum_light_raw=550,
            )

        with self.assertRaisesRegex(
            DomainValidationError,
            "within the observed",
        ):
            ArduinoAmbientLightCalibration(
                ambient_light_raw=549,
                sample_count=5,
                minimum_light_raw=550,
                maximum_light_raw=560,
            )

    def test_calibration_result_rejects_empty_sample_count(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "sample_count",
        ):
            ArduinoAmbientLightCalibration(
                ambient_light_raw=557,
                sample_count=0,
                minimum_light_raw=557,
                maximum_light_raw=557,
            )


if __name__ == "__main__":
    unittest.main()