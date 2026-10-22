"""Unit tests for deterministic Arduino sensor interpretation."""

import unittest

from ai_logistics_robot.adapters.hardware.arduino_sensor_interpreter import (
    ArduinoSensorInterpreter,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoSensorSnapshot,
)
from ai_logistics_robot.domain.errors import DomainValidationError


def _snapshot(
    *,
    sequence_id: int = 1,
    left_ir_raw: int = 1,
    right_ir_raw: int = 1,
    line_raw: int = 0,
    distance_mm: int | None = 500,
    light_raw: int = 300,
) -> ArduinoSensorSnapshot:
    """Return one valid synthetic raw sensor snapshot."""

    return ArduinoSensorSnapshot(
        sequence_id=sequence_id,
        left_ir_raw=left_ir_raw,
        right_ir_raw=right_ir_raw,
        line_raw=line_raw,
        distance_mm=distance_mm,
        light_raw=light_raw,
    )


class ArduinoSensorInterpreterTests(unittest.TestCase):
    """Verify polarity, thresholds, debounce, and hysteresis."""

    def test_exposes_initial_calibrated_state(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
        )

        self.assertEqual(
            interpreter.ambient_light_raw,
            300,
        )
        self.assertFalse(interpreter.target_active)

    def test_interprets_physical_sensor_polarities(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
        )

        interpretation = interpreter.interpret(
            _snapshot(
                sequence_id=7,
                left_ir_raw=0,
                right_ir_raw=1,
                line_raw=1,
                distance_mm=196,
                light_raw=314,
            )
        )

        self.assertEqual(interpretation.sequence_id, 7)
        self.assertTrue(
            interpretation.left_obstacle_detected
        )
        self.assertFalse(
            interpretation.right_obstacle_detected
        )
        self.assertTrue(interpretation.boundary_detected)
        self.assertTrue(
            interpretation.frontal_obstacle_detected
        )
        self.assertEqual(interpretation.distance_mm, 196)
        self.assertEqual(interpretation.light_raw, 314)
        self.assertEqual(interpretation.light_delta, 14)
        self.assertFalse(interpretation.target_active)

    def test_missing_ultrasonic_echo_is_not_fabricated(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
        )

        interpretation = interpreter.interpret(
            _snapshot(distance_mm=None)
        )

        self.assertIsNone(interpretation.distance_mm)
        self.assertFalse(
            interpretation.frontal_obstacle_detected
        )

    def test_frontal_distance_threshold_is_inclusive(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
            frontal_obstacle_distance_mm=200,
        )

        at_threshold = interpreter.interpret(
            _snapshot(distance_mm=200)
        )
        beyond_threshold = interpreter.interpret(
            _snapshot(
                sequence_id=2,
                distance_mm=201,
            )
        )

        self.assertTrue(
            at_threshold.frontal_obstacle_detected
        )
        self.assertFalse(
            beyond_threshold.frontal_obstacle_detected
        )

    def test_target_activation_requires_stable_readings(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
            target_activation_delta=220,
            target_release_delta=150,
            required_stable_readings=3,
        )

        first = interpreter.interpret(
            _snapshot(sequence_id=1, light_raw=520)
        )
        second = interpreter.interpret(
            _snapshot(sequence_id=2, light_raw=700)
        )
        third = interpreter.interpret(
            _snapshot(sequence_id=3, light_raw=900)
        )

        self.assertFalse(first.target_active)
        self.assertFalse(second.target_active)
        self.assertTrue(third.target_active)
        self.assertTrue(interpreter.target_active)

    def test_unstable_activation_sequence_is_reset(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
            required_stable_readings=3,
        )

        light_values = (
            900,
            900,
            400,
            900,
            900,
        )

        interpretations = tuple(
            interpreter.interpret(
                _snapshot(
                    sequence_id=index,
                    light_raw=light_raw,
                )
            )
            for index, light_raw in enumerate(
                light_values,
                start=1,
            )
        )

        self.assertTrue(
            all(
                not interpretation.target_active
                for interpretation in interpretations
            )
        )

        confirmed = interpreter.interpret(
            _snapshot(
                sequence_id=6,
                light_raw=900,
            )
        )

        self.assertTrue(confirmed.target_active)

    def test_target_release_uses_hysteresis_and_debounce(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
            target_activation_delta=220,
            target_release_delta=150,
            required_stable_readings=2,
        )

        interpreter.interpret(
            _snapshot(sequence_id=1, light_raw=900)
        )
        activated = interpreter.interpret(
            _snapshot(sequence_id=2, light_raw=900)
        )
        hysteresis = interpreter.interpret(
            _snapshot(sequence_id=3, light_raw=480)
        )
        first_release = interpreter.interpret(
            _snapshot(sequence_id=4, light_raw=450)
        )
        released = interpreter.interpret(
            _snapshot(sequence_id=5, light_raw=440)
        )

        self.assertTrue(activated.target_active)
        self.assertTrue(hysteresis.target_active)
        self.assertTrue(first_release.target_active)
        self.assertFalse(released.target_active)

    def test_release_counter_resets_after_recovery(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
            required_stable_readings=2,
        )

        interpreter.interpret(
            _snapshot(sequence_id=1, light_raw=900)
        )
        interpreter.interpret(
            _snapshot(sequence_id=2, light_raw=900)
        )
        interpreter.interpret(
            _snapshot(sequence_id=3, light_raw=400)
        )
        recovered = interpreter.interpret(
            _snapshot(sequence_id=4, light_raw=900)
        )
        new_release = interpreter.interpret(
            _snapshot(sequence_id=5, light_raw=400)
        )

        self.assertTrue(recovered.target_active)
        self.assertTrue(new_release.target_active)

    def test_bright_light_does_not_disable_ultrasonic(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
            frontal_obstacle_distance_mm=200,
            required_stable_readings=1,
        )

        interpretation = interpreter.interpret(
            _snapshot(
                distance_mm=144,
                light_raw=901,
            )
        )

        self.assertTrue(interpretation.target_active)
        self.assertTrue(
            interpretation.frontal_obstacle_detected
        )

    def test_rejects_invalid_configuration(self) -> None:
        cases = (
            (
                {"ambient_light_raw": -1},
                "ambient_light_raw",
            ),
            (
                {"ambient_light_raw": 1024},
                "ambient_light_raw",
            ),
            (
                {
                    "ambient_light_raw": 300,
                    "frontal_obstacle_distance_mm": 0,
                },
                "frontal_obstacle_distance_mm",
            ),
            (
                {
                    "ambient_light_raw": 300,
                    "target_activation_delta": 0,
                },
                "target_activation_delta",
            ),
            (
                {
                    "ambient_light_raw": 300,
                    "required_stable_readings": 0,
                },
                "required_stable_readings",
            ),
            (
                {
                    "ambient_light_raw": 300,
                    "target_activation_delta": 220,
                    "target_release_delta": 220,
                },
                "must be lower",
            ),
        )

        for arguments, expected_message in cases:
            with self.subTest(arguments=arguments):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    expected_message,
                ):
                    ArduinoSensorInterpreter(**arguments)

    def test_rejects_invalid_snapshot_dependency(self) -> None:
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "ArduinoSensorSnapshot",
        ):
            interpreter.interpret(object())


if __name__ == "__main__":
    unittest.main()