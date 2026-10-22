"""Unit tests for Arduino and visual perception fusion."""

import unittest
from dataclasses import replace
from datetime import UTC, datetime

from ai_logistics_robot.adapters.hardware.arduino_sensor_interpreter import (
    ArduinoSensorInterpreter,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoSensorSnapshot,
)
from ai_logistics_robot.adapters.hardware.sensor_fused_grid_perception import (
    SensorFusedGridPerception,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.ports.perception_port import PerceptionPort


def _raw_snapshot(
    *,
    sequence_id: int = 1,
    left_ir_raw: int = 1,
    right_ir_raw: int = 1,
    line_raw: int = 0,
    distance_mm: int | None = 500,
    light_raw: int = 300,
) -> ArduinoSensorSnapshot:
    """Return one valid raw Arduino snapshot."""

    return ArduinoSensorSnapshot(
        sequence_id=sequence_id,
        left_ir_raw=left_ir_raw,
        right_ir_raw=right_ir_raw,
        line_raw=line_raw,
        distance_mm=distance_mm,
        light_raw=light_raw,
    )


def _perception_snapshot(
    *,
    target_active: bool = False,
    hazard_detected: bool = False,
) -> PerceptionSnapshot:
    """Return one valid synthetic physical perception snapshot."""

    return PerceptionSnapshot(
        robot_id="robot-1",
        captured_at=datetime(
            2026,
            10,
            22,
            12,
            0,
            tzinfo=UTC,
        ),
        robot_pose=RobotPose(
            position=Position(x=3, y=3),
            heading=Heading.NORTH,
        ),
        observations=(),
        target_active=target_active,
        hazard_detected=hazard_detected,
    )


class _SnapshotReader:
    """Return synthetic sensor snapshots in deterministic order."""

    def __init__(
        self,
        *snapshots: ArduinoSensorSnapshot,
    ) -> None:
        self._snapshots = snapshots
        self.calls = 0

    def __call__(self) -> ArduinoSensorSnapshot:
        snapshot = self._snapshots[self.calls]
        self.calls += 1
        return snapshot


class _MutablePerception:
    """Minimal mutable physical perception test double."""

    def __init__(self) -> None:
        self.target_active = False
        self.hazard_detected = False
        self.observe_calls = 0

    def set_target_active(
        self,
        target_active: bool,
    ) -> None:
        self.target_active = target_active

    def set_hazard_detected(
        self,
        hazard_detected: bool,
    ) -> None:
        self.hazard_detected = hazard_detected

    def observe(self) -> PerceptionSnapshot:
        self.observe_calls += 1
        return replace(
            _perception_snapshot(),
            target_active=self.target_active,
            hazard_detected=self.hazard_detected,
        )


class _InvalidReturningPerception:
    """Satisfy the mutable surface but return invalid data."""

    def set_target_active(
        self,
        target_active: bool,
    ) -> None:
        del target_active

    def set_hazard_detected(
        self,
        hazard_detected: bool,
    ) -> None:
        del hazard_detected

    def observe(self) -> object:
        return object()


class SensorFusedGridPerceptionTests(unittest.TestCase):
    """Verify safe fusion without producing motor actions."""

    def test_constructor_exposes_valid_dependencies(self) -> None:
        perception = _MutablePerception()
        reader = _SnapshotReader(_raw_snapshot())
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
        )

        fused = SensorFusedGridPerception(
            perception=perception,
            sensor_reader=reader,
            interpreter=interpreter,
        )

        self.assertIs(fused.perception, perception)
        self.assertIs(fused.interpreter, interpreter)
        self.assertIsNone(fused.last_interpretation)
        self.assertIsInstance(fused, PerceptionPort)

    def test_target_and_boundary_are_fused_into_snapshot(
        self,
    ) -> None:
        perception = _MutablePerception()
        reader = _SnapshotReader(
            _raw_snapshot(
                line_raw=1,
                light_raw=900,
            )
        )
        fused = SensorFusedGridPerception(
            perception=perception,
            sensor_reader=reader,
            interpreter=ArduinoSensorInterpreter(
                ambient_light_raw=300,
                required_stable_readings=1,
            ),
        )

        snapshot = fused.observe()

        self.assertTrue(snapshot.target_active)
        self.assertTrue(snapshot.hazard_detected)
        self.assertEqual(reader.calls, 1)
        self.assertEqual(perception.observe_calls, 1)
        self.assertIsNotNone(
            fused.last_interpretation
        )

    def test_normal_obstacles_do_not_become_hazards(
        self,
    ) -> None:
        perception = _MutablePerception()
        fused = SensorFusedGridPerception(
            perception=perception,
            sensor_reader=_SnapshotReader(
                _raw_snapshot(
                    left_ir_raw=0,
                    right_ir_raw=0,
                    line_raw=0,
                    distance_mm=144,
                    light_raw=901,
                )
            ),
            interpreter=ArduinoSensorInterpreter(
                ambient_light_raw=300,
                frontal_obstacle_distance_mm=200,
                required_stable_readings=1,
            ),
        )

        snapshot = fused.observe()
        interpretation = fused.last_interpretation

        self.assertTrue(snapshot.target_active)
        self.assertFalse(snapshot.hazard_detected)
        self.assertIsNotNone(interpretation)
        assert interpretation is not None
        self.assertTrue(
            interpretation.left_obstacle_detected
        )
        self.assertTrue(
            interpretation.right_obstacle_detected
        )
        self.assertTrue(
            interpretation.frontal_obstacle_detected
        )

    def test_target_debounce_spans_multiple_observations(
        self,
    ) -> None:
        perception = _MutablePerception()
        reader = _SnapshotReader(
            _raw_snapshot(
                sequence_id=1,
                light_raw=900,
            ),
            _raw_snapshot(
                sequence_id=2,
                light_raw=900,
            ),
        )
        fused = SensorFusedGridPerception(
            perception=perception,
            sensor_reader=reader,
            interpreter=ArduinoSensorInterpreter(
                ambient_light_raw=300,
                required_stable_readings=2,
            ),
        )

        first = fused.observe()
        second = fused.observe()

        self.assertFalse(first.target_active)
        self.assertTrue(second.target_active)

    def test_rejects_invalid_dependencies(self) -> None:
        perception = _MutablePerception()
        reader = _SnapshotReader(_raw_snapshot())
        interpreter = ArduinoSensorInterpreter(
            ambient_light_raw=300,
        )

        cases = (
            (
                {
                    "perception": object(),
                    "sensor_reader": reader,
                    "interpreter": interpreter,
                },
                "perception",
            ),
            (
                {
                    "perception": perception,
                    "sensor_reader": object(),
                    "interpreter": interpreter,
                },
                "sensor_reader",
            ),
            (
                {
                    "perception": perception,
                    "sensor_reader": reader,
                    "interpreter": object(),
                },
                "interpreter",
            ),
        )

        for arguments, expected_message in cases:
            with self.subTest(arguments=arguments):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    expected_message,
                ):
                    SensorFusedGridPerception(**arguments)

    def test_rejects_invalid_raw_sensor_snapshot(self) -> None:
        fused = SensorFusedGridPerception(
            perception=_MutablePerception(),
            sensor_reader=lambda: object(),
            interpreter=ArduinoSensorInterpreter(
                ambient_light_raw=300,
            ),
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "ArduinoSensorSnapshot",
        ):
            fused.observe()

        self.assertIsNone(fused.last_interpretation)

    def test_rejects_invalid_delegated_snapshot(self) -> None:
        fused = SensorFusedGridPerception(
            perception=_InvalidReturningPerception(),
            sensor_reader=_SnapshotReader(
                _raw_snapshot()
            ),
            interpreter=ArduinoSensorInterpreter(
                ambient_light_raw=300,
            ),
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "PerceptionSnapshot",
        ):
            fused.observe()

        self.assertIsNone(fused.last_interpretation)


if __name__ == "__main__":
    unittest.main()