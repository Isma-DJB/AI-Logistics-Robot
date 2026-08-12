"""Unit tests for normalized perception objects."""

import unittest
from dataclasses import FrozenInstanceError
from datetime import UTC, datetime

from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.perception import (
    Observation,
    PerceptionSnapshot,
)


class ObservationTests(unittest.TestCase):
    """Verify observation validation and immutability."""

    def test_observation_accepts_valid_data(self) -> None:
        observation = Observation(
            kind="obstacle",
            position=Position(x=1, y=4),
            confidence=0.9,
        )

        self.assertEqual(observation.kind, "obstacle")
        self.assertEqual(observation.confidence, 0.9)

    def test_integer_confidence_is_normalized_to_float(self) -> None:
        observation = Observation(
            kind="target",
            position=Position(x=8, y=7),
            confidence=1,
        )

        self.assertEqual(observation.confidence, 1.0)
        self.assertIsInstance(observation.confidence, float)

    def test_observation_rejects_empty_kind(self) -> None:
        with self.assertRaises(DomainValidationError):
            Observation(
                kind=" ",
                position=Position(x=1, y=1),
                confidence=0.5,
            )

    def test_observation_rejects_invalid_position(self) -> None:
        with self.assertRaises(DomainValidationError):
            Observation(  # type: ignore[arg-type]
                kind="obstacle",
                position=(1, 1),
                confidence=0.5,
            )

    def test_confidence_must_be_finite_and_in_range(self) -> None:
        invalid_values = (
            -0.1,
            1.1,
            float("nan"),
            float("inf"),
            True,
            "0.5",
        )

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    Observation(  # type: ignore[arg-type]
                        kind="obstacle",
                        position=Position(x=1, y=1),
                        confidence=value,
                    )

    def test_observation_is_immutable(self) -> None:
        observation = Observation(
            kind="obstacle",
            position=Position(x=1, y=1),
            confidence=0.8,
        )

        with self.assertRaises(FrozenInstanceError):
            observation.kind = "target"  # type: ignore[misc]


class PerceptionSnapshotTests(unittest.TestCase):
    """Verify normalized snapshot validation."""

    def setUp(self) -> None:
        self.pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.timestamp = datetime(2026, 8, 12, tzinfo=UTC)

    def test_snapshot_accepts_consistent_data(self) -> None:
        observation = Observation(
            kind="target",
            position=Position(x=8, y=7),
            confidence=0.95,
        )
        snapshot = PerceptionSnapshot(
            robot_id="robot_1",
            captured_at=self.timestamp,
            robot_pose=self.pose,
            observations=(observation,),
            target_active=True,
            hazard_detected=False,
        )

        self.assertEqual(snapshot.observations, (observation,))
        self.assertTrue(snapshot.target_active)

    def test_snapshot_requires_timezone_aware_timestamp(self) -> None:
        with self.assertRaises(DomainValidationError):
            PerceptionSnapshot(
                robot_id="robot_1",
                captured_at=datetime(2026, 8, 12),
                robot_pose=self.pose,
                observations=(),
                target_active=False,
                hazard_detected=False,
            )

    def test_snapshot_requires_immutable_observations(self) -> None:
        with self.assertRaises(DomainValidationError):
            PerceptionSnapshot(  # type: ignore[arg-type]
                robot_id="robot_1",
                captured_at=self.timestamp,
                robot_pose=self.pose,
                observations=[],
                target_active=False,
                hazard_detected=False,
            )

    def test_snapshot_rejects_non_observation_entries(self) -> None:
        with self.assertRaises(DomainValidationError):
            PerceptionSnapshot(  # type: ignore[arg-type]
                robot_id="robot_1",
                captured_at=self.timestamp,
                robot_pose=self.pose,
                observations=(Position(x=1, y=1),),
                target_active=False,
                hazard_detected=False,
            )

    def test_snapshot_requires_boolean_flags(self) -> None:
        for field in ("target_active", "hazard_detected"):
            with self.subTest(field=field):
                data: dict[str, object] = {
                    "robot_id": "robot_1",
                    "captured_at": self.timestamp,
                    "robot_pose": self.pose,
                    "observations": (),
                    "target_active": False,
                    "hazard_detected": False,
                }
                data[field] = 1

                with self.assertRaises(DomainValidationError):
                    PerceptionSnapshot(**data)

    def test_snapshot_is_immutable(self) -> None:
        snapshot = PerceptionSnapshot(
            robot_id="robot_1",
            captured_at=self.timestamp,
            robot_pose=self.pose,
            observations=(),
            target_active=False,
            hazard_detected=False,
        )

        with self.assertRaises(FrozenInstanceError):
            snapshot.target_active = True  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()