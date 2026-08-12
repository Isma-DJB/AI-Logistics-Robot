"""Unit tests for the immutable mission model."""

import unittest
from dataclasses import FrozenInstanceError

from ai_logistics_robot.domain.enums import (
    FailureReason,
    MissionStatus,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position
from ai_logistics_robot.domain.mission import Mission


class MissionTests(unittest.TestCase):
    """Verify mission validation and lifecycle invariants."""

    def valid_mission_data(self) -> dict[str, object]:
        """Return reusable valid mission construction data."""

        return {
            "mission_id": "mission_1",
            "robot_id": "robot_1",
            "target_id": "target_1",
            "target_position": Position(x=8, y=7),
            "base_position": Position(x=1, y=1),
        }

    def test_created_mission_accepts_valid_data(self) -> None:
        mission = Mission(**self.valid_mission_data())

        self.assertIs(mission.status, MissionStatus.CREATED)
        self.assertFalse(mission.collection_completed)
        self.assertFalse(mission.base_arrival_confirmed)
        self.assertIsNone(mission.terminal_reason)

    def test_mission_rejects_invalid_identifiers(self) -> None:
        for field in ("mission_id", "robot_id", "target_id"):
            with self.subTest(field=field):
                data = self.valid_mission_data()
                data[field] = " "

                with self.assertRaises(DomainValidationError):
                    Mission(**data)

    def test_mission_rejects_invalid_positions(self) -> None:
        for field in ("target_position", "base_position"):
            with self.subTest(field=field):
                data = self.valid_mission_data()
                data[field] = (1, 1)

                with self.assertRaises(DomainValidationError):
                    Mission(**data)

    def test_mission_rejects_invalid_status(self) -> None:
        data = self.valid_mission_data()
        data["status"] = "CREATED"

        with self.assertRaises(DomainValidationError):
            Mission(**data)

    def test_base_arrival_requires_completed_collection(self) -> None:
        data = self.valid_mission_data()

        with self.assertRaises(InvariantViolationError):
            Mission(
                **data,
                status=MissionStatus.ACTIVE,
                base_arrival_confirmed=True,
            )

    def test_success_requires_completed_collection(self) -> None:
        data = self.valid_mission_data()

        with self.assertRaises(InvariantViolationError):
            Mission(
                **data,
                status=MissionStatus.SUCCESS,
                base_arrival_confirmed=True,
            )

    def test_success_requires_confirmed_base_arrival(self) -> None:
        data = self.valid_mission_data()

        with self.assertRaises(InvariantViolationError):
            Mission(
                **data,
                status=MissionStatus.SUCCESS,
                collection_completed=True,
            )

    def test_success_accepts_both_required_confirmations(self) -> None:
        mission = Mission(
            **self.valid_mission_data(),
            status=MissionStatus.SUCCESS,
            collection_completed=True,
            base_arrival_confirmed=True,
        )

        self.assertIs(mission.status, MissionStatus.SUCCESS)

    def test_aborted_mission_requires_explicit_reason(self) -> None:
        with self.assertRaises(InvariantViolationError):
            Mission(
                **self.valid_mission_data(),
                status=MissionStatus.ABORTED,
            )

    def test_failed_mission_accepts_explicit_reason(self) -> None:
        mission = Mission(
            **self.valid_mission_data(),
            status=MissionStatus.FAILED,
            terminal_reason=FailureReason.NO_PATH,
        )

        self.assertIs(
            mission.terminal_reason,
            FailureReason.NO_PATH,
        )

    def test_active_mission_rejects_terminal_reason(self) -> None:
        with self.assertRaises(InvariantViolationError):
            Mission(
                **self.valid_mission_data(),
                status=MissionStatus.ACTIVE,
                terminal_reason=FailureReason.BLOCKED,
            )

    def test_mission_is_immutable(self) -> None:
        mission = Mission(**self.valid_mission_data())

        with self.assertRaises(FrozenInstanceError):
            mission.status = MissionStatus.ACTIVE  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()