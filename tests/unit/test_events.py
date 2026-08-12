"""Unit tests for ordered immutable mission events."""

import unittest
from dataclasses import FrozenInstanceError
from datetime import UTC, datetime

from ai_logistics_robot.domain.enums import BrainState
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.events import (
    EventDetail,
    MissionEvent,
)


class EventDetailTests(unittest.TestCase):
    """Verify typed event-detail validation."""

    def test_event_detail_accepts_scalar_values(self) -> None:
        values = ("value", 1, 0.5, True, None)

        for value in values:
            with self.subTest(value=value):
                detail = EventDetail(name="result", value=value)
                self.assertEqual(detail.value, value)

    def test_event_detail_rejects_empty_name(self) -> None:
        with self.assertRaises(DomainValidationError):
            EventDetail(name=" ", value="value")

    def test_event_detail_rejects_mutable_or_complex_value(self) -> None:
        invalid_values = ([], {}, object())

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    EventDetail(  # type: ignore[arg-type]
                        name="payload",
                        value=value,
                    )

    def test_event_detail_rejects_non_finite_float(self) -> None:
        for value in (float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    EventDetail(name="measurement", value=value)

    def test_event_detail_is_immutable(self) -> None:
        detail = EventDetail(name="result", value="confirmed")

        with self.assertRaises(FrozenInstanceError):
            detail.value = "changed"  # type: ignore[misc]


class MissionEventTests(unittest.TestCase):
    """Verify mission-event ordering and immutable details."""

    def setUp(self) -> None:
        self.timestamp = datetime(2026, 8, 12, tzinfo=UTC)

    def valid_event(self) -> MissionEvent:
        """Build one valid mission event."""

        return MissionEvent(
            event_id="event_1",
            sequence_number=1,
            mission_id="mission_1",
            robot_id="robot_1",
            occurred_at=self.timestamp,
            source="brain",
            name="state_transition",
            brain_state=BrainState.OUTBOUND_PLANNING,
            details=(
                EventDetail(
                    name="previous_state",
                    value="WAITING_FOR_MISSION",
                ),
            ),
        )

    def test_mission_event_accepts_valid_data(self) -> None:
        event = self.valid_event()

        self.assertEqual(event.sequence_number, 1)
        self.assertIs(
            event.brain_state,
            BrainState.OUTBOUND_PLANNING,
        )

    def test_event_detail_value_returns_requested_value(self) -> None:
        event = self.valid_event()

        self.assertEqual(
            event.detail_value("previous_state"),
            "WAITING_FOR_MISSION",
        )

    def test_missing_detail_raises_key_error(self) -> None:
        event = self.valid_event()

        with self.assertRaises(KeyError):
            event.detail_value("missing")

    def test_sequence_number_must_be_positive_integer(self) -> None:
        for value in (0, -1, 1.5, True):
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    MissionEvent(  # type: ignore[arg-type]
                        event_id="event_1",
                        sequence_number=value,
                        mission_id="mission_1",
                        robot_id="robot_1",
                        occurred_at=self.timestamp,
                        source="brain",
                        name="state_transition",
                        brain_state=BrainState.OUTBOUND_PLANNING,
                    )

    def test_event_requires_timezone_aware_timestamp(self) -> None:
        with self.assertRaises(DomainValidationError):
            MissionEvent(
                event_id="event_1",
                sequence_number=1,
                mission_id="mission_1",
                robot_id="robot_1",
                occurred_at=datetime(2026, 8, 12),
                source="brain",
                name="state_transition",
                brain_state=BrainState.OUTBOUND_PLANNING,
            )

    def test_event_rejects_mutable_details_collection(self) -> None:
        with self.assertRaises(DomainValidationError):
            MissionEvent(  # type: ignore[arg-type]
                event_id="event_1",
                sequence_number=1,
                mission_id="mission_1",
                robot_id="robot_1",
                occurred_at=self.timestamp,
                source="brain",
                name="state_transition",
                brain_state=BrainState.OUTBOUND_PLANNING,
                details=[],
            )

    def test_event_rejects_duplicate_detail_names(self) -> None:
        with self.assertRaises(InvariantViolationError):
            MissionEvent(
                event_id="event_1",
                sequence_number=1,
                mission_id="mission_1",
                robot_id="robot_1",
                occurred_at=self.timestamp,
                source="brain",
                name="state_transition",
                brain_state=BrainState.OUTBOUND_PLANNING,
                details=(
                    EventDetail(name="result", value="first"),
                    EventDetail(name="result", value="second"),
                ),
            )

    def test_event_is_immutable(self) -> None:
        event = self.valid_event()

        with self.assertRaises(FrozenInstanceError):
            event.sequence_number = 2  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()