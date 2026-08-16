"""Unit tests for deterministic in-memory monitoring."""

import unittest
from datetime import UTC, datetime, timedelta

from ai_logistics_robot.adapters.monitoring import (
    InMemoryMonitoring,
)
from ai_logistics_robot.domain.enums import BrainState
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.ports import MonitoringPort


class InMemoryMonitoringTests(unittest.TestCase):
    """Verify immutable ordered mission-event publication."""

    def setUp(self) -> None:
        self.timestamp = datetime(
            2026,
            8,
            16,
            10,
            0,
            tzinfo=UTC,
        )
        self.monitoring = InMemoryMonitoring()

    def event(
        self,
        *,
        mission_id: str,
        sequence_number: int,
        event_id: str | None = None,
    ) -> MissionEvent:
        """Build one valid event for monitoring tests."""

        selected_event_id = (
            f"{mission_id}_event_{sequence_number}"
            if event_id is None
            else event_id
        )

        return MissionEvent(
            event_id=selected_event_id,
            sequence_number=sequence_number,
            mission_id=mission_id,
            robot_id="robot_1",
            occurred_at=(
                self.timestamp
                + timedelta(seconds=sequence_number)
            ),
            source="brain",
            name=f"event_{sequence_number}",
            brain_state=BrainState.OUTBOUND_NAVIGATION,
        )

    def test_monitoring_satisfies_public_port(self) -> None:
        self.assertIsInstance(
            self.monitoring,
            MonitoringPort,
        )

    def test_events_are_returned_by_mission_in_order(
        self,
    ) -> None:
        first = self.event(
            mission_id="mission_1",
            sequence_number=1,
        )
        other = self.event(
            mission_id="mission_2",
            sequence_number=1,
        )
        second = self.event(
            mission_id="mission_1",
            sequence_number=2,
        )

        self.monitoring.publish(first)
        self.monitoring.publish(other)
        self.monitoring.publish(second)

        mission_events = self.monitoring.events_for(
            "mission_1"
        )

        self.assertEqual(
            mission_events,
            (first, second),
        )
        self.assertIs(
            mission_events[0],
            first,
        )
        self.assertEqual(
            self.monitoring.events_for("mission_2"),
            (other,),
        )
        self.assertEqual(
            self.monitoring.events_for("mission_3"),
            (),
        )

    def test_duplicate_event_identity_is_rejected(
        self,
    ) -> None:
        first = self.event(
            mission_id="mission_1",
            sequence_number=1,
            event_id="shared_event",
        )
        duplicate = self.event(
            mission_id="mission_2",
            sequence_number=1,
            event_id="shared_event",
        )

        self.monitoring.publish(first)

        with self.assertRaises(
            InvariantViolationError
        ):
            self.monitoring.publish(duplicate)

        self.assertEqual(
            self.monitoring.events_for("mission_1"),
            (first,),
        )
        self.assertEqual(
            self.monitoring.events_for("mission_2"),
            (),
        )

    def test_sequence_must_increase_within_each_mission(
        self,
    ) -> None:
        accepted = self.event(
            mission_id="mission_1",
            sequence_number=2,
        )
        repeated = self.event(
            mission_id="mission_1",
            sequence_number=2,
            event_id="repeated_sequence",
        )
        backward = self.event(
            mission_id="mission_1",
            sequence_number=1,
            event_id="backward_sequence",
        )

        self.monitoring.publish(accepted)

        for event in (repeated, backward):
            with self.subTest(event=event):
                with self.assertRaises(
                    InvariantViolationError
                ):
                    self.monitoring.publish(event)

        self.assertEqual(
            self.monitoring.events_for("mission_1"),
            (accepted,),
        )

    def test_invalid_public_inputs_are_rejected(
        self,
    ) -> None:
        with self.assertRaises(DomainValidationError):
            self.monitoring.publish(  # type: ignore[arg-type]
                None
            )

        for mission_id in ("", " ", None, 1):
            with self.subTest(mission_id=mission_id):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.monitoring.events_for(
                        mission_id  # type: ignore[arg-type]
                    )


if __name__ == "__main__":
    unittest.main()
