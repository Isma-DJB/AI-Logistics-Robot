"""Unit tests for safety events and latched safety status."""

import unittest
from dataclasses import FrozenInstanceError
from datetime import UTC, datetime

from ai_logistics_robot.domain.enums import (
    FailureReason,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.safety import (
    SafetyEvent,
    SafetyStatus,
)


class SafetyEventTests(unittest.TestCase):
    """Verify safety-event validation."""

    def setUp(self) -> None:
        self.timestamp = datetime(2026, 8, 12, tzinfo=UTC)

    def test_critical_event_requires_priority_stop(self) -> None:
        event = SafetyEvent(
            event_id="safety_1",
            robot_id="robot_1",
            occurred_at=self.timestamp,
            severity=SafetySeverity.CRITICAL,
            source="local_control",
            message="Emergency stop requested.",
            reason=FailureReason.EMERGENCY_STOP,
        )

        self.assertTrue(event.requires_stop)

    def test_non_critical_event_does_not_require_stop(self) -> None:
        event = SafetyEvent(
            event_id="safety_1",
            robot_id="robot_1",
            occurred_at=self.timestamp,
            severity=SafetySeverity.WARNING,
            source="perception",
            message="Potential obstacle detected.",
        )

        self.assertFalse(event.requires_stop)

    def test_critical_event_requires_explicit_reason(self) -> None:
        with self.assertRaises(InvariantViolationError):
            SafetyEvent(
                event_id="safety_1",
                robot_id="robot_1",
                occurred_at=self.timestamp,
                severity=SafetySeverity.CRITICAL,
                source="control",
                message="Critical safety condition.",
            )

    def test_event_rejects_naive_timestamp(self) -> None:
        with self.assertRaises(DomainValidationError):
            SafetyEvent(
                event_id="safety_1",
                robot_id="robot_1",
                occurred_at=datetime(2026, 8, 12),
                severity=SafetySeverity.INFO,
                source="control",
                message="Safety system initialized.",
            )

    def test_event_rejects_invalid_severity(self) -> None:
        with self.assertRaises(DomainValidationError):
            SafetyEvent(  # type: ignore[arg-type]
                event_id="safety_1",
                robot_id="robot_1",
                occurred_at=self.timestamp,
                severity="CRITICAL",
                source="control",
                message="Invalid severity.",
            )

    def test_event_is_immutable(self) -> None:
        event = SafetyEvent(
            event_id="safety_1",
            robot_id="robot_1",
            occurred_at=self.timestamp,
            severity=SafetySeverity.INFO,
            source="control",
            message="Safety system initialized.",
        )

        with self.assertRaises(FrozenInstanceError):
            event.severity = SafetySeverity.WARNING  # type: ignore[misc]


class SafetyStatusTests(unittest.TestCase):
    """Verify safety-latch and manual-rearm invariants."""

    def setUp(self) -> None:
        self.timestamp = datetime(2026, 8, 12, tzinfo=UTC)

    def test_unlatched_status_is_safe(self) -> None:
        status = SafetyStatus(
            robot_id="robot_1",
            updated_at=self.timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )

        self.assertFalse(status.manual_rearm_required)
        self.assertIsNone(status.reason)

    def test_latched_status_requires_manual_rearm(self) -> None:
        status = SafetyStatus(
            robot_id="robot_1",
            updated_at=self.timestamp,
            latched=True,
            severity=SafetySeverity.CRITICAL,
            reason=FailureReason.EMERGENCY_STOP,
        )

        self.assertTrue(status.manual_rearm_required)

    def test_latched_status_requires_critical_severity(self) -> None:
        with self.assertRaises(InvariantViolationError):
            SafetyStatus(
                robot_id="robot_1",
                updated_at=self.timestamp,
                latched=True,
                severity=SafetySeverity.WARNING,
                reason=FailureReason.BLOCKED,
            )

    def test_latched_status_requires_reason(self) -> None:
        with self.assertRaises(InvariantViolationError):
            SafetyStatus(
                robot_id="robot_1",
                updated_at=self.timestamp,
                latched=True,
                severity=SafetySeverity.CRITICAL,
            )

    def test_unlatched_status_rejects_non_info_severity(self) -> None:
        with self.assertRaises(InvariantViolationError):
            SafetyStatus(
                robot_id="robot_1",
                updated_at=self.timestamp,
                latched=False,
                severity=SafetySeverity.WARNING,
            )

    def test_unlatched_status_cannot_retain_reason(self) -> None:
        with self.assertRaises(InvariantViolationError):
            SafetyStatus(
                robot_id="robot_1",
                updated_at=self.timestamp,
                latched=False,
                severity=SafetySeverity.INFO,
                reason=FailureReason.EMERGENCY_STOP,
            )

    def test_latched_flag_must_be_boolean(self) -> None:
        with self.assertRaises(DomainValidationError):
            SafetyStatus(  # type: ignore[arg-type]
                robot_id="robot_1",
                updated_at=self.timestamp,
                latched=1,
                severity=SafetySeverity.INFO,
            )

    def test_status_is_immutable(self) -> None:
        status = SafetyStatus(
            robot_id="robot_1",
            updated_at=self.timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )

        with self.assertRaises(FrozenInstanceError):
            status.latched = True  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()