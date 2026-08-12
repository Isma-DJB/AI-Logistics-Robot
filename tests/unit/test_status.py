"""Unit tests for the read-only system-status model."""

import unittest
from dataclasses import FrozenInstanceError
from datetime import UTC, datetime

from ai_logistics_robot.domain.enums import (
    BrainState,
    FailureReason,
    Heading,
    MissionStatus,
    PathPhase,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus


class SystemStatusTests(unittest.TestCase):
    """Verify read-only status consistency and immutability."""

    def setUp(self) -> None:
        self.timestamp = datetime(2026, 8, 12, tzinfo=UTC)
        self.pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.safety_status = SafetyStatus(
            robot_id="robot_1",
            updated_at=self.timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )

    def valid_status(self) -> SystemStatus:
        """Build a valid waiting status without an active mission."""

        return SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=BrainState.WAITING_FOR_MISSION,
            robot_pose=self.pose,
            safety_status=self.safety_status,
        )

    def valid_plan(self) -> PathPlan:
        """Build a valid active outbound plan."""

        positions = (
            Position(x=1, y=1),
            Position(x=1, y=2),
        )
        return PathPlan(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.OUTBOUND,
            version=1,
            positions=positions,
            goal=positions[-1],
        )

    def test_waiting_status_accepts_no_active_mission(self) -> None:
        status = self.valid_status()

        self.assertIs(
            status.brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertIsNone(status.mission_id)
        self.assertIsNone(status.active_plan)

    def test_status_accepts_consistent_active_mission_and_plan(self) -> None:
        plan = self.valid_plan()
        status = SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=BrainState.OUTBOUND_NAVIGATION,
            robot_pose=self.pose,
            safety_status=self.safety_status,
            mission_id="mission_1",
            mission_status=MissionStatus.ACTIVE,
            active_plan=plan,
        )

        self.assertEqual(status.active_plan, plan)
        self.assertIs(status.mission_status, MissionStatus.ACTIVE)

    def test_status_accepts_normalized_latest_error(self) -> None:
        status = SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=BrainState.MISSION_FAILED,
            robot_pose=self.pose,
            safety_status=self.safety_status,
            mission_id="mission_1",
            mission_status=MissionStatus.FAILED,
            latest_error=FailureReason.NO_PATH,
        )

        self.assertIs(status.latest_error, FailureReason.NO_PATH)

    def test_status_rejects_empty_robot_id(self) -> None:
        with self.assertRaises(DomainValidationError):
            SystemStatus(
                robot_id=" ",
                observed_at=self.timestamp,
                brain_state=BrainState.WAITING_FOR_MISSION,
                robot_pose=self.pose,
                safety_status=self.safety_status,
            )

    def test_status_requires_timezone_aware_timestamp(self) -> None:
        with self.assertRaises(DomainValidationError):
            SystemStatus(
                robot_id="robot_1",
                observed_at=datetime(2026, 8, 12),
                brain_state=BrainState.WAITING_FOR_MISSION,
                robot_pose=self.pose,
                safety_status=self.safety_status,
            )

    def test_status_rejects_invalid_brain_state(self) -> None:
        with self.assertRaises(DomainValidationError):
            SystemStatus(  # type: ignore[arg-type]
                robot_id="robot_1",
                observed_at=self.timestamp,
                brain_state="WAITING_FOR_MISSION",
                robot_pose=self.pose,
                safety_status=self.safety_status,
            )

    def test_safety_status_must_belong_to_same_robot(self) -> None:
        other_safety_status = SafetyStatus(
            robot_id="robot_2",
            updated_at=self.timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )

        with self.assertRaises(InvariantViolationError):
            SystemStatus(
                robot_id="robot_1",
                observed_at=self.timestamp,
                brain_state=BrainState.WAITING_FOR_MISSION,
                robot_pose=self.pose,
                safety_status=other_safety_status,
            )

    def test_mission_id_requires_mission_status(self) -> None:
        with self.assertRaises(InvariantViolationError):
            SystemStatus(
                robot_id="robot_1",
                observed_at=self.timestamp,
                brain_state=BrainState.OUTBOUND_PLANNING,
                robot_pose=self.pose,
                safety_status=self.safety_status,
                mission_id="mission_1",
            )

    def test_mission_status_requires_mission_id(self) -> None:
        with self.assertRaises(InvariantViolationError):
            SystemStatus(
                robot_id="robot_1",
                observed_at=self.timestamp,
                brain_state=BrainState.OUTBOUND_PLANNING,
                robot_pose=self.pose,
                safety_status=self.safety_status,
                mission_status=MissionStatus.ACTIVE,
            )

    def test_active_plan_requires_matching_mission(self) -> None:
        plan = self.valid_plan()

        with self.assertRaises(InvariantViolationError):
            SystemStatus(
                robot_id="robot_1",
                observed_at=self.timestamp,
                brain_state=BrainState.OUTBOUND_NAVIGATION,
                robot_pose=self.pose,
                safety_status=self.safety_status,
                mission_id="mission_2",
                mission_status=MissionStatus.ACTIVE,
                active_plan=plan,
            )

    def test_active_plan_requires_matching_robot(self) -> None:
        positions = (
            Position(x=1, y=1),
            Position(x=1, y=2),
        )
        plan = PathPlan(
            mission_id="mission_1",
            robot_id="robot_2",
            phase=PathPhase.OUTBOUND,
            version=1,
            positions=positions,
            goal=positions[-1],
        )

        with self.assertRaises(InvariantViolationError):
            SystemStatus(
                robot_id="robot_1",
                observed_at=self.timestamp,
                brain_state=BrainState.OUTBOUND_NAVIGATION,
                robot_pose=self.pose,
                safety_status=self.safety_status,
                mission_id="mission_1",
                mission_status=MissionStatus.ACTIVE,
                active_plan=plan,
            )

    def test_status_rejects_invalid_latest_error(self) -> None:
        with self.assertRaises(DomainValidationError):
            SystemStatus(  # type: ignore[arg-type]
                robot_id="robot_1",
                observed_at=self.timestamp,
                brain_state=BrainState.SYSTEM_ERROR,
                robot_pose=self.pose,
                safety_status=self.safety_status,
                latest_error="INTERNAL_ERROR",
            )

    def test_status_is_immutable(self) -> None:
        status = self.valid_status()

        with self.assertRaises(FrozenInstanceError):
            status.brain_state = BrainState.INITIALIZATION  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()