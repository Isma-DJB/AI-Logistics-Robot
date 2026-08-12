"""Unit tests for Memory and Monitoring ports."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.domain.enums import (
    BrainState,
    Heading,
    MissionStatus,
    PathPhase,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.domain.paths import PathRecord
from ai_logistics_robot.ports.memory_port import MemoryPort
from ai_logistics_robot.ports.monitoring_port import MonitoringPort


class CompatibleMemory:
    """Minimal structural implementation of MemoryPort."""

    def __init__(self, return_path: PathRecord) -> None:
        self._return_path = return_path
        self.active_mission: Mission | None = None
        self.completed_mission: Mission | None = None
        self.pose_records: list[tuple[PathPhase, RobotPose]] = []
        self.events: list[MissionEvent] = []
        self.reset_called = False

    def start(self, mission: Mission) -> None:
        """Record the active mission."""

        self.active_mission = mission

    def record_pose(
        self,
        phase: PathPhase,
        pose: RobotPose,
    ) -> None:
        """Record one confirmed phase and pose pair."""

        self.pose_records.append((phase, pose))

    def record_event(self, event: MissionEvent) -> None:
        """Record one mission event."""

        self.events.append(event)

    def build_return_path(self) -> PathRecord:
        """Return the configured confirmed path record."""

        return self._return_path

    def complete(self, mission: Mission) -> None:
        """Record the completed mission."""

        self.completed_mission = mission

    def reset(self) -> None:
        """Clear the active recording state."""

        self.active_mission = None
        self.completed_mission = None
        self.pose_records.clear()
        self.events.clear()
        self.reset_called = True


class IncompleteMemory:
    """Object intentionally missing MemoryPort operations."""


class CompatibleMonitoring:
    """Minimal structural implementation of MonitoringPort."""

    def __init__(self) -> None:
        self.events: list[MissionEvent] = []

    def publish(self, event: MissionEvent) -> None:
        """Store one published event."""

        self.events.append(event)

    def events_for(
        self,
        mission_id: str,
    ) -> tuple[MissionEvent, ...]:
        """Return matching events in deterministic sequence order."""

        return tuple(
            sorted(
                (
                    event
                    for event in self.events
                    if event.mission_id == mission_id
                ),
                key=lambda event: event.sequence_number,
            )
        )


class IncompleteMonitoring:
    """Object intentionally missing MonitoringPort operations."""


class RecordingPortTests(unittest.TestCase):
    """Verify recording-port compatibility and typed results."""

    def setUp(self) -> None:
        self.pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.mission = Mission(
            mission_id="mission_1",
            robot_id="robot_1",
            target_id="target_1",
            target_position=Position(x=8, y=7),
            base_position=Position(x=1, y=1),
        )
        self.completed_mission = Mission(
            mission_id="mission_1",
            robot_id="robot_1",
            target_id="target_1",
            target_position=Position(x=8, y=7),
            base_position=Position(x=1, y=1),
            status=MissionStatus.SUCCESS,
            collection_completed=True,
            base_arrival_confirmed=True,
        )
        self.return_path = PathRecord(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.RETURN,
            confirmed_poses=(self.pose,),
        )
        brain_state = next(iter(BrainState))
        occurred_at = datetime(2026, 8, 12, tzinfo=UTC)

        self.event_one = MissionEvent(
            event_id="event_1",
            sequence_number=1,
            mission_id="mission_1",
            robot_id="robot_1",
            occurred_at=occurred_at,
            source="brain",
            name="mission_started",
            brain_state=brain_state,
        )
        self.event_two = MissionEvent(
            event_id="event_2",
            sequence_number=2,
            mission_id="mission_1",
            robot_id="robot_1",
            occurred_at=occurred_at,
            source="memory",
            name="pose_recorded",
            brain_state=brain_state,
        )
        self.other_event = MissionEvent(
            event_id="event_3",
            sequence_number=1,
            mission_id="mission_2",
            robot_id="robot_2",
            occurred_at=occurred_at,
            source="brain",
            name="mission_started",
            brain_state=brain_state,
        )

    def test_memory_records_mission_pose_and_event(self) -> None:
        memory = CompatibleMemory(self.return_path)

        self.assertIsInstance(memory, MemoryPort)

        memory.start(self.mission)
        memory.record_pose(PathPhase.OUTBOUND, self.pose)
        memory.record_event(self.event_one)

        self.assertIs(memory.active_mission, self.mission)
        self.assertEqual(
            memory.pose_records,
            [(PathPhase.OUTBOUND, self.pose)],
        )
        self.assertEqual(memory.events, [self.event_one])

    def test_memory_builds_path_completes_and_resets(self) -> None:
        memory = CompatibleMemory(self.return_path)
        memory.start(self.mission)

        self.assertIs(memory.build_return_path(), self.return_path)

        memory.complete(self.completed_mission)

        self.assertIs(
            memory.completed_mission,
            self.completed_mission,
        )

        memory.reset()

        self.assertTrue(memory.reset_called)
        self.assertIsNone(memory.active_mission)
        self.assertIsNone(memory.completed_mission)
        self.assertEqual(memory.pose_records, [])
        self.assertEqual(memory.events, [])

    def test_incomplete_memory_is_rejected(self) -> None:
        self.assertNotIsInstance(IncompleteMemory(), MemoryPort)

    def test_monitoring_returns_ordered_immutable_events(self) -> None:
        monitoring = CompatibleMonitoring()

        self.assertIsInstance(monitoring, MonitoringPort)

        monitoring.publish(self.event_two)
        monitoring.publish(self.event_one)

        events = monitoring.events_for("mission_1")

        self.assertIsInstance(events, tuple)
        self.assertEqual(events, (self.event_one, self.event_two))

    def test_monitoring_isolates_mission_identities(self) -> None:
        monitoring = CompatibleMonitoring()

        monitoring.publish(self.event_one)
        monitoring.publish(self.other_event)

        self.assertEqual(
            monitoring.events_for("mission_1"),
            (self.event_one,),
        )
        self.assertEqual(
            monitoring.events_for("mission_2"),
            (self.other_event,),
        )

    def test_incomplete_monitoring_is_rejected(self) -> None:
        self.assertNotIsInstance(
            IncompleteMonitoring(),
            MonitoringPort,
        )


if __name__ == "__main__":
    unittest.main()