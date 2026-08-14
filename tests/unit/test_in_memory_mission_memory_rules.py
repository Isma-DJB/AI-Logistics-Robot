"""Unit tests for in-memory mission lifecycle rules."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.domain.enums import (
    BrainState,
    FailureReason,
    Heading,
    MissionStatus,
    PathPhase,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvalidStateTransitionError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.memory.in_memory_mission_memory import (
    InMemoryMissionMemory,
)


class InMemoryMissionMemoryRuleTests(unittest.TestCase):
    """Verify events, completion, rejection, and reset rules."""

    def setUp(self) -> None:
        self.mission = Mission(
            mission_id="mission_1",
            robot_id="robot_1",
            target_id="target_1",
            target_position=Position(x=4, y=4),
            base_position=Position(x=0, y=0),
        )
        self.pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.occurred_at = datetime(
            2026,
            8,
            14,
            tzinfo=UTC,
        )

    def make_event(
        self,
        *,
        event_id: str = "event_1",
        sequence_number: int = 1,
        mission_id: str | None = None,
        robot_id: str | None = None,
    ) -> MissionEvent:
        """Build one valid event with optional identity changes."""

        return MissionEvent(
            event_id=event_id,
            sequence_number=sequence_number,
            mission_id=(
                self.mission.mission_id
                if mission_id is None
                else mission_id
            ),
            robot_id=(
                self.mission.robot_id
                if robot_id is None
                else robot_id
            ),
            occurred_at=self.occurred_at,
            source="brain",
            name="mission_progressed",
            brain_state=BrainState.OUTBOUND_NAVIGATION,
        )

    def successful_mission(
        self,
        **overrides: object,
    ) -> Mission:
        """Build a successful outcome matching the started mission."""

        data: dict[str, object] = {
            "mission_id": self.mission.mission_id,
            "robot_id": self.mission.robot_id,
            "target_id": self.mission.target_id,
            "target_position": self.mission.target_position,
            "base_position": self.mission.base_position,
            "status": MissionStatus.SUCCESS,
            "collection_completed": True,
            "base_arrival_confirmed": True,
        }
        data.update(overrides)

        return Mission(
            **data,  # type: ignore[arg-type]
        )

    def failure_mission(
        self,
        status: MissionStatus,
    ) -> Mission:
        """Build one valid failed or aborted outcome."""

        return Mission(
            mission_id=self.mission.mission_id,
            robot_id=self.mission.robot_id,
            target_id=self.mission.target_id,
            target_position=self.mission.target_position,
            base_position=self.mission.base_position,
            status=status,
            terminal_reason=FailureReason.INTERNAL_ERROR,
        )

    def test_created_and_active_missions_can_start(self) -> None:
        active_mission = Mission(
            mission_id="mission_active",
            robot_id="robot_1",
            target_id="target_1",
            target_position=Position(x=4, y=4),
            base_position=Position(x=0, y=0),
            status=MissionStatus.ACTIVE,
        )

        for mission in (self.mission, active_mission):
            with self.subTest(status=mission.status):
                memory = InMemoryMissionMemory()

                memory.start(mission)

                self.assertIs(memory.active_mission, mission)

    def test_start_rejects_invalid_and_terminal_missions(
        self,
    ) -> None:
        memory = InMemoryMissionMemory()

        with self.assertRaises(DomainValidationError):
            memory.start("mission")  # type: ignore[arg-type]

        with self.assertRaises(InvalidStateTransitionError):
            memory.start(
                self.failure_mission(MissionStatus.FAILED)
            )

        self.assertIsNone(memory.active_mission)

    def test_second_start_requires_explicit_reset(self) -> None:
        memory = InMemoryMissionMemory()
        other_mission = Mission(
            mission_id="mission_2",
            robot_id="robot_2",
            target_id="target_2",
            target_position=Position(x=3, y=3),
            base_position=Position(x=1, y=1),
        )
        memory.start(self.mission)

        with self.assertRaises(InvalidStateTransitionError):
            memory.start(other_mission)

        self.assertIs(memory.active_mission, self.mission)

    def test_operations_require_started_mission(self) -> None:
        memory = InMemoryMissionMemory()
        event = self.make_event()
        completed = self.successful_mission()

        with self.assertRaises(InvalidStateTransitionError):
            memory.record_pose(PathPhase.OUTBOUND, self.pose)

        with self.assertRaises(InvalidStateTransitionError):
            memory.record_event(event)

        with self.assertRaises(InvalidStateTransitionError):
            memory.build_return_path()

        with self.assertRaises(InvalidStateTransitionError):
            memory.complete(completed)

    def test_pose_validation_is_atomic(self) -> None:
        memory = InMemoryMissionMemory()
        memory.start(self.mission)

        invalid_cases: tuple[tuple[object, object], ...] = (
            ("OUTBOUND", self.pose),
            (PathPhase.DETOUR, self.pose),
            (PathPhase.OUTBOUND, Position(x=0, y=0)),
        )

        for phase, pose in invalid_cases:
            with self.subTest(phase=phase, pose=pose):
                with self.assertRaises(
                    DomainValidationError
                ):
                    memory.record_pose(
                        phase,  # type: ignore[arg-type]
                        pose,  # type: ignore[arg-type]
                    )

        self.assertEqual(memory.outbound_poses, ())
        self.assertEqual(memory.return_poses, ())

    def test_events_preserve_increasing_sequence_order(
        self,
    ) -> None:
        memory = InMemoryMissionMemory()
        event_1 = self.make_event()
        event_3 = self.make_event(
            event_id="event_3",
            sequence_number=3,
        )
        memory.start(self.mission)

        memory.record_event(event_1)
        memory.record_event(event_3)

        self.assertEqual(
            memory.events,
            (event_1, event_3),
        )

    def test_event_identity_must_match_active_mission(
        self,
    ) -> None:
        memory = InMemoryMissionMemory()
        memory.start(self.mission)

        invalid_events = (
            self.make_event(mission_id="mission_2"),
            self.make_event(robot_id="robot_2"),
        )

        for event in invalid_events:
            with self.subTest(event=event):
                with self.assertRaises(
                    DomainValidationError
                ):
                    memory.record_event(event)

        self.assertEqual(memory.events, ())

    def test_event_identifiers_and_sequence_are_guarded(
        self,
    ) -> None:
        memory = InMemoryMissionMemory()
        first_event = self.make_event()
        memory.start(self.mission)
        memory.record_event(first_event)

        duplicate_identifier = self.make_event(
            event_id=first_event.event_id,
            sequence_number=2,
        )
        repeated_sequence = self.make_event(
            event_id="event_2",
            sequence_number=1,
        )

        for event in (
            duplicate_identifier,
            repeated_sequence,
        ):
            with self.subTest(event=event):
                with self.assertRaises(
                    InvariantViolationError
                ):
                    memory.record_event(event)

        self.assertEqual(memory.events, (first_event,))

    def test_successful_completion_preserves_recording(
        self,
    ) -> None:
        memory = InMemoryMissionMemory()
        event = self.make_event()
        completed = self.successful_mission()
        memory.start(self.mission)
        memory.record_pose(PathPhase.OUTBOUND, self.pose)
        memory.record_event(event)

        memory.complete(completed)

        self.assertIsNone(memory.active_mission)
        self.assertIs(memory.completed_mission, completed)
        self.assertEqual(memory.outbound_poses, (self.pose,))
        self.assertEqual(memory.events, (event,))
        self.assertEqual(
            memory.build_return_path().confirmed_poses,
            (self.pose,),
        )

    def test_failed_and_aborted_outcomes_can_complete(
        self,
    ) -> None:
        for status in (
            MissionStatus.FAILED,
            MissionStatus.ABORTED,
        ):
            with self.subTest(status=status):
                memory = InMemoryMissionMemory()
                completed = self.failure_mission(status)
                memory.start(self.mission)

                memory.complete(completed)

                self.assertIs(
                    memory.completed_mission,
                    completed,
                )

    def test_completion_requires_terminal_matching_mission(
        self,
    ) -> None:
        memory = InMemoryMissionMemory()
        memory.start(self.mission)

        with self.assertRaises(InvalidStateTransitionError):
            memory.complete(self.mission)

        mismatches: tuple[tuple[str, object], ...] = (
            ("mission_id", "mission_2"),
            ("robot_id", "robot_2"),
            ("target_id", "target_2"),
            ("target_position", Position(x=3, y=4)),
            ("base_position", Position(x=0, y=1)),
        )

        for field, value in mismatches:
            with self.subTest(field=field, value=value):
                with self.assertRaises(
                    DomainValidationError
                ):
                    memory.complete(
                        self.successful_mission(
                            **{field: value},
                        )
                    )

        self.assertIs(memory.active_mission, self.mission)
        self.assertIsNone(memory.completed_mission)

    def test_completed_recording_rejects_mutation(self) -> None:
        memory = InMemoryMissionMemory()
        completed = self.successful_mission()
        memory.start(self.mission)
        memory.complete(completed)

        with self.assertRaises(InvalidStateTransitionError):
            memory.record_pose(PathPhase.OUTBOUND, self.pose)

        with self.assertRaises(InvalidStateTransitionError):
            memory.record_event(self.make_event())

        with self.assertRaises(InvalidStateTransitionError):
            memory.complete(completed)

        with self.assertRaises(InvalidStateTransitionError):
            memory.start(self.mission)

        self.assertEqual(
            memory.build_return_path().confirmed_poses,
            (),
        )

    def test_reset_clears_state_and_allows_new_mission(
        self,
    ) -> None:
        memory = InMemoryMissionMemory()
        memory.start(self.mission)
        memory.record_pose(PathPhase.OUTBOUND, self.pose)
        memory.record_event(self.make_event())
        memory.complete(self.successful_mission())

        memory.reset()
        memory.reset()

        self.assertIsNone(memory.active_mission)
        self.assertIsNone(memory.completed_mission)
        self.assertEqual(memory.outbound_poses, ())
        self.assertEqual(memory.return_poses, ())
        self.assertEqual(memory.events, ())

        with self.assertRaises(InvalidStateTransitionError):
            memory.build_return_path()

        next_mission = Mission(
            mission_id="mission_2",
            robot_id="robot_2",
            target_id="target_2",
            target_position=Position(x=3, y=3),
            base_position=Position(x=1, y=1),
        )

        memory.start(next_mission)

        self.assertIs(memory.active_mission, next_mission)


if __name__ == "__main__":
    unittest.main()
