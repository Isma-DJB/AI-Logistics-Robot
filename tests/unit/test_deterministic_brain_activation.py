"""Unit tests for deterministic Brain initialization and activation."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.brain.deterministic_brain import (
    DeterministicBrain,
)
from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    BrainState,
    CommandStatus,
    FailureReason,
    Heading,
    MissionStatus,
    PathPhase,
    SafetySeverity,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.memory import InMemoryMissionMemory
from ai_logistics_robot.ports.brain_port import BrainPort


class BrainClock:
    """Provide deterministic wall-clock and monotonic time."""

    def __init__(self) -> None:
        self.current_time = datetime(
            2026,
            8,
            15,
            12,
            0,
            tzinfo=UTC,
        )
        self.now_calls = 0
        self.waited_deadlines: list[float] = []

    def now(self) -> datetime:
        """Return deterministic wall-clock time."""

        self.now_calls += 1
        return self.current_time

    def monotonic(self) -> float:
        """Return deterministic monotonic time."""

        return 20.0

    def wait_until(self, deadline: float) -> None:
        """Record a deadline without blocking."""

        self.waited_deadlines.append(deadline)


class QueuedPerception:
    """Return normalized snapshots in configured order."""

    def __init__(
        self,
        snapshots: list[PerceptionSnapshot],
    ) -> None:
        self.snapshots = snapshots
        self.observe_calls = 0

    def observe(self) -> PerceptionSnapshot:
        """Return the next queued snapshot."""

        self.observe_calls += 1

        if not self.snapshots:
            raise AssertionError(
                "no perception snapshot remains"
            )

        return self.snapshots.pop(0)


class UnusedPlanning:
    """Provide a structurally valid PlanningPort fake."""

    def __init__(self) -> None:
        self.create_calls = 0

    def create_plan(
        self,
        *,
        mission_id: str,
        robot_id: str,
        start_pose: RobotPose,
        authorized_goals: tuple[Position, ...],
        world: GridMap,
        phase: PathPhase,
        version: int,
    ) -> PathPlan:
        """Return a minimal deterministic plan if called."""

        self.create_calls += 1
        goal = authorized_goals[0]

        return PathPlan(
            mission_id=mission_id,
            robot_id=robot_id,
            phase=phase,
            version=version,
            positions=(start_pose.position, goal),
            goal=goal,
        )


class RecordingControl:
    """Record Brain control effects and expose safety state."""

    def __init__(
        self,
        *,
        robot_id: str,
        initial_pose: RobotPose,
        timestamp: datetime,
    ) -> None:
        self.robot_id = robot_id
        self.current_pose = initial_pose
        self.stop_calls = 0
        self.executed_commands: list[MotionCommand] = []
        self.emergency_reasons: list[FailureReason] = []
        self.reset_safety_calls = 0
        self.status_calls = 0
        self.safety_status = SafetyStatus(
            robot_id=robot_id,
            updated_at=timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )

    def execute_step(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Return a successful unchanged-pose result."""

        self.executed_commands.append(command)

        return CommandResult(
            command=command,
            status=CommandStatus.SUCCESS,
            pose_before=self.current_pose,
            pose_after=self.current_pose,
        )

    def stop(self) -> None:
        """Record a normal stop request."""

        self.stop_calls += 1

    def emergency_stop(
        self,
        reason: FailureReason,
    ) -> SafetyStatus:
        """Latch deterministic local safety."""

        self.emergency_reasons.append(reason)
        self.safety_status = SafetyStatus(
            robot_id=self.robot_id,
            updated_at=self.safety_status.updated_at,
            latched=True,
            severity=SafetySeverity.CRITICAL,
            reason=reason,
        )
        return self.safety_status

    def get_safety_status(self) -> SafetyStatus:
        """Return current safety without changing control."""

        self.status_calls += 1
        return self.safety_status

    def reset_safety_latch(self) -> SafetyStatus:
        """Record explicit external safety rearm."""

        self.reset_safety_calls += 1
        self.safety_status = SafetyStatus(
            robot_id=self.robot_id,
            updated_at=self.safety_status.updated_at,
            latched=False,
            severity=SafetySeverity.INFO,
        )
        return self.safety_status


class RecordingMonitoring:
    """Record published mission events in accepted order."""

    def __init__(self) -> None:
        self.events: list[MissionEvent] = []

    def publish(self, event: MissionEvent) -> None:
        """Record one published event."""

        self.events.append(event)

    def events_for(
        self,
        mission_id: str,
    ) -> tuple[MissionEvent, ...]:
        """Return events matching one mission identity."""

        return tuple(
            event
            for event in self.events
            if event.mission_id == mission_id
        )


class DeterministicBrainActivationTests(unittest.TestCase):
    """Verify initial state, target edge, mission, and reset."""

    def setUp(self) -> None:
        self.scenario_id = "v1-reference"
        self.robot_id = "robot_1"
        self.target_id = "target_1"
        self.initial_pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.world = GridMap(
            width=6,
            height=6,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=Position(x=5, y=5),
        )
        self.clock = BrainClock()

    def snapshot(
        self,
        *,
        target_active: bool,
        hazard_detected: bool = False,
        pose: RobotPose | None = None,
    ) -> PerceptionSnapshot:
        """Build one normalized perception snapshot."""

        return PerceptionSnapshot(
            robot_id=self.robot_id,
            captured_at=self.clock.current_time,
            robot_pose=(
                self.initial_pose
                if pose is None
                else pose
            ),
            observations=(),
            target_active=target_active,
            hazard_detected=hazard_detected,
        )

    def build_brain(
        self,
        snapshots: list[PerceptionSnapshot],
    ) -> tuple[
        DeterministicBrain,
        QueuedPerception,
        UnusedPlanning,
        RecordingControl,
        InMemoryMissionMemory,
        RecordingMonitoring,
    ]:
        """Build the Brain with inspectable port implementations."""

        perception = QueuedPerception(snapshots)
        planning = UnusedPlanning()
        control = RecordingControl(
            robot_id=self.robot_id,
            initial_pose=self.initial_pose,
            timestamp=self.clock.current_time,
        )
        memory = InMemoryMissionMemory()
        monitoring = RecordingMonitoring()
        brain = DeterministicBrain(
            scenario_id=self.scenario_id,
            robot_id=self.robot_id,
            target_id=self.target_id,
            world=self.world,
            initial_pose=self.initial_pose,
            collection_duration_s=3.0,
            maximum_replans=None,
            timeout_s=None,
            perception=perception,
            planning=planning,
            control=control,
            memory=memory,
            monitoring=monitoring,
            clock=self.clock,
        )

        return (
            brain,
            perception,
            planning,
            control,
            memory,
            monitoring,
        )

    def test_initial_status_is_read_only_and_satisfies_port(
        self,
    ) -> None:
        (
            brain,
            perception,
            _,
            control,
            _,
            _,
        ) = self.build_brain([])

        self.assertIsInstance(brain, BrainPort)

        first_status = brain.get_status()
        second_status = brain.get_status()

        self.assertIs(
            first_status.brain_state,
            BrainState.INITIALIZATION,
        )
        self.assertEqual(
            first_status.robot_pose,
            self.initial_pose,
        )
        self.assertIsNone(first_status.mission_id)
        self.assertIsNone(first_status.mission_status)
        self.assertIsNone(first_status.active_plan)
        self.assertIsNone(first_status.latest_error)
        self.assertEqual(
            second_status.brain_state,
            first_status.brain_state,
        )
        self.assertEqual(control.stop_calls, 0)
        self.assertEqual(control.executed_commands, [])
        self.assertEqual(perception.observe_calls, 0)

    def test_initialization_performs_only_stationary_transition(
        self,
    ) -> None:
        brain, perception, _, control, memory, monitoring = (
            self.build_brain([])
        )

        brain.update()

        status = brain.get_status()

        self.assertIs(
            status.brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertEqual(control.stop_calls, 1)
        self.assertEqual(perception.observe_calls, 0)
        self.assertIsNone(memory.active_mission)
        self.assertEqual(memory.events, ())
        self.assertEqual(monitoring.events, [])

    def test_first_inactive_observation_establishes_baseline(
        self,
    ) -> None:
        brain, perception, _, control, memory, _ = (
            self.build_brain(
                [self.snapshot(target_active=False)]
            )
        )
        brain.update()

        brain.update()

        self.assertIs(
            brain.get_status().brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertEqual(perception.observe_calls, 1)
        self.assertEqual(control.stop_calls, 2)
        self.assertIsNone(memory.active_mission)

    def test_inactive_to_active_edge_creates_deterministic_mission(
        self,
    ) -> None:
        brain, perception, planning, control, memory, monitoring = (
            self.build_brain(
                [
                    self.snapshot(target_active=False),
                    self.snapshot(target_active=True),
                ]
            )
        )
        brain.update()
        brain.update()

        brain.update()

        status = brain.get_status()
        mission = memory.active_mission

        self.assertIsNotNone(mission)
        assert mission is not None
        self.assertEqual(
            mission.mission_id,
            "v1-reference-mission-1",
        )
        self.assertEqual(mission.robot_id, self.robot_id)
        self.assertEqual(mission.target_id, self.target_id)
        self.assertEqual(
            mission.target_position,
            self.world.target_position,
        )
        self.assertEqual(
            mission.base_position,
            self.world.base_position,
        )
        self.assertIs(mission.status, MissionStatus.ACTIVE)
        self.assertIs(
            status.brain_state,
            BrainState.OUTBOUND_PLANNING,
        )
        self.assertEqual(status.mission_id, mission.mission_id)
        self.assertIs(
            status.mission_status,
            MissionStatus.ACTIVE,
        )
        self.assertEqual(
            memory.outbound_poses,
            (self.initial_pose,),
        )
        self.assertEqual(perception.observe_calls, 2)
        self.assertEqual(control.stop_calls, 3)
        self.assertEqual(planning.create_calls, 0)

        self.assertEqual(len(memory.events), 1)
        self.assertEqual(len(monitoring.events), 1)
        event = memory.events[0]

        self.assertIs(monitoring.events[0], event)
        self.assertEqual(
            event.event_id,
            "v1-reference-mission-1-event-1",
        )
        self.assertEqual(event.sequence_number, 1)
        self.assertEqual(event.name, "mission_started")
        self.assertIs(
            event.brain_state,
            BrainState.OUTBOUND_PLANNING,
        )

    def test_initial_active_level_requires_a_future_rising_edge(
        self,
    ) -> None:
        brain, _, _, _, memory, _ = self.build_brain(
            [
                self.snapshot(target_active=True),
                self.snapshot(target_active=True),
                self.snapshot(target_active=False),
                self.snapshot(target_active=True),
            ]
        )
        brain.update()

        brain.update()
        brain.update()
        brain.update()

        self.assertIsNone(memory.active_mission)
        self.assertIs(
            brain.get_status().brain_state,
            BrainState.WAITING_FOR_MISSION,
        )

        brain.update()

        self.assertIsNotNone(memory.active_mission)
        self.assertEqual(
            memory.active_mission.mission_id,
            "v1-reference-mission-1",
        )

    def test_active_mission_is_preserved_as_orchestration_advances(
        self,
    ) -> None:
        brain, perception, _, _, memory, monitoring = (
            self.build_brain(
                [
                    self.snapshot(target_active=False),
                    self.snapshot(target_active=True),
                    self.snapshot(target_active=False),
                    self.snapshot(target_active=True),
                ]
            )
        )
        brain.update()
        brain.update()
        brain.update()
        accepted_mission = memory.active_mission

        brain.update()

        self.assertIs(memory.active_mission, accepted_mission)
        self.assertIs(
            brain.get_status().brain_state,
            BrainState.OUTBOUND_NAVIGATION,
        )
        self.assertEqual(perception.observe_calls, 2)
        self.assertEqual(
            tuple(
                event.name
                for event in monitoring.events
            ),
            (
                "mission_started",
                "outbound_plan_created",
            ),
        )
    def test_reset_clears_state_without_rearming_control(
        self,
    ) -> None:
        brain, _, _, control, memory, monitoring = (
            self.build_brain(
                [
                    self.snapshot(target_active=False),
                    self.snapshot(target_active=True),
                    self.snapshot(target_active=False),
                    self.snapshot(target_active=True),
                ]
            )
        )
        brain.update()
        brain.update()
        brain.update()

        brain.reset()

        reset_status = brain.get_status()

        self.assertIs(
            reset_status.brain_state,
            BrainState.INITIALIZATION,
        )
        self.assertIsNone(reset_status.mission_id)
        self.assertIsNone(memory.active_mission)
        self.assertEqual(memory.outbound_poses, ())
        self.assertEqual(memory.events, ())
        self.assertEqual(control.reset_safety_calls, 0)
        self.assertEqual(len(monitoring.events), 1)

        brain.update()
        brain.update()
        brain.update()

        next_mission = memory.active_mission

        self.assertIsNotNone(next_mission)
        assert next_mission is not None
        self.assertEqual(
            next_mission.mission_id,
            "v1-reference-mission-2",
        )
        self.assertEqual(len(monitoring.events), 2)
        self.assertEqual(
            monitoring.events[-1].sequence_number,
            1,
        )


if __name__ == "__main__":
    unittest.main()
