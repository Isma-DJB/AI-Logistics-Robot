"""Reference integration tests for Brain and Control."""

import unittest
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

from ai_logistics_robot import brain as brain_package
from ai_logistics_robot import control as control_package
from ai_logistics_robot.adapters.simulation.grid_world import (
    GridWorld,
)
from ai_logistics_robot.app.settings import load_settings
from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    BrainState,
    CommandType,
    MissionStatus,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.memory import InMemoryMissionMemory
from ai_logistics_robot.planning import AStarPlanner

_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
_REFERENCE_CONFIG = (
    _REPOSITORY_ROOT
    / "configs"
    / "simulation.yaml"
)


class ReplayClock:
    """Provide deterministic mission and collection time."""

    def __init__(self) -> None:
        self.wall_time = datetime(
            2026,
            8,
            16,
            8,
            0,
            tzinfo=UTC,
        )
        self.monotonic_time = 0.0
        self.waited_deadlines: list[float] = []

    def now(self) -> datetime:
        """Return the fixed deterministic wall time."""

        return self.wall_time

    def monotonic(self) -> float:
        """Return current deterministic monotonic time."""

        return self.monotonic_time

    def wait_until(self, deadline: float) -> None:
        """Reach and record one deterministic deadline."""

        self.waited_deadlines.append(deadline)
        self.monotonic_time = deadline


class RecordingSimulation:
    """Record commands while delegating to the reference GridWorld."""

    def __init__(self, grid_world: GridWorld) -> None:
        self.grid_world = grid_world
        self.applied_commands: list[MotionCommand] = []

    @property
    def current_pose(self) -> RobotPose:
        """Return the current confirmed simulated pose."""

        return self.grid_world.current_pose

    def reset(self) -> None:
        """Reset simulation state and command history."""

        self.grid_world.reset()
        self.applied_commands.clear()

    def read_world(self) -> GridMap:
        """Return the immutable reference world."""

        return self.grid_world.read_world()

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Record and execute one command."""

        self.applied_commands.append(command)
        return self.grid_world.apply_command(command)

    def advance_time(self, seconds: float) -> None:
        """Advance delegated deterministic simulation time."""

        self.grid_world.advance_time(seconds)


class ReferencePerception:
    """Observe reference pose and one activation edge."""

    def __init__(
        self,
        *,
        robot_id: str,
        simulation: RecordingSimulation,
        clock: ReplayClock,
    ) -> None:
        self.robot_id = robot_id
        self.simulation = simulation
        self.clock = clock
        self.target_states = [False, True]
        self.target_active = False

    def observe(self) -> PerceptionSnapshot:
        """Return one normalized reference observation."""

        if self.target_states:
            self.target_active = self.target_states.pop(0)

        return PerceptionSnapshot(
            robot_id=self.robot_id,
            captured_at=self.clock.now(),
            robot_pose=self.simulation.current_pose,
            observations=(),
            target_active=self.target_active,
            hazard_detected=False,
        )


class ReferenceMonitoring:
    """Retain published events for replay comparison."""

    def __init__(self) -> None:
        self.events: list[MissionEvent] = []

    def publish(self, event: MissionEvent) -> None:
        """Store one published event."""

        self.events.append(event)

    def events_for(
        self,
        mission_id: str,
    ) -> tuple[MissionEvent, ...]:
        """Return events belonging to one mission."""

        return tuple(
            event
            for event in self.events
            if event.mission_id == mission_id
        )


@dataclass(frozen=True, slots=True)
class ReplayTrace:
    """Comparable result of one complete reference replay."""

    status: SystemStatus
    completed_mission: Mission
    outbound_poses: tuple[RobotPose, ...]
    return_poses: tuple[RobotPose, ...]
    event_ids: tuple[str, ...]
    event_names: tuple[str, ...]
    event_states: tuple[BrainState, ...]
    command_types: tuple[CommandType, ...]
    waited_deadlines: tuple[float, ...]


class ReferenceMissionIntegrationTests(unittest.TestCase):
    """Verify public integration and deterministic replay."""

    def run_reference_mission(self) -> ReplayTrace:
        """Assemble and complete the configured reference mission."""

        settings = load_settings(_REFERENCE_CONFIG)
        clock = ReplayClock()
        simulation = RecordingSimulation(
            GridWorld(
                world=settings.grid_map,
                robot_id=settings.robot.robot_id,
                initial_pose=settings.robot.initial_pose,
            )
        )
        perception = ReferencePerception(
            robot_id=settings.robot.robot_id,
            simulation=simulation,
            clock=clock,
        )
        monitoring = ReferenceMonitoring()
        memory = InMemoryMissionMemory()
        control = control_package.SafeRobotControl(
            robot_id=settings.robot.robot_id,
            initial_pose=settings.robot.initial_pose,
            simulation=simulation,
            clock=clock,
        )
        brain = brain_package.DeterministicBrain(
            scenario_id=settings.scenario.scenario_id,
            robot_id=settings.robot.robot_id,
            target_id=settings.target.target_id,
            world=settings.grid_map,
            initial_pose=settings.robot.initial_pose,
            collection_duration_s=(
                settings.mission.collection_duration_s
            ),
            maximum_replans=(
                settings.mission.maximum_replans
            ),
            timeout_s=settings.mission.timeout_s,
            perception=perception,
            planning=AStarPlanner(),
            control=control,
            memory=memory,
            monitoring=monitoring,
            clock=clock,
        )

        for _ in range(500):
            brain.update()
            status = brain.get_status()

            if (
                status.brain_state
                is BrainState.WAITING_FOR_MISSION
                and status.mission_status
                is MissionStatus.SUCCESS
            ):
                break
        else:
            self.fail(
                "the reference mission did not complete "
                "within 500 updates"
            )

        completed_mission = memory.completed_mission

        self.assertIsNotNone(completed_mission)
        assert completed_mission is not None
        self.assertEqual(
            simulation.current_pose.position,
            settings.grid_map.base_position,
        )
        self.assertTrue(
            all(
                pose.position
                != settings.grid_map.target_position
                for pose in memory.outbound_poses
            )
        )
        self.assertEqual(
            len(memory.events),
            len(monitoring.events),
        )

        for stored, published in zip(
            memory.events,
            monitoring.events,
            strict=True,
        ):
            self.assertIs(stored, published)

        return ReplayTrace(
            status=status,
            completed_mission=completed_mission,
            outbound_poses=memory.outbound_poses,
            return_poses=memory.return_poses,
            event_ids=tuple(
                event.event_id
                for event in memory.events
            ),
            event_names=tuple(
                event.name
                for event in memory.events
            ),
            event_states=tuple(
                event.brain_state
                for event in memory.events
            ),
            command_types=tuple(
                command.command_type
                for command in simulation.applied_commands
            ),
            waited_deadlines=tuple(
                clock.waited_deadlines
            ),
        )

    def test_concrete_brain_and_control_are_publicly_exported(
        self,
    ) -> None:
        self.assertEqual(
            brain_package.__all__,
            ("DeterministicBrain",),
        )
        self.assertEqual(
            control_package.__all__,
            ("SafeRobotControl",),
        )
        self.assertEqual(
            brain_package.DeterministicBrain.__name__,
            "DeterministicBrain",
        )
        self.assertEqual(
            control_package.SafeRobotControl.__name__,
            "SafeRobotControl",
        )

    def test_reference_configuration_completes_successfully(
        self,
    ) -> None:
        trace = self.run_reference_mission()

        self.assertIs(
            trace.status.brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertIs(
            trace.completed_mission.status,
            MissionStatus.SUCCESS,
        )
        self.assertTrue(
            trace.completed_mission.collection_completed
        )
        self.assertTrue(
            trace.completed_mission.base_arrival_confirmed
        )
        self.assertIsNone(
            trace.completed_mission.terminal_reason
        )
        self.assertIsNone(trace.status.latest_error)
        self.assertFalse(
            trace.status.safety_status.latched
        )
        self.assertTrue(trace.outbound_poses)
        self.assertTrue(trace.return_poses)
        self.assertEqual(
            trace.event_names[0],
            "mission_started",
        )
        self.assertEqual(
            trace.event_names[-1],
            "mission_completed",
        )
        self.assertIn(
            "outbound_plan_created",
            trace.event_names,
        )
        self.assertIn(
            "collection_completed",
            trace.event_names,
        )
        self.assertIn(
            "return_path_prepared",
            trace.event_names,
        )
        self.assertEqual(
            trace.waited_deadlines,
            (3.0,),
        )

    def test_reference_mission_replays_identically(
        self,
    ) -> None:
        first_trace = self.run_reference_mission()
        second_trace = self.run_reference_mission()

        self.assertEqual(second_trace, first_trace)


if __name__ == "__main__":
    unittest.main()
