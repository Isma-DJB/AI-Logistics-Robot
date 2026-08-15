"""Unit tests for Brain outbound navigation and collection."""

import unittest
from dataclasses import replace
from datetime import UTC, datetime

from ai_logistics_robot.adapters.simulation.grid_world import (
    GridWorld,
)
from ai_logistics_robot.brain.deterministic_brain import (
    DeterministicBrain,
)
from ai_logistics_robot.control.safe_robot_control import (
    SafeRobotControl,
)
from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    BrainState,
    CommandType,
    Heading,
    PathPhase,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.memory import InMemoryMissionMemory
from ai_logistics_robot.planning import AStarPlanner


class NavigationClock:
    """Provide deterministic collection and event time."""

    def __init__(self) -> None:
        self.wall_time = datetime(
            2026,
            8,
            15,
            14,
            0,
            tzinfo=UTC,
        )
        self.monotonic_time = 20.0
        self.waited_deadlines: list[float] = []

    def now(self) -> datetime:
        """Return deterministic wall-clock time."""

        return self.wall_time

    def monotonic(self) -> float:
        """Return the current deterministic monotonic time."""

        return self.monotonic_time

    def wait_until(self, deadline: float) -> None:
        """Record and reach the requested deadline."""

        self.waited_deadlines.append(deadline)
        self.monotonic_time = deadline


class RecordingGridWorld:
    """Record commands while delegating rules to GridWorld."""

    def __init__(self, grid_world: GridWorld) -> None:
        self.grid_world = grid_world
        self.applied_commands: list[MotionCommand] = []
        self.reset_calls = 0

    @property
    def current_pose(self) -> RobotPose:
        """Return the confirmed GridWorld pose."""

        return self.grid_world.current_pose

    @property
    def elapsed_time_seconds(self) -> float:
        """Return confirmed simulated elapsed time."""

        return self.grid_world.elapsed_time_seconds

    def reset(self) -> None:
        """Reset the delegated GridWorld."""

        self.grid_world.reset()
        self.applied_commands.clear()
        self.reset_calls += 1

    def read_world(self) -> GridMap:
        """Return the delegated immutable world."""

        return self.grid_world.read_world()

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Record and delegate one command."""

        self.applied_commands.append(command)
        return self.grid_world.apply_command(command)

    def advance_time(self, seconds: float) -> None:
        """Delegate simulated-time advancement."""

        self.grid_world.advance_time(seconds)


class WorldPerception:
    """Observe the current simulated pose and scripted target state."""

    def __init__(
        self,
        *,
        robot_id: str,
        simulation: RecordingGridWorld,
        clock: NavigationClock,
        target_states: list[bool],
    ) -> None:
        self.robot_id = robot_id
        self.simulation = simulation
        self.clock = clock
        self.target_states = target_states
        self.last_target_state = False
        self.observe_calls = 0

    def observe(self) -> PerceptionSnapshot:
        """Return one current normalized snapshot."""

        self.observe_calls += 1

        if self.target_states:
            self.last_target_state = self.target_states.pop(0)

        return PerceptionSnapshot(
            robot_id=self.robot_id,
            captured_at=self.clock.now(),
            robot_pose=self.simulation.current_pose,
            observations=(),
            target_active=self.last_target_state,
            hazard_detected=False,
        )


class NavigationMonitoring:
    """Record ordered events published by the Brain."""

    def __init__(self) -> None:
        self.events: list[MissionEvent] = []

    def publish(self, event: MissionEvent) -> None:
        """Record one event."""

        self.events.append(event)

    def events_for(
        self,
        mission_id: str,
    ) -> tuple[MissionEvent, ...]:
        """Return events for one mission."""

        return tuple(
            event
            for event in self.events
            if event.mission_id == mission_id
        )


class DeterministicBrainOutboundTests(unittest.TestCase):
    """Verify outbound plan execution, replanning, and collection."""

    def setUp(self) -> None:
        self.robot_id = "robot_1"
        self.target_id = "target_1"
        self.world = GridMap(
            width=6,
            height=6,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=Position(x=4, y=4),
        )

    def build_brain(
        self,
        *,
        initial_heading: Heading = Heading.NORTH,
        hidden_obstacles: frozenset[Position] = frozenset(),
        target_states: list[bool] | None = None,
    ) -> tuple[
        DeterministicBrain,
        RecordingGridWorld,
        InMemoryMissionMemory,
        NavigationMonitoring,
        NavigationClock,
    ]:
        """Assemble a complete headless outbound test system."""

        initial_pose = RobotPose(
            position=self.world.base_position,
            heading=initial_heading,
        )
        simulation_world = replace(
            self.world,
            obstacles=hidden_obstacles,
        )
        simulation = RecordingGridWorld(
            GridWorld(
                world=simulation_world,
                robot_id=self.robot_id,
                initial_pose=initial_pose,
            )
        )
        clock = NavigationClock()
        perception = WorldPerception(
            robot_id=self.robot_id,
            simulation=simulation,
            clock=clock,
            target_states=(
                [False, True]
                if target_states is None
                else target_states
            ),
        )
        memory = InMemoryMissionMemory()
        monitoring = NavigationMonitoring()
        control = SafeRobotControl(
            robot_id=self.robot_id,
            initial_pose=initial_pose,
            simulation=simulation,
            clock=clock,
        )
        brain = DeterministicBrain(
            scenario_id="v1-reference",
            robot_id=self.robot_id,
            target_id=self.target_id,
            world=self.world,
            initial_pose=initial_pose,
            collection_duration_s=3.0,
            maximum_replans=None,
            timeout_s=None,
            perception=perception,
            planning=AStarPlanner(),
            control=control,
            memory=memory,
            monitoring=monitoring,
            clock=clock,
        )

        return (
            brain,
            simulation,
            memory,
            monitoring,
            clock,
        )

    def activate_mission(
        self,
        brain: DeterministicBrain,
    ) -> None:
        """Advance through initialization and one activation edge."""

        brain.update()
        brain.update()
        brain.update()

        self.assertIs(
            brain.get_status().brain_state,
            BrainState.OUTBOUND_PLANNING,
        )

    def advance_to_state(
        self,
        brain: DeterministicBrain,
        expected_state: BrainState,
        *,
        maximum_updates: int = 100,
    ) -> None:
        """Advance until one expected state is reached."""

        for _ in range(maximum_updates):
            if brain.get_status().brain_state is expected_state:
                return

            brain.update()

        self.fail(
            f"Brain did not reach {expected_state} "
            f"within {maximum_updates} updates."
        )

    def test_outbound_plan_is_created_in_its_own_cycle(
        self,
    ) -> None:
        brain, _, memory, monitoring, _ = self.build_brain()
        self.activate_mission(brain)

        brain.update()

        status = brain.get_status()
        plan = status.active_plan

        self.assertIs(
            status.brain_state,
            BrainState.OUTBOUND_NAVIGATION,
        )
        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertIs(plan.phase, PathPhase.OUTBOUND)
        self.assertEqual(plan.version, 1)
        self.assertEqual(
            plan.positions[0],
            self.world.base_position,
        )
        self.assertEqual(
            plan.goal,
            Position(x=4, y=3),
        )
        self.assertIn(
            plan.goal,
            self.world.authorized_arrival_positions,
        )
        self.assertEqual(
            tuple(event.name for event in memory.events),
            (
                "mission_started",
                "outbound_plan_created",
            ),
        )
        self.assertIs(
            monitoring.events[-1],
            memory.events[-1],
        )

    def test_navigation_executes_one_confirmed_command_per_cycle(
        self,
    ) -> None:
        brain, simulation, memory, _, _ = self.build_brain(
            target_states=[False, True, False]
        )
        self.activate_mission(brain)
        brain.update()
        simulation.applied_commands.clear()

        brain.update()

        self.assertEqual(len(simulation.applied_commands), 1)
        self.assertIs(
            simulation.applied_commands[0].command_type,
            CommandType.MOVE_FORWARD,
        )
        self.assertEqual(
            simulation.current_pose,
            RobotPose(
                position=Position(x=1, y=2),
                heading=Heading.NORTH,
            ),
        )
        self.assertEqual(len(memory.outbound_poses), 2)
        self.assertEqual(
            memory.outbound_poses[-1],
            simulation.current_pose,
        )
        self.assertEqual(
            memory.events[-1].name,
            "outbound_step_confirmed",
        )
        self.assertIs(
            brain.get_status().brain_state,
            BrainState.OUTBOUND_NAVIGATION,
        )

    def test_opposite_heading_uses_deterministic_right_turns(
        self,
    ) -> None:
        brain, simulation, memory, _, _ = self.build_brain(
            initial_heading=Heading.SOUTH
        )
        self.activate_mission(brain)
        brain.update()
        simulation.applied_commands.clear()

        brain.update()
        brain.update()
        brain.update()

        self.assertEqual(
            tuple(
                command.command_type
                for command in simulation.applied_commands
            ),
            (
                CommandType.TURN_RIGHT,
                CommandType.TURN_RIGHT,
                CommandType.MOVE_FORWARD,
            ),
        )
        self.assertEqual(
            simulation.current_pose.position,
            Position(x=1, y=2),
        )
        self.assertEqual(len(memory.outbound_poses), 4)

    def test_arrival_enters_collection_without_waiting_same_cycle(
        self,
    ) -> None:
        brain, simulation, memory, _, clock = self.build_brain()
        self.activate_mission(brain)

        self.advance_to_state(
            brain,
            BrainState.COLLECTION,
        )

        status = brain.get_status()

        self.assertEqual(
            simulation.current_pose.position,
            Position(x=4, y=3),
        )
        self.assertIsNone(status.active_plan)
        self.assertEqual(clock.waited_deadlines, [])
        self.assertIs(
            simulation.applied_commands[-1].command_type,
            CommandType.MOVE_FORWARD,
        )
        self.assertEqual(
            memory.events[-1].name,
            "arrival_confirmed",
        )

    def test_collection_stops_and_waits_configured_duration(
        self,
    ) -> None:
        brain, simulation, memory, _, clock = self.build_brain()
        self.activate_mission(brain)
        self.advance_to_state(
            brain,
            BrainState.COLLECTION,
        )
        simulation.applied_commands.clear()

        brain.update()

        self.assertIs(
            brain.get_status().brain_state,
            BrainState.RETURN_PREPARATION,
        )
        self.assertEqual(clock.waited_deadlines, [23.0])
        self.assertEqual(len(simulation.applied_commands), 1)
        self.assertIs(
            simulation.applied_commands[0].command_type,
            CommandType.STOP,
        )
        self.assertEqual(
            memory.events[-1].name,
            "collection_completed",
        )

    def test_blocked_step_is_not_recorded_and_replans_detour(
        self,
    ) -> None:
        hidden_obstacle = Position(x=1, y=2)
        brain, simulation, memory, _, _ = self.build_brain(
            hidden_obstacles=frozenset({hidden_obstacle})
        )
        self.activate_mission(brain)
        brain.update()
        simulation.applied_commands.clear()

        brain.update()

        self.assertIs(
            brain.get_status().brain_state,
            BrainState.OUTBOUND_REPLANNING,
        )
        self.assertEqual(
            simulation.current_pose.position,
            self.world.base_position,
        )
        self.assertEqual(
            memory.outbound_poses,
            (
                RobotPose(
                    position=self.world.base_position,
                    heading=Heading.NORTH,
                ),
            ),
        )
        self.assertEqual(len(simulation.applied_commands), 1)
        self.assertIs(
            simulation.applied_commands[0].command_type,
            CommandType.MOVE_FORWARD,
        )
        self.assertEqual(
            memory.events[-1].name,
            "outbound_step_blocked",
        )

        brain.update()

        status = brain.get_status()
        plan = status.active_plan

        self.assertIs(
            status.brain_state,
            BrainState.OUTBOUND_NAVIGATION,
        )
        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertIs(plan.phase, PathPhase.DETOUR)
        self.assertEqual(plan.version, 2)
        self.assertNotIn(hidden_obstacle, plan.positions)
        self.assertEqual(
            memory.events[-1].name,
            "outbound_plan_recreated",
        )
        self.assertEqual(len(simulation.applied_commands), 1)


if __name__ == "__main__":
    unittest.main()
