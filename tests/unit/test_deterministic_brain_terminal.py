"""Unit tests for Brain terminal and exceptional mission behavior."""

import unittest
from dataclasses import replace

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
    CommandStatus,
    CommandType,
    FailureReason,
    Heading,
    MissionStatus,
    PathPhase,
)
from ai_logistics_robot.domain.errors import NoPathError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.memory import InMemoryMissionMemory
from ai_logistics_robot.planning import AStarPlanner
from tests.unit.test_deterministic_brain_outbound import (
    NavigationClock,
    NavigationMonitoring,
    RecordingGridWorld,
    WorldPerception,
)


class NoPathPlanning:
    """Raise the normalized no-path domain failure."""

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
        """Reject every requested plan."""

        self.create_calls += 1
        raise NoPathError("no authorized goal is reachable")


class OutOfBoundsSimulation(RecordingGridWorld):
    """Return one unexpected out-of-bounds movement result."""

    def __init__(self, grid_world: GridWorld) -> None:
        super().__init__(grid_world)
        self.failure_returned = False

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Fail the first forward movement unexpectedly."""

        self.applied_commands.append(command)

        if (
            not self.failure_returned
            and command.command_type
            is CommandType.MOVE_FORWARD
        ):
            self.failure_returned = True
            pose = self.current_pose

            return CommandResult(
                command=command,
                status=CommandStatus.FAILED,
                pose_before=pose,
                pose_after=pose,
                failure_reason=FailureReason.OUT_OF_BOUNDS,
            )

        return self.grid_world.apply_command(command)


class ReturnBlockingSimulation(RecordingGridWorld):
    """Reject one specific movement during return navigation."""

    def __init__(self, grid_world: GridWorld) -> None:
        super().__init__(grid_world)
        self.return_blocked = False
        self.blocked_position = Position(x=3, y=3)

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Block the first westward return movement."""

        self.applied_commands.append(command)
        pose = self.current_pose

        if (
            not self.return_blocked
            and command.command_type
            is CommandType.MOVE_FORWARD
            and pose.position == Position(x=4, y=3)
            and pose.heading is Heading.WEST
        ):
            self.return_blocked = True

            return CommandResult(
                command=command,
                status=CommandStatus.FAILED,
                pose_before=pose,
                pose_after=pose,
                failure_reason=FailureReason.BLOCKED,
            )

        return self.grid_world.apply_command(command)


class DeterministicBrainTerminalTests(unittest.TestCase):
    """Verify terminal outcomes and exceptional transitions."""

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
        self.initial_pose = RobotPose(
            position=self.world.base_position,
            heading=Heading.NORTH,
        )

    def build_brain(
        self,
        *,
        planning: AStarPlanner | NoPathPlanning | None = None,
        simulation: RecordingGridWorld | None = None,
        hidden_obstacles: frozenset[Position] = frozenset(),
        maximum_replans: int | None = None,
        timeout_s: float | None = None,
    ) -> tuple[
        DeterministicBrain,
        RecordingGridWorld,
        InMemoryMissionMemory,
        NavigationMonitoring,
        NavigationClock,
        SafeRobotControl,
    ]:
        """Assemble an inspectable deterministic mission system."""

        selected_simulation = simulation

        if selected_simulation is None:
            simulation_world = replace(
                self.world,
                obstacles=hidden_obstacles,
            )
            selected_simulation = RecordingGridWorld(
                GridWorld(
                    world=simulation_world,
                    robot_id=self.robot_id,
                    initial_pose=self.initial_pose,
                )
            )

        clock = NavigationClock()
        perception = WorldPerception(
            robot_id=self.robot_id,
            simulation=selected_simulation,
            clock=clock,
            target_states=[False, True],
        )
        memory = InMemoryMissionMemory()
        monitoring = NavigationMonitoring()
        control = SafeRobotControl(
            robot_id=self.robot_id,
            initial_pose=self.initial_pose,
            simulation=selected_simulation,
            clock=clock,
        )
        brain = DeterministicBrain(
            scenario_id="v1-reference",
            robot_id=self.robot_id,
            target_id=self.target_id,
            world=self.world,
            initial_pose=self.initial_pose,
            collection_duration_s=3.0,
            maximum_replans=maximum_replans,
            timeout_s=timeout_s,
            perception=perception,
            planning=(
                AStarPlanner()
                if planning is None
                else planning
            ),
            control=control,
            memory=memory,
            monitoring=monitoring,
            clock=clock,
        )

        return (
            brain,
            selected_simulation,
            memory,
            monitoring,
            clock,
            control,
        )

    def activate_mission(
        self,
        brain: DeterministicBrain,
    ) -> None:
        """Advance through one accepted activation edge."""

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
        maximum_updates: int = 150,
    ) -> None:
        """Advance until the expected state is observable."""

        for _ in range(maximum_updates):
            if brain.get_status().brain_state is expected_state:
                return

            brain.update()

        self.fail(
            f"Brain did not reach {expected_state} "
            f"within {maximum_updates} updates."
        )

    def test_completed_state_finalizes_success_and_returns_waiting(
        self,
    ) -> None:
        brain, _, memory, monitoring, _, _ = (
            self.build_brain()
        )
        self.activate_mission(brain)
        self.advance_to_state(
            brain,
            BrainState.MISSION_COMPLETED,
        )

        brain.update()

        status = brain.get_status()
        completed_mission = memory.completed_mission

        self.assertIs(
            status.brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertIs(
            status.mission_status,
            MissionStatus.SUCCESS,
        )
        self.assertIsNotNone(completed_mission)
        assert completed_mission is not None
        self.assertIs(
            completed_mission.status,
            MissionStatus.SUCCESS,
        )
        self.assertTrue(
            completed_mission.collection_completed
        )
        self.assertTrue(
            completed_mission.base_arrival_confirmed
        )
        self.assertIsNone(
            completed_mission.terminal_reason
        )
        self.assertEqual(
            memory.events[-1].name,
            "mission_completed",
        )
        self.assertIs(
            monitoring.events[-1],
            memory.events[-1],
        )

    def test_no_path_completes_failed_mission(
        self,
    ) -> None:
        planning = NoPathPlanning()
        brain, _, memory, _, _, _ = self.build_brain(
            planning=planning
        )
        self.activate_mission(brain)

        brain.update()

        status = brain.get_status()
        failed_mission = memory.completed_mission

        self.assertIs(
            status.brain_state,
            BrainState.MISSION_FAILED,
        )
        self.assertIs(
            status.latest_error,
            FailureReason.NO_PATH,
        )
        self.assertIsNotNone(failed_mission)
        assert failed_mission is not None
        self.assertIs(
            failed_mission.status,
            MissionStatus.FAILED,
        )
        self.assertIs(
            failed_mission.terminal_reason,
            FailureReason.NO_PATH,
        )
        self.assertEqual(planning.create_calls, 1)
        self.assertEqual(
            memory.events[-1].name,
            "mission_failed",
        )

    def test_replanning_limit_exhaustion_fails_as_blocked(
        self,
    ) -> None:
        hidden_obstacles = frozenset(
            {
                Position(x=1, y=2),
                Position(x=2, y=1),
            }
        )
        brain, _, memory, _, _, _ = self.build_brain(
            hidden_obstacles=hidden_obstacles,
            maximum_replans=1,
        )
        self.activate_mission(brain)

        self.advance_to_state(
            brain,
            BrainState.MISSION_FAILED,
        )

        failed_mission = memory.completed_mission

        self.assertIsNotNone(failed_mission)
        assert failed_mission is not None
        self.assertIs(
            failed_mission.terminal_reason,
            FailureReason.BLOCKED,
        )
        self.assertTrue(memory.outbound_poses)
        self.assertTrue(
            all(
                pose.position == self.world.base_position
                for pose in memory.outbound_poses
            )
        )
        self.assertEqual(
            memory.events[-1].name,
            "mission_failed",
        )

    def test_zero_timeout_fails_before_outbound_planning(
        self,
    ) -> None:
        brain, simulation, memory, _, _, _ = (
            self.build_brain(timeout_s=0.0)
        )
        self.activate_mission(brain)
        simulation.applied_commands.clear()

        brain.update()

        status = brain.get_status()
        failed_mission = memory.completed_mission

        self.assertIs(
            status.brain_state,
            BrainState.MISSION_FAILED,
        )
        self.assertIs(
            status.latest_error,
            FailureReason.TIMEOUT,
        )
        self.assertIsNotNone(failed_mission)
        assert failed_mission is not None
        self.assertIs(
            failed_mission.terminal_reason,
            FailureReason.TIMEOUT,
        )
        self.assertEqual(len(simulation.applied_commands), 1)
        self.assertIs(
            simulation.applied_commands[0].command_type,
            CommandType.STOP,
        )

    def test_out_of_bounds_result_enters_system_error(
        self,
    ) -> None:
        simulation = OutOfBoundsSimulation(
            GridWorld(
                world=self.world,
                robot_id=self.robot_id,
                initial_pose=self.initial_pose,
            )
        )
        brain, _, memory, _, _, _ = self.build_brain(
            simulation=simulation
        )
        self.activate_mission(brain)
        brain.update()

        brain.update()

        status = brain.get_status()
        failed_mission = memory.completed_mission

        self.assertIs(
            status.brain_state,
            BrainState.SYSTEM_ERROR,
        )
        self.assertIs(
            status.latest_error,
            FailureReason.OUT_OF_BOUNDS,
        )
        self.assertIsNotNone(failed_mission)
        assert failed_mission is not None
        self.assertIs(
            failed_mission.status,
            MissionStatus.FAILED,
        )
        self.assertIs(
            failed_mission.terminal_reason,
            FailureReason.OUT_OF_BOUNDS,
        )
        self.assertEqual(
            memory.events[-1].name,
            "system_error",
        )

    def test_external_emergency_aborts_without_auto_recovery(
        self,
    ) -> None:
        brain, _, memory, _, _, control = self.build_brain()
        self.activate_mission(brain)
        brain.update()

        control.emergency_stop(
            FailureReason.EMERGENCY_STOP
        )
        brain.update()

        status = brain.get_status()
        aborted_mission = memory.completed_mission

        self.assertIs(
            status.brain_state,
            BrainState.SAFETY_STOP,
        )
        self.assertIs(
            status.latest_error,
            FailureReason.EMERGENCY_STOP,
        )
        self.assertIsNotNone(aborted_mission)
        assert aborted_mission is not None
        self.assertIs(
            aborted_mission.status,
            MissionStatus.ABORTED,
        )
        self.assertIs(
            aborted_mission.terminal_reason,
            FailureReason.EMERGENCY_STOP,
        )
        self.assertEqual(
            memory.events[-1].name,
            "safety_stop",
        )

        control.reset_safety_latch()
        brain.update()

        self.assertIs(
            brain.get_status().brain_state,
            BrainState.SAFETY_STOP,
        )
        self.assertIs(
            memory.completed_mission,
            aborted_mission,
        )

    def test_blocked_return_replans_detour_to_base(
        self,
    ) -> None:
        simulation = ReturnBlockingSimulation(
            GridWorld(
                world=self.world,
                robot_id=self.robot_id,
                initial_pose=self.initial_pose,
            )
        )
        brain, _, memory, _, _, _ = self.build_brain(
            simulation=simulation
        )
        self.activate_mission(brain)
        self.advance_to_state(
            brain,
            BrainState.RETURN_NAVIGATION,
        )
        simulation.applied_commands.clear()

        brain.update()
        brain.update()
        brain.update()

        self.assertIs(
            brain.get_status().brain_state,
            BrainState.RETURN_REPLANNING,
        )
        self.assertEqual(len(memory.return_poses), 2)
        self.assertEqual(
            memory.events[-1].name,
            "return_step_blocked",
        )

        brain.update()

        status = brain.get_status()
        plan = status.active_plan

        self.assertIs(
            status.brain_state,
            BrainState.RETURN_NAVIGATION,
        )
        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertIs(plan.phase, PathPhase.DETOUR)
        self.assertEqual(plan.version, 2)
        self.assertEqual(
            plan.goal,
            self.world.base_position,
        )
        self.assertNotIn(
            simulation.blocked_position,
            plan.positions,
        )
        self.assertEqual(
            memory.events[-1].name,
            "return_plan_recreated",
        )


if __name__ == "__main__":
    unittest.main()
