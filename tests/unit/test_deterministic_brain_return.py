"""Unit tests for deterministic nominal return navigation."""

import unittest

from ai_logistics_robot.adapters.simulation.grid_world import (
    GridWorld,
)
from ai_logistics_robot.brain.deterministic_brain import (
    DeterministicBrain,
)
from ai_logistics_robot.control.safe_robot_control import (
    SafeRobotControl,
)
from ai_logistics_robot.domain.enums import (
    BrainState,
    CommandType,
    Heading,
    PathPhase,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.memory import InMemoryMissionMemory
from ai_logistics_robot.planning import AStarPlanner
from tests.unit.test_deterministic_brain_outbound import (
    NavigationClock,
    NavigationMonitoring,
    RecordingGridWorld,
    WorldPerception,
)


class DeterministicBrainReturnTests(unittest.TestCase):
    """Verify reverse-path preparation and confirmed return travel."""

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
    ) -> tuple[
        DeterministicBrain,
        RecordingGridWorld,
        InMemoryMissionMemory,
        NavigationMonitoring,
        NavigationClock,
    ]:
        """Assemble a complete deterministic nominal mission."""

        simulation = RecordingGridWorld(
            GridWorld(
                world=self.world,
                robot_id=self.robot_id,
                initial_pose=self.initial_pose,
            )
        )
        clock = NavigationClock()
        perception = WorldPerception(
            robot_id=self.robot_id,
            simulation=simulation,
            clock=clock,
            target_states=[False, True],
        )
        memory = InMemoryMissionMemory()
        monitoring = NavigationMonitoring()
        control = SafeRobotControl(
            robot_id=self.robot_id,
            initial_pose=self.initial_pose,
            simulation=simulation,
            clock=clock,
        )
        brain = DeterministicBrain(
            scenario_id="v1-reference",
            robot_id=self.robot_id,
            target_id=self.target_id,
            world=self.world,
            initial_pose=self.initial_pose,
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

    def advance_to_return_preparation(
        self,
        brain: DeterministicBrain,
    ) -> None:
        """Execute activation, outbound travel, and collection."""

        brain.update()
        brain.update()
        brain.update()

        self.advance_to_state(
            brain,
            BrainState.RETURN_PREPARATION,
        )

    def test_return_preparation_uses_exact_reversed_outbound_record(
        self,
    ) -> None:
        brain, simulation, memory, _, _ = self.build_brain()
        self.advance_to_return_preparation(brain)
        expected_positions = tuple(
            pose.position
            for pose in reversed(memory.outbound_poses)
        )
        simulation.applied_commands.clear()

        brain.update()

        status = brain.get_status()
        plan = status.active_plan

        self.assertIs(
            status.brain_state,
            BrainState.RETURN_NAVIGATION,
        )
        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertIs(plan.phase, PathPhase.RETURN)
        self.assertEqual(plan.version, 1)
        self.assertEqual(
            plan.positions,
            expected_positions,
        )
        self.assertEqual(
            plan.positions[0],
            simulation.current_pose.position,
        )
        self.assertEqual(
            plan.goal,
            self.world.base_position,
        )
        self.assertEqual(simulation.applied_commands, [])
        self.assertEqual(
            memory.events[-1].name,
            "return_path_prepared",
        )

    def test_return_commands_recalculate_heading_from_positions(
        self,
    ) -> None:
        brain, simulation, memory, _, _ = self.build_brain()
        self.advance_to_return_preparation(brain)
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
            simulation.current_pose,
            RobotPose(
                position=Position(x=3, y=3),
                heading=Heading.WEST,
            ),
        )
        self.assertEqual(
            memory.return_poses,
            (
                RobotPose(
                    position=Position(x=4, y=3),
                    heading=Heading.SOUTH,
                ),
                RobotPose(
                    position=Position(x=4, y=3),
                    heading=Heading.WEST,
                ),
                RobotPose(
                    position=Position(x=3, y=3),
                    heading=Heading.WEST,
                ),
            ),
        )
        self.assertEqual(
            tuple(
                event.name
                for event in memory.events[-3:]
            ),
            (
                "return_step_confirmed",
                "return_step_confirmed",
                "return_step_confirmed",
            ),
        )

    def test_confirmed_base_arrival_enters_completed_state(
        self,
    ) -> None:
        brain, simulation, memory, monitoring, _ = (
            self.build_brain()
        )
        self.advance_to_return_preparation(brain)
        brain.update()

        self.advance_to_state(
            brain,
            BrainState.MISSION_COMPLETED,
        )

        status = brain.get_status()

        self.assertEqual(
            status.robot_pose.position,
            self.world.base_position,
        )
        self.assertEqual(
            simulation.current_pose.position,
            self.world.base_position,
        )
        self.assertIsNone(status.active_plan)
        self.assertTrue(memory.return_poses)
        self.assertEqual(
            memory.return_poses[-1].position,
            self.world.base_position,
        )
        self.assertIsNone(memory.completed_mission)
        self.assertEqual(
            memory.events[-1].name,
            "base_arrival_confirmed",
        )
        self.assertIs(
            monitoring.events[-1],
            memory.events[-1],
        )
        self.assertIs(
            simulation.applied_commands[-1].command_type,
            CommandType.MOVE_FORWARD,
        )


if __name__ == "__main__":
    unittest.main()
