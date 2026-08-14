"""Integration tests for reference Planning and Memory behavior."""

import unittest
from pathlib import Path

from ai_logistics_robot.app.settings import load_settings
from ai_logistics_robot.domain.enums import PathPhase
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.domain.paths import PathPlan, PathRecord
from ai_logistics_robot.memory import InMemoryMissionMemory
from ai_logistics_robot.planning import AStarPlanner
from ai_logistics_robot.ports.memory_port import MemoryPort
from ai_logistics_robot.ports.planning_port import PlanningPort


class ReferencePlanningMemoryTests(unittest.TestCase):
    """Verify composition using the approved reference settings."""

    def setUp(self) -> None:
        project_root = Path(__file__).resolve().parents[3]
        self.settings = load_settings(
            project_root / "configs" / "simulation.yaml"
        )
        self.mission = Mission(
            mission_id="mission_reference",
            robot_id=self.settings.robot.robot_id,
            target_id=self.settings.target.target_id,
            target_position=(
                self.settings.target.target_position
            ),
            base_position=(
                self.settings.grid_map.base_position
            ),
        )
        self.planner = AStarPlanner()
        self.memory = InMemoryMissionMemory()

    def create_outbound_plan(self) -> PathPlan:
        """Create the deterministic reference outbound plan."""

        return self.planner.create_plan(
            mission_id=self.mission.mission_id,
            robot_id=self.mission.robot_id,
            start_pose=self.settings.robot.initial_pose,
            authorized_goals=(
                self.settings
                .grid_map
                .authorized_arrival_positions
            ),
            world=self.settings.grid_map,
            phase=PathPhase.OUTBOUND,
            version=1,
        )

    def confirmed_poses(
        self,
        plan: PathPlan,
    ) -> tuple[RobotPose, ...]:
        """Represent supplied plan positions as confirmed poses."""

        return tuple(
            RobotPose(
                position=position,
                heading=(
                    self.settings.robot.initial_pose.heading
                ),
            )
            for position in plan.positions
        )

    def record_outbound_plan(
        self,
        plan: PathPlan,
    ) -> PathRecord:
        """Record one plan as confirmed outbound history."""

        self.memory.start(self.mission)

        for pose in self.confirmed_poses(plan):
            self.memory.record_pose(
                PathPhase.OUTBOUND,
                pose,
            )

        return self.memory.build_return_path()

    def test_components_are_public_and_satisfy_ports(
        self,
    ) -> None:
        self.assertIsInstance(self.planner, PlanningPort)
        self.assertIsInstance(self.memory, MemoryPort)

    def test_reference_plan_selects_safe_arrival_position(
        self,
    ) -> None:
        plan = self.create_outbound_plan()

        self.assertEqual(
            plan.positions[0],
            self.settings.robot.initial_pose.position,
        )
        self.assertEqual(
            plan.goal,
            Position(x=8, y=6),
        )
        self.assertIn(
            plan.goal,
            self.settings
            .grid_map
            .authorized_arrival_positions,
        )

        for position in plan.positions:
            self.assertTrue(
                self.settings
                .grid_map
                .is_traversable(position)
            )

    def test_confirmed_history_builds_exact_reverse_path(
        self,
    ) -> None:
        plan = self.create_outbound_plan()

        return_path = self.record_outbound_plan(plan)

        self.assertEqual(
            return_path.confirmed_positions,
            tuple(reversed(plan.positions)),
        )
        self.assertEqual(
            return_path.mission_id,
            self.mission.mission_id,
        )
        self.assertEqual(
            return_path.robot_id,
            self.mission.robot_id,
        )
        self.assertIs(
            return_path.phase,
            PathPhase.RETURN,
        )

    def test_reset_replays_identical_plan_and_record(
        self,
    ) -> None:
        first_plan = self.create_outbound_plan()
        first_return = self.record_outbound_plan(first_plan)

        self.memory.reset()

        second_plan = self.create_outbound_plan()
        second_return = self.record_outbound_plan(
            second_plan
        )

        self.assertEqual(first_plan, second_plan)
        self.assertEqual(first_return, second_return)


if __name__ == "__main__":
    unittest.main()
