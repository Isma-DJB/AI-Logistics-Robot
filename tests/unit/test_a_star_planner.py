"""Unit tests for deterministic A* path planning."""

import unittest

from ai_logistics_robot.domain.enums import Heading, PathPhase
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.planning.a_star_planner import AStarPlanner
from ai_logistics_robot.ports.planning_port import PlanningPort


class AStarPlannerNominalTests(unittest.TestCase):
    """Verify nominal deterministic shortest-path planning."""

    def setUp(self) -> None:
        self.world = GridMap(
            width=6,
            height=6,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=0, y=0),
            target_position=Position(x=5, y=5),
            obstacles=frozenset(
                {
                    Position(x=1, y=0),
                    Position(x=1, y=1),
                    Position(x=1, y=2),
                }
            ),
        )
        self.start_pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.planner = AStarPlanner()

    def create_plan(
        self,
        goals: tuple[Position, ...],
        *,
        world: GridMap | None = None,
        start_pose: RobotPose | None = None,
        phase: PathPhase = PathPhase.OUTBOUND,
        version: int = 1,
    ) -> PathPlan:
        """Create a plan with stable test identities."""

        selected_world = self.world if world is None else world
        selected_pose = (
            self.start_pose
            if start_pose is None
            else start_pose
        )

        return self.planner.create_plan(
            mission_id="mission_1",
            robot_id="robot_1",
            start_pose=selected_pose,
            authorized_goals=goals,
            world=selected_world,
            phase=phase,
            version=version,
        )

    def test_planner_satisfies_runtime_protocol(self) -> None:
        self.assertIsInstance(self.planner, PlanningPort)

    def test_shortest_path_avoids_obstacles(self) -> None:
        goal = Position(x=2, y=0)

        plan = self.create_plan((goal,))

        self.assertEqual(
            plan.positions,
            (
                Position(x=0, y=0),
                Position(x=0, y=1),
                Position(x=0, y=2),
                Position(x=0, y=3),
                Position(x=1, y=3),
                Position(x=2, y=3),
                Position(x=2, y=2),
                Position(x=2, y=1),
                Position(x=2, y=0),
            ),
        )
        self.assertEqual(plan.goal, goal)
        self.assertEqual(plan.mission_id, "mission_1")
        self.assertEqual(plan.robot_id, "robot_1")
        self.assertIs(plan.phase, PathPhase.OUTBOUND)
        self.assertEqual(plan.version, 1)

        for position in plan.positions:
            self.assertTrue(
                self.world.is_traversable(position)
            )

        for current, following in zip(
            plan.positions,
            plan.positions[1:],
            strict=False,
        ):
            distance = (
                abs(current.x - following.x)
                + abs(current.y - following.y)
            )
            self.assertEqual(distance, 1)

    def test_shortest_reachable_goal_is_selected(self) -> None:
        goals = (
            Position(x=4, y=0),
            Position(x=0, y=2),
        )

        plan = self.create_plan(goals)

        self.assertEqual(plan.goal, goals[1])
        self.assertEqual(
            plan.positions,
            (
                Position(x=0, y=0),
                Position(x=0, y=1),
                Position(x=0, y=2),
            ),
        )

    def test_equal_cost_goals_follow_supplied_order(self) -> None:
        open_world = GridMap(
            width=6,
            height=6,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=0, y=0),
            target_position=Position(x=5, y=5),
        )
        centered_pose = RobotPose(
            position=Position(x=2, y=2),
            heading=Heading.SOUTH,
        )
        west_goal = Position(x=0, y=2)
        east_goal = Position(x=4, y=2)

        west_first = self.create_plan(
            (west_goal, east_goal),
            world=open_world,
            start_pose=centered_pose,
        )
        east_first = self.create_plan(
            (east_goal, west_goal),
            world=open_world,
            start_pose=centered_pose,
        )

        self.assertEqual(west_first.goal, west_goal)
        self.assertEqual(east_first.goal, east_goal)

    def test_repeated_calls_are_identical(self) -> None:
        goal = Position(x=2, y=0)

        plans = tuple(
            self.create_plan(
                (goal,),
                phase=PathPhase.DETOUR,
                version=4,
            )
            for _ in range(5)
        )

        self.assertTrue(
            all(plan == plans[0] for plan in plans)
        )
        self.assertIs(plans[0].phase, PathPhase.DETOUR)
        self.assertEqual(plans[0].version, 4)


if __name__ == "__main__":
    unittest.main()
