"""Unit tests for A* validation and unreachable paths."""

import unittest

from ai_logistics_robot.domain.enums import Heading, PathPhase
from ai_logistics_robot.domain.errors import (
    DomainError,
    DomainValidationError,
    NoPathError,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.planning.a_star_planner import AStarPlanner


class AStarPlannerRejectionTests(unittest.TestCase):
    """Verify planning validation and no-path behavior."""

    def setUp(self) -> None:
        self.world = GridMap(
            width=5,
            height=5,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=0, y=0),
            target_position=Position(x=4, y=4),
            obstacles=frozenset(
                {
                    Position(x=1, y=0),
                }
            ),
        )
        self.start_pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.goal = Position(x=0, y=2)
        self.planner = AStarPlanner()

    def create_plan(
        self,
        **overrides: object,
    ) -> PathPlan:
        """Create a planning request with optional invalid values."""

        data: dict[str, object] = {
            "mission_id": "mission_1",
            "robot_id": "robot_1",
            "start_pose": self.start_pose,
            "authorized_goals": (self.goal,),
            "world": self.world,
            "phase": PathPhase.OUTBOUND,
            "version": 1,
        }
        data.update(overrides)

        return self.planner.create_plan(
            **data,  # type: ignore[arg-type]
        )

    def isolated_world(self) -> GridMap:
        """Build a world containing one isolated traversable cell."""

        return GridMap(
            width=5,
            height=5,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=0, y=0),
            target_position=Position(x=4, y=4),
            obstacles=frozenset(
                {
                    Position(x=2, y=3),
                    Position(x=3, y=2),
                    Position(x=2, y=1),
                    Position(x=1, y=2),
                }
            ),
        )

    def test_start_already_at_goal_returns_single_position(
        self,
    ) -> None:
        plan = self.create_plan(
            authorized_goals=(self.start_pose.position,),
        )

        self.assertEqual(
            plan.positions,
            (self.start_pose.position,),
        )
        self.assertEqual(
            plan.goal,
            self.start_pose.position,
        )

    def test_shifted_origin_uses_configured_grid_bounds(
        self,
    ) -> None:
        shifted_world = GridMap(
            width=4,
            height=4,
            cell_size_cm=20,
            origin=Position(x=5, y=5),
            base_position=Position(x=5, y=5),
            target_position=Position(x=8, y=8),
            obstacles=frozenset(
                {
                    Position(x=6, y=5),
                }
            ),
        )
        shifted_pose = RobotPose(
            position=Position(x=5, y=5),
            heading=Heading.EAST,
        )
        goal = Position(x=7, y=5)

        plan = self.create_plan(
            world=shifted_world,
            start_pose=shifted_pose,
            authorized_goals=(goal,),
            phase=PathPhase.RETURN,
            version=2,
        )

        self.assertEqual(
            plan.positions,
            (
                Position(x=5, y=5),
                Position(x=5, y=6),
                Position(x=6, y=6),
                Position(x=7, y=6),
                Position(x=7, y=5),
            ),
        )
        self.assertIs(plan.phase, PathPhase.RETURN)
        self.assertEqual(plan.version, 2)

    def test_no_reachable_goal_raises_dedicated_error(
        self,
    ) -> None:
        isolated_world = self.isolated_world()
        isolated_goal = Position(x=2, y=2)

        self.assertTrue(
            issubclass(NoPathError, DomainError)
        )

        with self.assertRaisesRegex(
            NoPathError,
            "no authorized goal is reachable",
        ):
            self.create_plan(
                world=isolated_world,
                authorized_goals=(isolated_goal,),
            )

        valid_plan = self.create_plan()

        self.assertEqual(valid_plan.goal, self.goal)

    def test_unreachable_goal_is_skipped_when_another_is_reachable(
        self,
    ) -> None:
        isolated_world = self.isolated_world()
        isolated_goal = Position(x=2, y=2)
        reachable_goal = Position(x=0, y=1)

        plan = self.create_plan(
            world=isolated_world,
            authorized_goals=(
                isolated_goal,
                reachable_goal,
            ),
        )

        self.assertEqual(plan.goal, reachable_goal)
        self.assertEqual(
            plan.positions,
            (
                Position(x=0, y=0),
                Position(x=0, y=1),
            ),
        )

    def test_identifiers_must_be_non_empty_strings(self) -> None:
        for field in ("mission_id", "robot_id"):
            for value in ("", "   ", None, 1):
                with self.subTest(
                    field=field,
                    value=value,
                ):
                    with self.assertRaises(
                        DomainValidationError
                    ):
                        self.create_plan(
                            **{field: value},
                        )

    def test_request_types_and_version_are_validated(
        self,
    ) -> None:
        cases: tuple[tuple[str, object], ...] = (
            ("start_pose", Position(x=0, y=0)),
            ("world", "world"),
            ("phase", "OUTBOUND"),
            ("version", 0),
            ("version", -1),
            ("version", 1.5),
            ("version", True),
        )

        for field, value in cases:
            with self.subTest(field=field, value=value):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.create_plan(
                        **{field: value},
                    )

    def test_authorized_goals_must_be_valid_collection(
        self,
    ) -> None:
        invalid_values: tuple[object, ...] = (
            [self.goal],
            (),
            (self.goal, self.goal),
            (self.goal, "invalid"),
        )

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.create_plan(
                        authorized_goals=value,
                    )

    def test_authorized_goals_must_be_traversable(
        self,
    ) -> None:
        invalid_goals = (
            Position(x=1, y=0),
            self.world.target_position,
            Position(x=5, y=0),
        )

        for goal in invalid_goals:
            with self.subTest(goal=goal):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.create_plan(
                        authorized_goals=(goal,),
                    )

    def test_start_pose_must_be_traversable(self) -> None:
        invalid_positions = (
            Position(x=1, y=0),
            self.world.target_position,
            Position(x=5, y=0),
        )

        for position in invalid_positions:
            with self.subTest(position=position):
                invalid_pose = RobotPose(
                    position=position,
                    heading=Heading.NORTH,
                )

                with self.assertRaises(
                    DomainValidationError
                ):
                    self.create_plan(
                        start_pose=invalid_pose,
                    )


if __name__ == "__main__":
    unittest.main()
