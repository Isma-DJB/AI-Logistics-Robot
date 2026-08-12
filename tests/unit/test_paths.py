"""Unit tests for path plans and confirmed path records."""

import unittest
from dataclasses import FrozenInstanceError

from ai_logistics_robot.domain.enums import Heading, PathPhase
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan, PathRecord


class PathPlanTests(unittest.TestCase):
    """Verify versioned plan validation and immutability."""

    def valid_plan(self) -> PathPlan:
        """Build a valid outbound plan for test variations."""

        positions = (
            Position(x=1, y=1),
            Position(x=1, y=2),
            Position(x=1, y=3),
        )
        return PathPlan(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.OUTBOUND,
            version=1,
            positions=positions,
            goal=positions[-1],
        )

    def test_path_plan_accepts_valid_versioned_path(self) -> None:
        plan = self.valid_plan()

        self.assertEqual(plan.version, 1)
        self.assertEqual(plan.positions[-1], plan.goal)
        self.assertIs(plan.phase, PathPhase.OUTBOUND)

    def test_path_plan_rejects_empty_identifier(self) -> None:
        plan = self.valid_plan()

        with self.assertRaises(DomainValidationError):
            PathPlan(
                mission_id=" ",
                robot_id=plan.robot_id,
                phase=plan.phase,
                version=plan.version,
                positions=plan.positions,
                goal=plan.goal,
            )

    def test_path_plan_requires_positive_integer_version(self) -> None:
        plan = self.valid_plan()

        for value in (0, -1, 1.5, True):
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    PathPlan(  # type: ignore[arg-type]
                        mission_id=plan.mission_id,
                        robot_id=plan.robot_id,
                        phase=plan.phase,
                        version=value,
                        positions=plan.positions,
                        goal=plan.goal,
                    )

    def test_path_plan_requires_immutable_positions(self) -> None:
        goal = Position(x=1, y=2)

        with self.assertRaises(DomainValidationError):
            PathPlan(  # type: ignore[arg-type]
                mission_id="mission_1",
                robot_id="robot_1",
                phase=PathPhase.OUTBOUND,
                version=1,
                positions=[Position(x=1, y=1), goal],
                goal=goal,
            )

    def test_path_plan_rejects_empty_positions(self) -> None:
        with self.assertRaises(DomainValidationError):
            PathPlan(
                mission_id="mission_1",
                robot_id="robot_1",
                phase=PathPhase.OUTBOUND,
                version=1,
                positions=(),
                goal=Position(x=1, y=1),
            )

    def test_path_plan_requires_goal_as_final_position(self) -> None:
        with self.assertRaises(InvariantViolationError):
            PathPlan(
                mission_id="mission_1",
                robot_id="robot_1",
                phase=PathPhase.OUTBOUND,
                version=1,
                positions=(
                    Position(x=1, y=1),
                    Position(x=1, y=2),
                ),
                goal=Position(x=2, y=2),
            )

    def test_path_plan_is_immutable(self) -> None:
        plan = self.valid_plan()

        with self.assertRaises(FrozenInstanceError):
            plan.version = 2  # type: ignore[misc]


class PathRecordTests(unittest.TestCase):
    """Verify confirmed history validation and immutability."""

    def setUp(self) -> None:
        self.pose_1 = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.pose_2 = RobotPose(
            position=Position(x=1, y=2),
            heading=Heading.NORTH,
        )

    def test_path_record_exposes_confirmed_positions(self) -> None:
        record = PathRecord(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.OUTBOUND,
            confirmed_poses=(self.pose_1, self.pose_2),
        )

        self.assertEqual(
            record.confirmed_positions,
            (self.pose_1.position, self.pose_2.position),
        )

    def test_empty_path_record_is_valid_at_mission_start(self) -> None:
        record = PathRecord(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.OUTBOUND,
            confirmed_poses=(),
        )

        self.assertEqual(record.confirmed_positions, ())

    def test_path_record_rejects_detour_phase(self) -> None:
        with self.assertRaises(DomainValidationError):
            PathRecord(
                mission_id="mission_1",
                robot_id="robot_1",
                phase=PathPhase.DETOUR,
                confirmed_poses=(self.pose_1,),
            )

    def test_path_record_requires_immutable_pose_history(self) -> None:
        with self.assertRaises(DomainValidationError):
            PathRecord(  # type: ignore[arg-type]
                mission_id="mission_1",
                robot_id="robot_1",
                phase=PathPhase.RETURN,
                confirmed_poses=[self.pose_1],
            )

    def test_path_record_rejects_non_pose_entries(self) -> None:
        with self.assertRaises(DomainValidationError):
            PathRecord(  # type: ignore[arg-type]
                mission_id="mission_1",
                robot_id="robot_1",
                phase=PathPhase.RETURN,
                confirmed_poses=(self.pose_1, Position(x=1, y=2)),
            )

    def test_path_record_is_immutable(self) -> None:
        record = PathRecord(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.RETURN,
            confirmed_poses=(self.pose_1,),
        )

        with self.assertRaises(FrozenInstanceError):
            record.phase = PathPhase.OUTBOUND  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()