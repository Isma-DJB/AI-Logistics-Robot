"""Unit tests for immutable assisted-mission evidence."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.control.mission_evidence import (
    MissionEvidence,
    MissionEvidenceStatus,
)
from ai_logistics_robot.control.physical_obstacle_evidence import (
    ObstacleEvidence,
    ObstacleEvidenceSource,
    ObstacleEvidenceStatus,
    PhysicalObstacleMap,
)
from ai_logistics_robot.domain.enums import (
    Heading,
    PathPhase,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import (
    Position,
    RobotPose,
)
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.perception import (
    PerceptionSnapshot,
)
from ai_logistics_robot.domain.world import GridMap


class MissionEvidenceTests(unittest.TestCase):
    """Verify evidence validation without hardware dependencies."""

    def setUp(self) -> None:
        self.world = GridMap(
            width=4,
            height=4,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=0, y=0),
            target_position=Position(x=3, y=3),
            obstacles=frozenset(
                {
                    Position(x=2, y=2),
                }
            ),
        )
        self.perception = PerceptionSnapshot(
            robot_id="robot-1",
            captured_at=datetime(
                2026,
                10,
                26,
                10,
                0,
                tzinfo=UTC,
            ),
            robot_pose=RobotPose(
                position=Position(x=0, y=0),
                heading=Heading.NORTH,
            ),
            observations=(),
            target_active=True,
            hazard_detected=False,
        )
        self.plan = PathPlan(
            mission_id="mission-1",
            robot_id="robot-1",
            phase=PathPhase.OUTBOUND,
            version=1,
            positions=(
                Position(x=0, y=0),
                Position(x=0, y=1),
                Position(x=1, y=1),
            ),
            goal=Position(x=1, y=1),
        )

    def build_evidence(
        self,
        **changes: object,
    ) -> MissionEvidence:
        values: dict[str, object] = {
            "scenario_id": "batch-e-reference",
            "step_number": 1,
            "stage": "outbound planning",
            "status": MissionEvidenceStatus.PLANNED,
            "world": self.world,
            "perception": self.perception,
            "plan": self.plan,
            "confirmed_positions": (
                Position(x=0, y=0),
            ),
            "next_position": Position(x=0, y=1),
        }
        values.update(changes)
        return MissionEvidence(**values)  # type: ignore[arg-type]

    def test_consistent_evidence_is_immutable(self) -> None:
        evidence = self.build_evidence()

        self.assertEqual(
            evidence.status,
            MissionEvidenceStatus.PLANNED,
        )
        self.assertEqual(
            evidence.next_position,
            Position(x=0, y=1),
        )

        with self.assertRaises(AttributeError):
            evidence.stage = "changed"  # type: ignore[misc]

    def test_initial_evidence_may_have_no_plan(self) -> None:
        evidence = self.build_evidence(
            plan=None,
            confirmed_positions=(),
            next_position=None,
        )

        self.assertIsNone(evidence.plan)
        self.assertIsNone(evidence.next_position)

    def test_identifiers_and_step_are_validated(self) -> None:
        invalid_changes = (
            {"scenario_id": ""},
            {"stage": "  "},
            {"step_number": 0},
            {"step_number": True},
        )

        for changes in invalid_changes:
            with self.subTest(changes=changes):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.build_evidence(**changes)

    def test_matching_obstacle_map_is_retained(
        self,
    ) -> None:
        obstacle_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(
                ObstacleEvidence(
                    position=Position(x=3, y=2),
                    source=ObstacleEvidenceSource.CAMERA,
                    status=ObstacleEvidenceStatus.DETECTED,
                    confidence=0.8,
                ),
            ),
        )

        evidence = self.build_evidence(
            obstacle_map=obstacle_map,
        )

        self.assertIs(
            evidence.obstacle_map,
            obstacle_map,
        )
        self.assertEqual(
            obstacle_map.confirmed_world,
            self.world,
        )

    def test_obstacle_map_type_is_validated(
        self,
    ) -> None:
        with self.assertRaises(
            DomainValidationError
        ):
            self.build_evidence(
                obstacle_map="invalid",
            )

    def test_obstacle_map_must_match_world(
        self,
    ) -> None:
        obstacle_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(
                ObstacleEvidence(
                    position=Position(x=3, y=2),
                    source=ObstacleEvidenceSource.OPERATOR,
                    status=ObstacleEvidenceStatus.CONFIRMED,
                    confidence=1.0,
                ),
            ),
        )

        with self.assertRaisesRegex(
            InvariantViolationError,
            "must equal the confirmed",
        ):
            self.build_evidence(
                obstacle_map=obstacle_map,
            )

    def test_status_type_is_validated(self) -> None:
        with self.assertRaises(
            DomainValidationError
        ):
            self.build_evidence(
                status="PLANNED",
            )

    def test_robot_must_occupy_traversable_cell(
        self,
    ) -> None:
        invalid_perception = PerceptionSnapshot(
            robot_id="robot-1",
            captured_at=self.perception.captured_at,
            robot_pose=RobotPose(
                position=Position(x=2, y=2),
                heading=Heading.NORTH,
            ),
            observations=(),
            target_active=True,
            hazard_detected=False,
        )

        with self.assertRaisesRegex(
            InvariantViolationError,
            "perceived robot position must be traversable",
        ):
            self.build_evidence(
                perception=invalid_perception,
            )

    def test_plan_and_perception_robot_must_match(
        self,
    ) -> None:
        mismatched_plan = PathPlan(
            mission_id="mission-1",
            robot_id="robot-2",
            phase=PathPhase.OUTBOUND,
            version=1,
            positions=self.plan.positions,
            goal=self.plan.goal,
        )

        with self.assertRaisesRegex(
            InvariantViolationError,
            "identifiers must match",
        ):
            self.build_evidence(
                plan=mismatched_plan,
            )

    def test_every_planned_position_must_be_traversable(
        self,
    ) -> None:
        blocked_plan = PathPlan(
            mission_id="mission-1",
            robot_id="robot-1",
            phase=PathPhase.DETOUR,
            version=2,
            positions=(
                Position(x=0, y=0),
                Position(x=1, y=0),
                Position(x=2, y=0),
                Position(x=2, y=1),
                Position(x=2, y=2),
            ),
            goal=Position(x=2, y=2),
        )

        with self.assertRaisesRegex(
            InvariantViolationError,
            "planned position must be traversable",
        ):
            self.build_evidence(
                plan=blocked_plan,
                next_position=None,
            )

    def test_next_position_must_belong_to_plan(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            InvariantViolationError,
            "must belong to the path plan",
        ):
            self.build_evidence(
                next_position=Position(x=3, y=0),
            )

    def test_next_position_requires_plan(self) -> None:
        with self.assertRaisesRegex(
            InvariantViolationError,
            "requires a path plan",
        ):
            self.build_evidence(
                plan=None,
                next_position=Position(x=0, y=1),
            )

    def test_confirmed_positions_must_be_immutable(
        self,
    ) -> None:
        with self.assertRaises(
            DomainValidationError
        ):
            self.build_evidence(
                confirmed_positions=[
                    Position(x=0, y=0),
                ],
            )


if __name__ == "__main__":
    unittest.main()
