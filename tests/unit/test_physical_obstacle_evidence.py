"""Unit tests for validated physical obstacle evidence."""

import unittest

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
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.planning.a_star_planner import (
    AStarPlanner,
)


class PhysicalObstacleEvidenceTests(unittest.TestCase):
    """Verify safe obstacle promotion into planning."""

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
                    Position(x=1, y=1),
                }
            ),
        )

    @staticmethod
    def evidence(
        *,
        position: Position,
        source: ObstacleEvidenceSource,
        status: ObstacleEvidenceStatus,
        confidence: float = 1.0,
    ) -> ObstacleEvidence:
        return ObstacleEvidence(
            position=position,
            source=source,
            status=status,
            confidence=confidence,
        )

    def test_camera_may_only_propose_detection(
        self,
    ) -> None:
        item = self.evidence(
            position=Position(x=2, y=2),
            source=ObstacleEvidenceSource.CAMERA,
            status=ObstacleEvidenceStatus.DETECTED,
            confidence=0.82,
        )

        self.assertEqual(item.confidence, 0.82)

        with self.assertRaisesRegex(
            InvariantViolationError,
            "not authorized",
        ):
            self.evidence(
                position=Position(x=2, y=2),
                source=ObstacleEvidenceSource.CAMERA,
                status=ObstacleEvidenceStatus.CONFIRMED,
            )

    def test_operator_may_confirm_or_clear(
        self,
    ) -> None:
        for status in (
            ObstacleEvidenceStatus.CONFIRMED,
            ObstacleEvidenceStatus.CLEARED,
        ):
            with self.subTest(status=status):
                item = self.evidence(
                    position=Position(x=2, y=2),
                    source=ObstacleEvidenceSource.OPERATOR,
                    status=status,
                )
                self.assertIs(item.status, status)

    def test_local_sensor_may_confirm_only(
        self,
    ) -> None:
        confirmed = self.evidence(
            position=Position(x=2, y=2),
            source=ObstacleEvidenceSource.LOCAL_SENSOR,
            status=ObstacleEvidenceStatus.CONFIRMED,
        )
        self.assertIs(
            confirmed.status,
            ObstacleEvidenceStatus.CONFIRMED,
        )

        with self.assertRaises(
            InvariantViolationError
        ):
            self.evidence(
                position=Position(x=2, y=2),
                source=ObstacleEvidenceSource.LOCAL_SENSOR,
                status=ObstacleEvidenceStatus.CLEARED,
            )

    def test_confidence_is_validated(self) -> None:
        invalid_values = (
            -0.1,
            1.1,
            float("inf"),
            float("nan"),
            True,
        )

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.evidence(
                        position=Position(x=2, y=2),
                        source=ObstacleEvidenceSource.CAMERA,
                        status=ObstacleEvidenceStatus.DETECTED,
                        confidence=value,
                    )

    def test_positions_are_partitioned_by_status(
        self,
    ) -> None:
        physical_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(
                self.evidence(
                    position=Position(x=2, y=2),
                    source=ObstacleEvidenceSource.CAMERA,
                    status=ObstacleEvidenceStatus.DETECTED,
                    confidence=0.8,
                ),
                self.evidence(
                    position=Position(x=3, y=2),
                    source=ObstacleEvidenceSource.OPERATOR,
                    status=ObstacleEvidenceStatus.CONFIRMED,
                ),
                self.evidence(
                    position=Position(x=1, y=1),
                    source=ObstacleEvidenceSource.OPERATOR,
                    status=ObstacleEvidenceStatus.CLEARED,
                ),
            ),
        )

        self.assertEqual(
            physical_map.detected_positions,
            frozenset(
                {
                    Position(x=2, y=2),
                }
            ),
        )
        self.assertEqual(
            physical_map.confirmed_positions,
            frozenset(
                {
                    Position(x=3, y=2),
                }
            ),
        )
        self.assertEqual(
            physical_map.cleared_positions,
            frozenset(
                {
                    Position(x=1, y=1),
                }
            ),
        )

    def test_detected_cells_do_not_block_planning(
        self,
    ) -> None:
        physical_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(
                self.evidence(
                    position=Position(x=2, y=2),
                    source=ObstacleEvidenceSource.CAMERA,
                    status=ObstacleEvidenceStatus.DETECTED,
                    confidence=0.9,
                ),
            ),
        )

        self.assertNotIn(
            Position(x=2, y=2),
            physical_map.confirmed_world.obstacles,
        )

    def test_confirmed_cells_block_planning(
        self,
    ) -> None:
        physical_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(
                self.evidence(
                    position=Position(x=2, y=2),
                    source=ObstacleEvidenceSource.OPERATOR,
                    status=ObstacleEvidenceStatus.CONFIRMED,
                ),
            ),
        )

        self.assertIn(
            Position(x=2, y=2),
            physical_map.confirmed_world.obstacles,
        )

    def test_cleared_cells_remove_existing_obstacle(
        self,
    ) -> None:
        physical_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(
                self.evidence(
                    position=Position(x=1, y=1),
                    source=ObstacleEvidenceSource.OPERATOR,
                    status=ObstacleEvidenceStatus.CLEARED,
                ),
            ),
        )

        self.assertNotIn(
            Position(x=1, y=1),
            physical_map.confirmed_world.obstacles,
        )

    def test_duplicate_positions_are_rejected(
        self,
    ) -> None:
        position = Position(x=2, y=2)

        with self.assertRaisesRegex(
            InvariantViolationError,
            "positions must be unique",
        ):
            PhysicalObstacleMap(
                base_world=self.world,
                evidence=(
                    self.evidence(
                        position=position,
                        source=ObstacleEvidenceSource.CAMERA,
                        status=ObstacleEvidenceStatus.DETECTED,
                    ),
                    self.evidence(
                        position=position,
                        source=ObstacleEvidenceSource.OPERATOR,
                        status=ObstacleEvidenceStatus.CONFIRMED,
                    ),
                ),
            )

    def test_out_of_bounds_position_is_rejected(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            InvariantViolationError,
            "within the grid",
        ):
            PhysicalObstacleMap(
                base_world=self.world,
                evidence=(
                    self.evidence(
                        position=Position(x=7, y=7),
                        source=ObstacleEvidenceSource.CAMERA,
                        status=ObstacleEvidenceStatus.DETECTED,
                    ),
                ),
            )

    def test_base_and_target_positions_are_rejected(
        self,
    ) -> None:
        for position in (
            self.world.base_position,
            self.world.target_position,
        ):
            with self.subTest(position=position):
                with self.assertRaisesRegex(
                    InvariantViolationError,
                    "base and target",
                ):
                    PhysicalObstacleMap(
                        base_world=self.world,
                        evidence=(
                            self.evidence(
                                position=position,
                                source=ObstacleEvidenceSource.CAMERA,
                                status=ObstacleEvidenceStatus.DETECTED,
                            ),
                        ),
                    )

    def test_planner_uses_only_confirmed_obstacles(
        self,
    ) -> None:
        candidate = Position(x=1, y=0)
        goal = Position(x=2, y=0)
        start_pose = RobotPose(
            position=self.world.base_position,
            heading=Heading.EAST,
        )
        planner = AStarPlanner()

        detected_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(
                self.evidence(
                    position=candidate,
                    source=ObstacleEvidenceSource.CAMERA,
                    status=ObstacleEvidenceStatus.DETECTED,
                    confidence=0.9,
                ),
            ),
        )
        detected_plan = planner.create_plan(
            mission_id="mission-detected",
            robot_id="robot-1",
            start_pose=start_pose,
            authorized_goals=(goal,),
            world=detected_map.confirmed_world,
            phase=PathPhase.OUTBOUND,
            version=1,
        )

        self.assertIn(
            candidate,
            detected_plan.positions,
        )

        confirmed_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(
                self.evidence(
                    position=candidate,
                    source=ObstacleEvidenceSource.OPERATOR,
                    status=ObstacleEvidenceStatus.CONFIRMED,
                    confidence=1.0,
                ),
            ),
        )
        confirmed_plan = planner.create_plan(
            mission_id="mission-confirmed",
            robot_id="robot-1",
            start_pose=start_pose,
            authorized_goals=(goal,),
            world=confirmed_map.confirmed_world,
            phase=PathPhase.DETOUR,
            version=2,
        )

        self.assertNotIn(
            candidate,
            confirmed_plan.positions,
        )
        self.assertEqual(
            confirmed_plan.goal,
            goal,
        )

    def test_collection_must_be_immutable(self) -> None:
        with self.assertRaises(
            DomainValidationError
        ):
            PhysicalObstacleMap(
                base_world=self.world,
                evidence=[],  # type: ignore[arg-type]
            )


if __name__ == "__main__":
    unittest.main()
