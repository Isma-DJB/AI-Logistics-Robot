"""Unit tests for the assisted obstacle-scene diagnostic."""

import argparse
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from ai_logistics_robot.control.mission_evidence import (
    MissionEvidenceStatus,
)
from ai_logistics_robot.control.physical_obstacle_evidence import (
    ObstacleEvidence,
    ObstacleEvidenceSource,
    ObstacleEvidenceStatus,
)
from ai_logistics_robot.domain.enums import PathPhase
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
)
from ai_logistics_robot.domain.geometry import Position
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.world import GridMap
from tools.check_assisted_obstacle_scene import (
    AssistedObstacleSceneError,
    _authorized_arrival_positions,
    _authorized_goal_positions,
    _build_observations,
    _evidence_stage,
    _evidence_status_for_plan,
    _format_positions,
    _is_inside_world,
    _load_frame,
    _parse_position,
    build_parser,
    main,
)


def _world(
    *,
    obstacles: frozenset[Position] = frozenset(),
    origin: Position | None = None,
) -> GridMap:
    """Build one small physical planning world."""

    normalized_origin = (
        Position(x=0, y=0)
        if origin is None
        else origin
    )

    return GridMap(
        width=5,
        height=5,
        cell_size_cm=20.0,
        origin=normalized_origin,
        base_position=normalized_origin,
        target_position=Position(
            x=normalized_origin.x + 2,
            y=normalized_origin.y + 2,
        ),
        obstacles=obstacles,
    )


class AssistedObstacleSceneToolTests(unittest.TestCase):
    """Verify guarded assisted-scene tool helpers."""

    def test_position_parser_accepts_trimmed_coordinates(
        self,
    ) -> None:
        self.assertEqual(
            _parse_position(" 3, 4 "),
            Position(x=3, y=4),
        )

    def test_position_parser_rejects_invalid_shapes(
        self,
    ) -> None:
        invalid_values = (
            "",
            "1",
            "1,2,3",
            "x,2",
            "1,y",
        )

        for invalid_value in invalid_values:
            with self.subTest(
                invalid_value=invalid_value,
            ):
                with self.assertRaises(
                    argparse.ArgumentTypeError
                ):
                    _parse_position(invalid_value)

    def test_parser_collects_repeated_operator_decisions(
        self,
    ) -> None:
        arguments = build_parser().parse_args(
            [
                "--reference",
                "reference.jpg",
                "--current",
                "current.jpg",
                "--output",
                "output.jpg",
                "--confirm-obstacle",
                "1,2",
                "--clear-proposal",
                "1,3",
                "--clear-proposal",
                "3,4",
                "--existing-obstacle",
                "4,1",
                "--maximum-changed-cell-fraction",
                "0.3",
                "--target-active",
            ]
        )

        self.assertEqual(
            arguments.reference,
            Path("reference.jpg"),
        )
        self.assertEqual(
            arguments.confirm_obstacle,
            [
                Position(x=1, y=2),
            ],
        )
        self.assertEqual(
            arguments.clear_proposal,
            [
                Position(x=1, y=3),
                Position(x=3, y=4),
            ],
        )
        self.assertEqual(
            arguments.existing_obstacle,
            [
                Position(x=4, y=1),
            ],
        )
        self.assertEqual(
            arguments.maximum_changed_cell_fraction,
            0.3,
        )
        self.assertTrue(arguments.target_active)

    def test_arrivals_are_cardinal_and_traversable(
        self,
    ) -> None:
        world = _world(
            obstacles=frozenset(
                {
                    Position(x=2, y=1),
                    Position(x=1, y=2),
                    Position(x=3, y=2),
                }
            )
        )

        self.assertEqual(
            _authorized_arrival_positions(world),
            (
                Position(x=2, y=3),
            ),
        )

    def test_missing_target_arrival_is_rejected(
        self,
    ) -> None:
        world = _world(
            obstacles=frozenset(
                {
                    Position(x=2, y=1),
                    Position(x=1, y=2),
                    Position(x=3, y=2),
                    Position(x=2, y=3),
                }
            )
        )

        with self.assertRaisesRegex(
            AssistedObstacleSceneError,
            "no traversable cardinal arrival",
        ):
            _authorized_arrival_positions(world)

    def test_world_bounds_support_nonzero_origin(
        self,
    ) -> None:
        world = _world(
            origin=Position(x=10, y=20),
        )

        inside = (
            Position(x=10, y=20),
            Position(x=14, y=24),
        )
        outside = (
            Position(x=9, y=20),
            Position(x=10, y=19),
            Position(x=15, y=24),
            Position(x=14, y=25),
        )

        for position in inside:
            with self.subTest(position=position):
                self.assertTrue(
                    _is_inside_world(
                        position,
                        world=world,
                    )
                )

        for position in outside:
            with self.subTest(position=position):
                self.assertFalse(
                    _is_inside_world(
                        position,
                        world=world,
                    )
                )

    def test_reviewed_evidence_becomes_observations(
        self,
    ) -> None:
        evidence = (
            ObstacleEvidence(
                position=Position(x=1, y=2),
                source=ObstacleEvidenceSource.OPERATOR,
                status=ObstacleEvidenceStatus.CONFIRMED,
                confidence=0.421,
            ),
            ObstacleEvidence(
                position=Position(x=1, y=3),
                source=ObstacleEvidenceSource.OPERATOR,
                status=ObstacleEvidenceStatus.CLEARED,
                confidence=0.278,
            ),
        )

        observations = _build_observations(evidence)

        self.assertEqual(
            tuple(
                item.kind
                for item in observations
            ),
            (
                "physical_obstacle_confirmed",
                "camera_change_cleared",
            ),
        )
        self.assertEqual(
            tuple(
                item.position
                for item in observations
            ),
            (
                Position(x=1, y=2),
                Position(x=1, y=3),
            ),
        )

    def test_position_formatting_is_compact(self) -> None:
        self.assertEqual(
            _format_positions(()),
            "none",
        )
        self.assertEqual(
            _format_positions(
                (
                    Position(x=3, y=3),
                    Position(x=3, y=4),
                )
            ),
            "(3,3) -> (3,4)",
        )

    def test_frame_loader_rejects_missing_image(
        self,
    ) -> None:
        with patch(
            "tools.check_assisted_obstacle_scene.cv2.imread",
            return_value=None,
        ):
            with self.assertRaisesRegex(
                AssistedObstacleSceneError,
                "unable to decode reference image",
            ):
                _load_frame(
                    Path("missing.jpg"),
                    label="reference",
                )

    def test_frame_loader_returns_decoded_array(
        self,
    ) -> None:
        expected = np.zeros(
            (700, 700, 3),
            dtype=np.uint8,
        )

        with patch(
            "tools.check_assisted_obstacle_scene.cv2.imread",
            return_value=expected,
        ):
            actual = _load_frame(
                Path("frame.jpg"),
                label="current",
            )

        self.assertIs(actual, expected)

    def test_main_normalizes_diagnostic_failure(
        self,
    ) -> None:
        arguments = [
            "--reference",
            "reference.jpg",
            "--current",
            "current.jpg",
            "--output",
            "output.jpg",
        ]

        with (
            patch(
                "tools.check_assisted_obstacle_scene."
                "_run_diagnostic",
                side_effect=AssistedObstacleSceneError(
                    "review incomplete"
                ),
            ),
            patch("builtins.print") as print_mock,
        ):
            exit_code = main(arguments)

        self.assertEqual(exit_code, 2)
        print_mock.assert_called_once_with(
            "Assisted obstacle scene diagnostic failed: "
            "review incomplete"
        )

    def test_invalid_world_is_still_domain_validated(
        self,
    ) -> None:
        with self.assertRaises(DomainValidationError):
            GridMap(
                width=0,
                height=5,
                cell_size_cm=20.0,
                origin=Position(x=0, y=0),
                base_position=Position(x=0, y=0),
                target_position=Position(x=2, y=2),
            )


    def test_multi_position_plan_is_replanned(
        self,
    ) -> None:
        plan = PathPlan(
            mission_id="mission-1",
            robot_id="robot-1",
            phase=PathPhase.OUTBOUND,
            version=1,
            positions=(
                Position(x=4, y=4),
                Position(x=5, y=4),
            ),
            goal=Position(x=5, y=4),
        )

        self.assertEqual(
            _evidence_status_for_plan(plan).value,
            "REPLANNED",
        )

    def test_goal_only_plan_is_completed(self) -> None:
        goal = Position(x=5, y=4)
        plan = PathPlan(
            mission_id="mission-1",
            robot_id="robot-1",
            phase=PathPhase.OUTBOUND,
            version=2,
            positions=(goal,),
            goal=goal,
        )

        self.assertEqual(
            _evidence_status_for_plan(plan).value,
            "COMPLETED",
        )



    def test_return_goal_is_physical_base(self) -> None:
        world = _world()

        self.assertEqual(
            _authorized_goal_positions(
                world,
                phase=PathPhase.RETURN,
            ),
            (
                world.base_position,
            ),
        )

    def test_outbound_goals_remain_target_adjacent(
        self,
    ) -> None:
        world = _world()

        self.assertEqual(
            _authorized_goal_positions(
                world,
                phase=PathPhase.OUTBOUND,
            ),
            _authorized_arrival_positions(world),
        )

    def test_evidence_stage_describes_return_state(
        self,
    ) -> None:
        self.assertEqual(
            _evidence_stage(
                status=(
                    MissionEvidenceStatus.REPLANNED
                ),
                phase=PathPhase.RETURN,
            ),
            "operator-confirmed-return-replan",
        )
        self.assertEqual(
            _evidence_stage(
                status=(
                    MissionEvidenceStatus.COMPLETED
                ),
                phase=PathPhase.RETURN,
            ),
            "base-return-confirmed",
        )



if __name__ == "__main__":
    unittest.main()