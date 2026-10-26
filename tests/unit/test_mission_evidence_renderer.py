"""Unit tests for Batch E mission evidence rendering."""

import unittest
from datetime import UTC, datetime

import numpy as np

from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
    ImageArray,
)
from ai_logistics_robot.adapters.visualization.mission_evidence_renderer import (
    MissionEvidenceRenderer,
)
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


class MissionEvidenceRendererTests(unittest.TestCase):
    """Verify deterministic rendering without physical devices."""

    def setUp(self) -> None:
        self.calibration = GridCalibration(
            source_width=800,
            source_height=600,
            source_points=(
                (100.0, 20.0),
                (700.0, 20.0),
                (700.0, 580.0),
                (100.0, 580.0),
            ),
            columns=4,
            rows=4,
            cell_size_cm=20,
            pixels_per_cell=50,
        )
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
                30,
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
        self.evidence = MissionEvidence(
            scenario_id="batch-e",
            step_number=1,
            stage="outbound",
            status=MissionEvidenceStatus.PLANNED,
            world=self.world,
            perception=self.perception,
            plan=self.plan,
            confirmed_positions=(
                Position(x=0, y=0),
            ),
            next_position=Position(x=0, y=1),
        )
        self.renderer = MissionEvidenceRenderer(
            calibration=self.calibration,
        )

    def render_obstacle_evidence(
        self,
        item: ObstacleEvidence,
    ) -> ImageArray:
        obstacle_map = PhysicalObstacleMap(
            base_world=self.world,
            evidence=(item,),
        )
        evidence = MissionEvidence(
            scenario_id="batch-e",
            step_number=2,
            stage="obstacle validation",
            status=MissionEvidenceStatus.REPLANNED,
            world=obstacle_map.confirmed_world,
            perception=self.perception,
            obstacle_map=obstacle_map,
            confirmed_positions=(
                self.perception.robot_pose.position,
            ),
        )
        source = np.zeros(
            (
                self.calibration.output_height,
                self.calibration.output_width,
                3,
            ),
            dtype=np.uint8,
        )
        return self.renderer.render(
            source,
            evidence,
        )

    def test_renderer_retains_calibration(self) -> None:
        self.assertIs(
            self.renderer.calibration,
            self.calibration,
        )

    def test_invalid_calibration_is_rejected(self) -> None:
        with self.assertRaises(
            DomainValidationError
        ):
            MissionEvidenceRenderer(
                calibration=None,  # type: ignore[arg-type]
            )

    def test_render_returns_bgr_copy(self) -> None:
        source = np.zeros(
            (
                self.calibration.output_height,
                self.calibration.output_width,
                3,
            ),
            dtype=np.uint8,
        )
        original = source.copy()

        rendered = self.renderer.render(
            source,
            self.evidence,
        )

        self.assertEqual(rendered.shape, source.shape)
        self.assertEqual(rendered.dtype, np.uint8)
        self.assertFalse(
            np.array_equal(rendered, original)
        )
        np.testing.assert_array_equal(
            source,
            original,
        )

    def test_grayscale_frame_is_converted_to_bgr(
        self,
    ) -> None:
        source = np.zeros(
            (
                self.calibration.output_height,
                self.calibration.output_width,
            ),
            dtype=np.uint8,
        )

        rendered = self.renderer.render(
            source,
            self.evidence,
        )

        self.assertEqual(
            rendered.shape,
            (
                self.calibration.output_height,
                self.calibration.output_width,
                3,
            ),
        )

    def test_invalid_frame_type_is_rejected(self) -> None:
        with self.assertRaises(
            DomainValidationError
        ):
            self.renderer.render(
                None,
                self.evidence,
            )

    def test_wrong_frame_dimensions_are_rejected(
        self,
    ) -> None:
        source = np.zeros(
            (100, 100, 3),
            dtype=np.uint8,
        )

        with self.assertRaisesRegex(
            InvariantViolationError,
            "dimensions",
        ):
            self.renderer.render(
                source,
                self.evidence,
            )

    def test_world_and_calibration_must_match(
        self,
    ) -> None:
        incompatible_calibration = GridCalibration(
            source_width=800,
            source_height=600,
            source_points=(
                (100.0, 20.0),
                (700.0, 20.0),
                (700.0, 580.0),
                (100.0, 580.0),
            ),
            columns=5,
            rows=5,
            cell_size_cm=20,
            pixels_per_cell=40,
        )
        renderer = MissionEvidenceRenderer(
            calibration=incompatible_calibration,
        )
        source = np.zeros(
            (
                incompatible_calibration.output_height,
                incompatible_calibration.output_width,
                3,
            ),
            dtype=np.uint8,
        )

        with self.assertRaisesRegex(
            InvariantViolationError,
            "world dimensions",
        ):
            renderer.render(
                source,
                self.evidence,
            )

    def test_target_and_obstacle_cells_are_visible(
        self,
    ) -> None:
        source = np.zeros(
            (
                self.calibration.output_height,
                self.calibration.output_width,
                3,
            ),
            dtype=np.uint8,
        )

        rendered = self.renderer.render(
            source,
            self.evidence,
        )

        obstacle_center = rendered[74, 124]
        target_center = rendered[24, 174]

        self.assertGreater(
            int(obstacle_center[2]),
            int(obstacle_center[0]),
        )
        self.assertGreater(
            int(target_center[0]),
            0,
        )
        self.assertGreater(
            int(target_center[2]),
            0,
        )

    def test_next_cell_receives_yellow_border(
        self,
    ) -> None:
        source = np.zeros(
            (
                self.calibration.output_height,
                self.calibration.output_width,
                3,
            ),
            dtype=np.uint8,
        )

        rendered = self.renderer.render(
            source,
            self.evidence,
        )

        next_border = rendered[103, 25]
        self.assertGreater(
            int(next_border[1]),
            0,
        )
        self.assertGreater(
            int(next_border[2]),
            0,
        )

    def test_camera_detection_is_orange(
        self,
    ) -> None:
        rendered = self.render_obstacle_evidence(
            ObstacleEvidence(
                position=Position(x=3, y=2),
                source=ObstacleEvidenceSource.CAMERA,
                status=ObstacleEvidenceStatus.DETECTED,
                confidence=0.8,
            )
        )

        center = rendered[74, 174]
        self.assertGreater(
            int(center[1]),
            int(center[0]),
        )
        self.assertGreater(
            int(center[2]),
            int(center[1]),
        )

    def test_operator_confirmation_is_red(
        self,
    ) -> None:
        rendered = self.render_obstacle_evidence(
            ObstacleEvidence(
                position=Position(x=3, y=2),
                source=ObstacleEvidenceSource.OPERATOR,
                status=ObstacleEvidenceStatus.CONFIRMED,
                confidence=1.0,
            )
        )

        center = rendered[74, 174]
        self.assertGreater(
            int(center[2]),
            int(center[1]),
        )
        self.assertGreater(
            int(center[2]),
            int(center[0]),
        )

    def test_local_sensor_has_cyan_border(
        self,
    ) -> None:
        rendered = self.render_obstacle_evidence(
            ObstacleEvidence(
                position=Position(x=3, y=2),
                source=ObstacleEvidenceSource.LOCAL_SENSOR,
                status=ObstacleEvidenceStatus.CONFIRMED,
                confidence=1.0,
            )
        )

        border = rendered[74, 153]
        self.assertGreater(
            int(border[0]),
            0,
        )
        self.assertGreater(
            int(border[1]),
            0,
        )

    def test_cleared_cell_has_green_border(
        self,
    ) -> None:
        rendered = self.render_obstacle_evidence(
            ObstacleEvidence(
                position=Position(x=2, y=2),
                source=ObstacleEvidenceSource.OPERATOR,
                status=ObstacleEvidenceStatus.CLEARED,
                confidence=1.0,
            )
        )

        border = rendered[74, 103]
        self.assertGreater(
            int(border[1]),
            int(border[0]),
        )
        self.assertGreater(
            int(border[1]),
            int(border[2]),
        )

    def test_robot_heading_is_drawn(self) -> None:
        source = np.zeros(
            (
                self.calibration.output_height,
                self.calibration.output_width,
                3,
            ),
            dtype=np.uint8,
        )

        rendered = self.renderer.render(
            source,
            self.evidence,
        )

        robot_center = rendered[174, 24]
        self.assertTrue(
            np.any(robot_center > 0)
        )


if __name__ == "__main__":
    unittest.main()
