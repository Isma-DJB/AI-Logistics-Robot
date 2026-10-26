"""Unit tests for camera-based grid-cell change proposals."""

import unittest

import numpy as np

from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
    ImageArray,
)
from ai_logistics_robot.adapters.hardware.grid_cell_change_detector import (
    GridCellChangeDetector,
)
from ai_logistics_robot.control.physical_obstacle_evidence import (
    ObstacleEvidenceSource,
    ObstacleEvidenceStatus,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position


class GridCellChangeDetectorTests(unittest.TestCase):
    """Verify passive cell proposals without hardware access."""

    def setUp(self) -> None:
        self.calibration = GridCalibration(
            source_width=90,
            source_height=90,
            source_points=(
                (0.0, 0.0),
                (89.0, 0.0),
                (89.0, 89.0),
                (0.0, 89.0),
            ),
            columns=3,
            rows=3,
            cell_size_cm=20,
            pixels_per_cell=30,
        )
        self.detector = GridCellChangeDetector(
            calibration=self.calibration,
            pixel_difference_threshold=25,
            minimum_changed_fraction=0.2,
            cell_margin_pixels=3,
        )
        self.reference = np.zeros(
            (90, 90, 3),
            dtype=np.uint8,
        )

    def changed_frame(
        self,
        *positions: Position,
    ) -> ImageArray:
        frame = self.reference.copy()
        pixels = self.calibration.pixels_per_cell

        for position in positions:
            image_row = (
                self.calibration.rows
                - 1
                - position.y
            )
            left = position.x * pixels + 3
            right = (position.x + 1) * pixels - 3
            top = image_row * pixels + 3
            bottom = (image_row + 1) * pixels - 3
            frame[top:bottom, left:right] = 255

        return frame

    def test_identical_frames_produce_no_proposals(
        self,
    ) -> None:
        proposals = self.detector.detect(
            self.reference,
            self.reference.copy(),
        )

        self.assertEqual(proposals, ())

    def test_changed_cell_is_mapped_to_domain_position(
        self,
    ) -> None:
        proposals = self.detector.detect(
            self.reference,
            self.changed_frame(
                Position(x=1, y=2)
            ),
        )

        self.assertEqual(len(proposals), 1)
        proposal = proposals[0]
        self.assertEqual(
            proposal.position,
            Position(x=1, y=2),
        )
        self.assertIs(
            proposal.source,
            ObstacleEvidenceSource.CAMERA,
        )
        self.assertIs(
            proposal.status,
            ObstacleEvidenceStatus.DETECTED,
        )
        self.assertGreater(
            proposal.confidence,
            0.2,
        )

    def test_multiple_proposals_are_deterministic(
        self,
    ) -> None:
        proposals = self.detector.detect(
            self.reference,
            self.changed_frame(
                Position(x=2, y=2),
                Position(x=0, y=0),
                Position(x=1, y=1),
            ),
        )

        self.assertEqual(
            tuple(
                item.position
                for item in proposals
            ),
            (
                Position(x=0, y=0),
                Position(x=1, y=1),
                Position(x=2, y=2),
            ),
        )

    def test_excluded_positions_are_ignored(
        self,
    ) -> None:
        changed_position = Position(x=1, y=1)

        proposals = self.detector.detect(
            self.reference,
            self.changed_frame(
                changed_position
            ),
            excluded_positions=frozenset(
                {
                    changed_position,
                }
            ),
        )

        self.assertEqual(proposals, ())

    def test_small_change_below_fraction_is_ignored(
        self,
    ) -> None:
        current = self.reference.copy()
        current[35:38, 35:38] = 255

        proposals = self.detector.detect(
            self.reference,
            current,
        )

        self.assertEqual(proposals, ())

    def test_grayscale_frames_are_supported(
        self,
    ) -> None:
        reference = np.zeros(
            (90, 90),
            dtype=np.uint8,
        )
        current = reference.copy()
        current[3:27, 33:57] = 255

        proposals = self.detector.detect(
            reference,
            current,
        )

        self.assertEqual(
            proposals[0].position,
            Position(x=1, y=2),
        )

    def test_source_frames_are_not_modified(
        self,
    ) -> None:
        current = self.changed_frame(
            Position(x=1, y=1)
        )
        reference_before = self.reference.copy()
        current_before = current.copy()

        self.detector.detect(
            self.reference,
            current,
        )

        np.testing.assert_array_equal(
            self.reference,
            reference_before,
        )
        np.testing.assert_array_equal(
            current,
            current_before,
        )

    def test_grid_origin_offsets_positions(
        self,
    ) -> None:
        detector = GridCellChangeDetector(
            calibration=self.calibration,
            pixel_difference_threshold=25,
            minimum_changed_fraction=0.2,
            cell_margin_pixels=3,
            grid_origin=Position(x=5, y=7),
        )

        proposals = detector.detect(
            self.reference,
            self.changed_frame(
                Position(x=1, y=2)
            ),
        )

        self.assertEqual(
            proposals[0].position,
            Position(x=6, y=9),
        )

    def test_invalid_settings_are_rejected(
        self,
    ) -> None:
        invalid_settings = (
            {
                "pixel_difference_threshold": 0,
            },
            {
                "pixel_difference_threshold": True,
            },
            {
                "minimum_changed_fraction": 0,
            },
            {
                "minimum_changed_fraction": 1.1,
            },
            {
                "cell_margin_pixels": 15,
            },
            {
                "cell_margin_pixels": True,
            },
        )

        for settings in invalid_settings:
            with self.subTest(settings=settings):
                with self.assertRaises(
                    DomainValidationError
                ):
                    GridCellChangeDetector(
                        calibration=self.calibration,
                        **settings,
                    )

    def test_invalid_frames_are_rejected(self) -> None:
        invalid_frames = (
            None,
            np.zeros(
                (90, 90),
                dtype=np.float32,
            ),
            np.zeros(
                (90, 90, 4),
                dtype=np.uint8,
            ),
        )

        for frame in invalid_frames:
            with self.subTest(
                frame_type=type(frame),
            ):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.detector.detect(
                        frame,
                        self.reference,
                    )

    def test_wrong_dimensions_are_rejected(
        self,
    ) -> None:
        wrong_size = np.zeros(
            (60, 60, 3),
            dtype=np.uint8,
        )

        with self.assertRaisesRegex(
            InvariantViolationError,
            "dimensions",
        ):
            self.detector.detect(
                self.reference,
                wrong_size,
            )

    def test_exclusions_must_be_immutable_positions(
        self,
    ) -> None:
        with self.assertRaises(
            DomainValidationError
        ):
            self.detector.detect(
                self.reference,
                self.reference,
                excluded_positions={  # type: ignore[arg-type]
                    Position(x=0, y=0),
                },
            )


if __name__ == "__main__":
    unittest.main()
