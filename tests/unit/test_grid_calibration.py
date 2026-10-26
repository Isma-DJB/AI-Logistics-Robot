"""Unit tests for fixed overhead grid perspective calibration."""

import unittest

import numpy as np

from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
    build_perspective_matrix,
    rectified_pixel_to_cell,
    rectify_frame,
)


class GridCalibrationTests(unittest.TestCase):
    """Verify calibration validation and perspective operations."""

    def test_default_output_represents_seven_by_seven_grid(self) -> None:
        calibration = GridCalibration(
            source_width=800,
            source_height=600,
            source_points=(
                (100.0, 50.0),
                (700.0, 50.0),
                (700.0, 550.0),
                (100.0, 550.0),
            ),
        )

        self.assertEqual(calibration.columns, 7)
        self.assertEqual(calibration.rows, 7)
        self.assertEqual(calibration.cell_size_cm, 20.0)
        self.assertEqual(calibration.output_width, 700)
        self.assertEqual(calibration.output_height, 700)

    def test_duplicate_corner_points_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be unique"):
            GridCalibration(
                source_width=800,
                source_height=600,
                source_points=(
                    (100.0, 50.0),
                    (700.0, 50.0),
                    (700.0, 550.0),
                    (100.0, 50.0),
                ),
            )

    def test_corner_outside_source_image_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "outside the image width"):
            GridCalibration(
                source_width=800,
                source_height=600,
                source_points=(
                    (100.0, 50.0),
                    (800.0, 50.0),
                    (700.0, 550.0),
                    (100.0, 550.0),
                ),
            )

    def test_collinear_corner_points_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "valid quadrilateral"):
            GridCalibration(
                source_width=800,
                source_height=600,
                source_points=(
                    (100.0, 100.0),
                    (200.0, 100.0),
                    (300.0, 100.0),
                    (400.0, 100.0),
                ),
            )

    def test_square_source_builds_identity_perspective_matrix(self) -> None:
        calibration = GridCalibration(
            source_width=4,
            source_height=4,
            source_points=(
                (0.0, 0.0),
                (3.0, 0.0),
                (3.0, 3.0),
                (0.0, 3.0),
            ),
            columns=2,
            rows=2,
            pixels_per_cell=2,
        )

        matrix = build_perspective_matrix(calibration)

        np.testing.assert_allclose(matrix, np.eye(3), atol=1e-8)

    def test_identity_calibration_preserves_frame(self) -> None:
        calibration = GridCalibration(
            source_width=4,
            source_height=4,
            source_points=(
                (0.0, 0.0),
                (3.0, 0.0),
                (3.0, 3.0),
                (0.0, 3.0),
            ),
            columns=2,
            rows=2,
            pixels_per_cell=2,
        )
        frame = np.arange(48, dtype=np.uint8).reshape((4, 4, 3))

        rectified = rectify_frame(frame, calibration)

        self.assertEqual(rectified.shape, (4, 4, 3))
        np.testing.assert_array_equal(rectified, frame)

    def test_frame_dimensions_must_match_calibration(self) -> None:
        calibration = GridCalibration(
            source_width=4,
            source_height=4,
            source_points=(
                (0.0, 0.0),
                (3.0, 0.0),
                (3.0, 3.0),
                (0.0, 3.0),
            ),
            columns=2,
            rows=2,
            pixels_per_cell=2,
        )
        frame = np.zeros((5, 4, 3), dtype=np.uint8)

        with self.assertRaisesRegex(ValueError, "do not match"):
            rectify_frame(frame, calibration)

    def test_rectified_pixels_map_to_zero_based_grid_cells(self) -> None:
        calibration = GridCalibration(
            source_width=800,
            source_height=600,
            source_points=(
                (100.0, 50.0),
                (700.0, 50.0),
                (700.0, 550.0),
                (100.0, 550.0),
            ),
        )

        self.assertEqual(
            rectified_pixel_to_cell(0, 0, calibration),
            (0, 0),
        )
        self.assertEqual(
            rectified_pixel_to_cell(321, 456, calibration),
            (3, 4),
        )
        self.assertEqual(
            rectified_pixel_to_cell(699, 699, calibration),
            (6, 6),
        )

    def test_rectified_pixel_outside_grid_is_rejected(self) -> None:
        calibration = GridCalibration(
            source_width=800,
            source_height=600,
            source_points=(
                (100.0, 50.0),
                (700.0, 50.0),
                (700.0, 550.0),
                (100.0, 550.0),
            ),
        )

        with self.assertRaisesRegex(ValueError, "outside the grid"):
            rectified_pixel_to_cell(700, 699, calibration)


if __name__ == "__main__":
    unittest.main()