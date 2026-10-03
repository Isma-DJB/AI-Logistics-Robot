"""Unit tests for loading versioned physical-grid calibration files."""

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml

from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibrationFileError,
    load_grid_calibration,
)

REPOSITORY_CALIBRATION = Path(
    "configs/hardware/grid_calibration_7x7.yaml"
)


class GridCalibrationLoadingTests(unittest.TestCase):
    """Verify valid and invalid YAML calibration documents."""

    def test_repository_calibration_loads_expected_geometry(self) -> None:
        calibration = load_grid_calibration(
            REPOSITORY_CALIBRATION
        )

        self.assertEqual(calibration.source_width, 800)
        self.assertEqual(calibration.source_height, 600)
        self.assertEqual(calibration.columns, 7)
        self.assertEqual(calibration.rows, 7)
        self.assertEqual(calibration.cell_size_cm, 20.0)
        self.assertEqual(calibration.pixels_per_cell, 100)
        self.assertEqual(calibration.output_width, 700)
        self.assertEqual(calibration.output_height, 700)
        self.assertEqual(
            calibration.source_points,
            (
                (106.0, 1.0),
                (597.0, 2.0),
                (588.0, 471.0),
                (122.0, 480.0),
            ),
        )

    def test_missing_calibration_file_is_rejected(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            missing_path = (
                Path(temporary_directory) / "missing.yaml"
            )

            with self.assertRaisesRegex(
                GridCalibrationFileError,
                "could not be read",
            ):
                load_grid_calibration(missing_path)

    def test_invalid_yaml_is_rejected(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            calibration_path = (
                Path(temporary_directory) / "invalid.yaml"
            )
            calibration_path.write_text(
                "source_image: [",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                GridCalibrationFileError,
                "not valid YAML",
            ):
                load_grid_calibration(calibration_path)

    def test_unsupported_schema_version_is_rejected(self) -> None:
        document = self._valid_document()
        document["schema_version"] = 2

        self._assert_invalid_document(
            document,
            "schema_version must be 1",
        )

    def test_inconsistent_physical_width_is_rejected(self) -> None:
        document = self._valid_document()
        grid = document["grid"]
        self.assertIsInstance(grid, dict)
        grid["physical_width_cm"] = 160.0

        self._assert_invalid_document(
            document,
            "physical_width_cm is inconsistent",
        )

    def test_inconsistent_rectified_width_is_rejected(self) -> None:
        document = self._valid_document()
        rectified_image = document["rectified_image"]
        self.assertIsInstance(rectified_image, dict)
        rectified_image["width"] = 800

        self._assert_invalid_document(
            document,
            "rectified_image.width is inconsistent",
        )

    def test_source_point_outside_image_is_rejected(self) -> None:
        document = self._valid_document()
        source_points = document["source_points"]
        self.assertIsInstance(source_points, dict)
        source_points["top_right"] = [800.0, 2.0]

        self._assert_invalid_document(
            document,
            "geometry is invalid",
        )

    def _assert_invalid_document(
        self,
        document: dict[str, object],
        expected_message: str,
    ) -> None:
        """Write and reject one intentionally invalid document."""

        with TemporaryDirectory() as temporary_directory:
            calibration_path = (
                Path(temporary_directory) / "calibration.yaml"
            )
            calibration_path.write_text(
                yaml.safe_dump(
                    document,
                    sort_keys=False,
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                GridCalibrationFileError,
                expected_message,
            ):
                load_grid_calibration(calibration_path)

    @staticmethod
    def _valid_document() -> dict[str, object]:
        """Return one complete valid calibration document."""

        return {
            "schema_version": 1,
            "reference_image": (
                "docs/assets/i-1.0/images/"
                "grid-reference-7x7-svga.jpg"
            ),
            "source_image": {
                "width": 800,
                "height": 600,
            },
            "grid": {
                "columns": 7,
                "rows": 7,
                "cell_size_cm": 20.0,
                "physical_width_cm": 140.0,
                "physical_height_cm": 140.0,
            },
            "source_points": {
                "top_left": [106.0, 1.0],
                "top_right": [597.0, 2.0],
                "bottom_right": [588.0, 471.0],
                "bottom_left": [122.0, 480.0],
            },
            "rectified_image": {
                "width": 700,
                "height": 700,
                "pixels_per_cell": 100,
            },
        }


if __name__ == "__main__":
    unittest.main()