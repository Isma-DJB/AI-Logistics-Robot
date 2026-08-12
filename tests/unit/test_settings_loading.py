"""Unit tests for safe YAML settings loading."""

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from ai_logistics_robot.app.settings import load_settings
from ai_logistics_robot.domain.enums import (
    CellLayer,
    Heading,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position


class SettingsLoadingTests(unittest.TestCase):
    """Verify YAML parsing and typed configuration creation."""

    def setUp(self) -> None:
        self.project_root = Path(__file__).resolve().parents[2]
        self.configuration_path = (
            self.project_root
            / "configs"
            / "simulation.yaml"
        )

    def test_reference_yaml_loads_as_validated_settings(self) -> None:
        settings = load_settings(self.configuration_path)

        self.assertEqual(settings.schema_version, 1)
        self.assertEqual(
            settings.scenario.scenario_id,
            "v1-reference",
        )
        self.assertEqual(settings.grid_map.width, 10)
        self.assertEqual(settings.grid_map.height, 10)
        self.assertEqual(
            settings.robot.initial_pose.position,
            Position(x=1, y=1),
        )
        self.assertIs(
            settings.robot.initial_pose.heading,
            Heading.NORTH,
        )
        self.assertEqual(
            settings.grid_map.layers,
            (
                CellLayer.TERRAIN,
                CellLayer.SEMANTIC,
                CellLayer.DYNAMIC,
            ),
        )

    def test_reference_yaml_preserves_unknown_values(self) -> None:
        settings = load_settings(self.configuration_path)

        self.assertIsNone(settings.robot.safety_margin_cm)
        self.assertIsNone(settings.target.debounce_ms)
        self.assertIsNone(
            settings.target.confidence_threshold
        )
        self.assertIsNone(settings.mission.maximum_replans)
        self.assertIsNone(settings.mission.timeout_s)

    def test_missing_configuration_file_is_rejected(self) -> None:
        missing_path = (
            self.project_root
            / "configs"
            / "missing.yaml"
        )

        with self.assertRaises(DomainValidationError):
            load_settings(missing_path)

    def test_invalid_yaml_is_rejected(self) -> None:
        with TemporaryDirectory() as directory:
            invalid_path = Path(directory) / "invalid.yaml"
            invalid_path.write_text(
                "grid: [",
                encoding="utf-8",
            )

            with self.assertRaises(DomainValidationError):
                load_settings(invalid_path)

    def test_invalid_heading_is_rejected(self) -> None:
        yaml_text = self.configuration_path.read_text(
            encoding="utf-8"
        )
        invalid_yaml = yaml_text.replace(
            "heading: NORTH",
            "heading: NORTHEAST",
        )

        with TemporaryDirectory() as directory:
            invalid_path = Path(directory) / "invalid-heading.yaml"
            invalid_path.write_text(
                invalid_yaml,
                encoding="utf-8",
            )

            with self.assertRaises(DomainValidationError):
                load_settings(invalid_path)

    def test_missing_required_section_is_rejected(self) -> None:
        with TemporaryDirectory() as directory:
            invalid_path = Path(directory) / "incomplete.yaml"
            invalid_path.write_text(
                "schema_version: 1\n",
                encoding="utf-8",
            )

            with self.assertRaises(DomainValidationError):
                load_settings(invalid_path)


if __name__ == "__main__":
    unittest.main()