"""Unit tests for immutable Pygame renderer settings."""

import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml

from ai_logistics_robot.app.settings import (
    RendererSettings,
    load_settings,
)
from ai_logistics_robot.domain.errors import DomainValidationError


class RendererSettingsTests(unittest.TestCase):
    """Verify renderer configuration values and YAML loading."""

    def setUp(self) -> None:
        self.project_root = Path(__file__).resolve().parents[2]
        self.configuration_path = (
            self.project_root
            / "configs"
            / "simulation.yaml"
        )

    def valid_settings(
        self,
        **overrides: object,
    ) -> RendererSettings:
        """Build valid renderer settings with optional changes."""

        data: dict[str, object] = {
            "enabled": True,
            "window_title": "AI-Logistics-Robot",
            "cell_size_px": 64,
            "status_panel_width_px": 360,
            "frames_per_second": 30,
            "recent_event_limit": 6,
        }
        data.update(overrides)

        return RendererSettings(
            **data,  # type: ignore[arg-type]
        )

    def write_configuration(
        self,
        directory: str,
        document: object,
        name: str,
    ) -> Path:
        """Write one temporary YAML configuration."""

        path = Path(directory) / name
        path.write_text(
            yaml.safe_dump(
                document,
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return path

    def test_reference_values_are_accepted(self) -> None:
        settings = self.valid_settings()

        self.assertTrue(settings.enabled)
        self.assertEqual(
            settings.window_title,
            "AI-Logistics-Robot",
        )
        self.assertEqual(settings.cell_size_px, 64)
        self.assertEqual(
            settings.status_panel_width_px,
            360,
        )
        self.assertEqual(settings.frames_per_second, 30)
        self.assertEqual(settings.recent_event_limit, 6)

    def test_disabled_renderer_is_valid(self) -> None:
        settings = self.valid_settings(enabled=False)

        self.assertFalse(settings.enabled)

    def test_enabled_must_be_boolean(self) -> None:
        for value in (1, 0, "true", None):
            with self.subTest(value=value):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.valid_settings(enabled=value)

    def test_window_title_must_be_non_empty_text(
        self,
    ) -> None:
        for value in ("", "   ", None, 1):
            with self.subTest(value=value):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.valid_settings(window_title=value)

    def test_dimensions_and_limits_are_positive_integers(
        self,
    ) -> None:
        fields = (
            "cell_size_px",
            "status_panel_width_px",
            "frames_per_second",
            "recent_event_limit",
        )
        invalid_values = (0, -1, 1.5, True, None)

        for field in fields:
            for value in invalid_values:
                with self.subTest(
                    field=field,
                    value=value,
                ):
                    with self.assertRaises(
                        DomainValidationError
                    ):
                        self.valid_settings(
                            **{field: value},
                        )

    def test_renderer_settings_are_immutable(self) -> None:
        settings = self.valid_settings()

        with self.assertRaises(FrozenInstanceError):
            settings.cell_size_px = 32  # type: ignore[misc]

    def test_reference_yaml_loads_renderer_settings(
        self,
    ) -> None:
        settings = load_settings(self.configuration_path)

        self.assertEqual(
            settings.renderer,
            self.valid_settings(),
        )

    def test_missing_renderer_section_is_rejected(
        self,
    ) -> None:
        document = yaml.safe_load(
            self.configuration_path.read_text(
                encoding="utf-8"
            )
        )
        document.pop("renderer")

        with TemporaryDirectory() as directory:
            path = self.write_configuration(
                directory,
                document,
                "missing-renderer.yaml",
            )

            with self.assertRaises(
                DomainValidationError
            ):
                load_settings(path)

    def test_invalid_renderer_yaml_values_are_rejected(
        self,
    ) -> None:
        cases: tuple[tuple[str, object], ...] = (
            ("enabled", "true"),
            ("window_title", " "),
            ("cell_size_px", 0),
            ("status_panel_width_px", -1),
            ("frames_per_second", 1.5),
            ("recent_event_limit", True),
        )

        for field, value in cases:
            with self.subTest(field=field, value=value):
                document = yaml.safe_load(
                    self.configuration_path.read_text(
                        encoding="utf-8"
                    )
                )
                document["renderer"][field] = value

                with TemporaryDirectory() as directory:
                    path = self.write_configuration(
                        directory,
                        document,
                        f"invalid-{field}.yaml",
                    )

                    with self.assertRaises(
                        DomainValidationError
                    ):
                        load_settings(path)


if __name__ == "__main__":
    unittest.main()
