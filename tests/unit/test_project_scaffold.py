"""Tests for the Implementation Draft I-0.1 scaffold."""

from __future__ import annotations

import subprocess
import sys
import tomllib
import unittest
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = PROJECT_ROOT / "src"

if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))


class ProjectScaffoldTests(unittest.TestCase):
    """Verify packaging, configuration, and architectural structure."""

    def test_package_is_importable(self) -> None:
        import ai_logistics_robot

        self.assertEqual(ai_logistics_robot.__version__, "0.1.0.dev0")

    def test_pyproject_declares_expected_package(self) -> None:
        with (PROJECT_ROOT / "pyproject.toml").open("rb") as stream:
            configuration = tomllib.load(stream)

        project = configuration["project"]
        self.assertEqual(project["name"], "ai-logistics-robot")
        self.assertEqual(project["requires-python"], ">=3.11")

    def test_reference_grid_matches_approved_v1_environment(self) -> None:
        with (PROJECT_ROOT / "configs" / "simulation.yaml").open(encoding="utf-8") as stream:
            configuration = yaml.safe_load(stream)

        self.assertEqual(configuration["grid"]["width"], 10)
        self.assertEqual(configuration["grid"]["height"], 10)
        self.assertEqual(configuration["grid"]["cell_size_cm"], 20)
        self.assertEqual(configuration["base"]["position"], [1, 1])
        self.assertEqual(configuration["target"]["position"], [8, 7])

    def test_physical_dimensions_are_not_guessed(self) -> None:
        with (PROJECT_ROOT / "configs" / "robot.yaml").open(encoding="utf-8") as stream:
            configuration = yaml.safe_load(stream)

        self.assertTrue(
            all(value is None for value in configuration["physical_dimensions"].values())
        )

    def test_structure_checker_passes(self) -> None:
        result = subprocess.run(
            [sys.executable, "tools/check_project_structure.py"],
            cwd=PROJECT_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, msg=result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
