"""Unit tests for the public I-0.8 software status."""

import subprocess
import sys
import tomllib
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

import ai_logistics_robot.app as app_package
from ai_logistics_robot.__main__ import main
from ai_logistics_robot.app import (
    MissionRunner,
    SimulationApplication,
    build_simulation_application,
)


class CommandLineStatusTests(unittest.TestCase):
    """Verify that public interfaces describe the completed software."""

    def setUp(self) -> None:
        self.project_root = Path(__file__).resolve().parents[2]

    def test_main_reports_completed_i_0_8_capabilities(
        self,
    ) -> None:
        output = StringIO()

        with redirect_stdout(output):
            exit_code = main()

        status_text = output.getvalue()

        self.assertEqual(exit_code, 0)
        self.assertIn(
            "Implementation Draft I-0.8",
            status_text,
        )
        self.assertIn(
            "complete simulation-side V1 software",
            status_text,
        )
        self.assertIn("MissionRunner", status_text)
        self.assertIn(
            "deterministic supporting adapters",
            status_text,
        )
        self.assertIn(
            "reference dependency assembly",
            status_text,
        )
        self.assertIn(
            "AC-01 through AC-12",
            status_text,
        )
        self.assertIn(
            "physical hardware",
            status_text.lower(),
        )
        self.assertIn(
            "remain deferred",
            status_text,
        )
        self.assertNotIn(
            "Implementation Draft I-0.7",
            status_text,
        )
        self.assertNotIn(
            "Application runner",
            status_text,
        )
        self.assertNotIn(
            "not implemented yet",
            status_text,
        )

    def test_module_execution_reports_same_public_milestone(
        self,
    ) -> None:
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "ai_logistics_robot",
            ],
            cwd=self.project_root,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(
            result.returncode,
            0,
            msg=result.stdout + result.stderr,
        )
        self.assertEqual(result.stderr, "")
        self.assertIn(
            "Implementation Draft I-0.8",
            result.stdout,
        )
        self.assertIn(
            "complete simulation-side V1 software",
            result.stdout,
        )
        self.assertNotIn(
            "Implementation Draft I-0.7",
            result.stdout,
        )

    def test_application_exports_and_console_script_are_public(
        self,
    ) -> None:
        expected_exports = (
            "MissionRunner",
            "SimulationApplication",
            "build_simulation_application",
        )

        self.assertEqual(
            app_package.__all__,
            expected_exports,
        )
        self.assertIs(
            app_package.MissionRunner,
            MissionRunner,
        )
        self.assertIs(
            app_package.SimulationApplication,
            SimulationApplication,
        )
        self.assertIs(
            app_package.build_simulation_application,
            build_simulation_application,
        )

        metadata = tomllib.loads(
            (
                self.project_root
                / "pyproject.toml"
            ).read_text(encoding="utf-8")
        )

        self.assertEqual(
            metadata["project"]["scripts"][
                "ai-logistics-robot"
            ],
            "ai_logistics_robot.__main__:main",
        )

    def test_readme_reports_complete_simulation_software(
        self,
    ) -> None:
        readme = (
            self.project_root
            / "README.md"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "Implementation Draft I-0.8",
            readme,
        )
        self.assertIn(
            "complete simulation-side V1 software",
            readme,
        )
        self.assertIn(
            "370 automated tests",
            readme,
        )
        self.assertIn(
            "AC-01 through AC-12",
            readme,
        )
        self.assertIn(
            "I-0.9",
            readme,
        )
        self.assertIn(
            "I-1.0",
            readme,
        )
        self.assertNotIn(
            "Implementation Draft I-0.7",
            readme,
        )
        self.assertNotIn(
            "The complete application runner, concrete "
            "perception, monitoring and clock adapters, "
            "complete acceptance scenarios",
            readme,
        )


if __name__ == "__main__":
    unittest.main()
