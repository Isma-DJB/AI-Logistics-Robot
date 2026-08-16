"""Unit tests for the reference simulation bootstrap."""

import os
import subprocess
import sys
import unittest
from dataclasses import FrozenInstanceError, replace
from datetime import UTC, datetime
from pathlib import Path

from ai_logistics_robot.adapters.monitoring import InMemoryMonitoring
from ai_logistics_robot.adapters.simulation import (
    GridWorld,
    GridWorldPerception,
    HeadlessRenderer,
    SimulatedClock,
)
from ai_logistics_robot.adapters.visualization import PygameRenderer
from ai_logistics_robot.app import (
    SimulationApplication,
    build_simulation_application,
)
from ai_logistics_robot.app.settings import Settings, load_settings
from ai_logistics_robot.brain import DeterministicBrain
from ai_logistics_robot.control import SafeRobotControl
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.memory import InMemoryMissionMemory
from ai_logistics_robot.planning import AStarPlanner
from ai_logistics_robot.ports import (
    BrainPort,
    ClockPort,
    ControlPort,
    MemoryPort,
    MonitoringPort,
    PerceptionPort,
    PlanningPort,
    RendererPort,
    SimulationPort,
)


class SimulationBootstrapTests(unittest.TestCase):
    """Verify complete isolated reference dependency assembly."""

    def setUp(self) -> None:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

        self.project_root = Path(__file__).resolve().parents[2]
        self.settings: Settings = load_settings(
            self.project_root
            / "configs"
            / "simulation.yaml"
        )
        self.epoch = datetime(
            2026,
            8,
            16,
            13,
            0,
            tzinfo=UTC,
        )

    def build_application(
        self,
        *,
        enabled: bool,
    ) -> SimulationApplication:
        """Build one application with explicit renderer selection."""

        settings = replace(
            self.settings,
            renderer=replace(
                self.settings.renderer,
                enabled=enabled,
            ),
        )

        return build_simulation_application(
            settings=settings,
            epoch=self.epoch,
        )

    def test_headless_reference_components_are_assembled(
        self,
    ) -> None:
        application = self.build_application(
            enabled=False
        )

        self.assertIsInstance(
            application,
            SimulationApplication,
        )
        self.assertIsInstance(
            application.simulation,
            GridWorld,
        )
        self.assertIsInstance(
            application.clock,
            SimulatedClock,
        )
        self.assertIsInstance(
            application.perception,
            GridWorldPerception,
        )
        self.assertIsInstance(
            application.planning,
            AStarPlanner,
        )
        self.assertIsInstance(
            application.control,
            SafeRobotControl,
        )
        self.assertIsInstance(
            application.memory,
            InMemoryMissionMemory,
        )
        self.assertIsInstance(
            application.monitoring,
            InMemoryMonitoring,
        )
        self.assertIsInstance(
            application.brain,
            DeterministicBrain,
        )
        self.assertIsInstance(
            application.renderer,
            HeadlessRenderer,
        )
        self.assertTrue(application.runner.configured)
        self.assertFalse(application.runner.running)

    def test_assembled_components_satisfy_public_ports(
        self,
    ) -> None:
        application = self.build_application(
            enabled=False
        )

        components = (
            (application.simulation, SimulationPort),
            (application.clock, ClockPort),
            (application.perception, PerceptionPort),
            (application.planning, PlanningPort),
            (application.control, ControlPort),
            (application.memory, MemoryPort),
            (application.monitoring, MonitoringPort),
            (application.brain, BrainPort),
            (application.renderer, RendererPort),
        )

        for component, port in components:
            with self.subTest(
                component=component,
                port=port,
            ):
                self.assertIsInstance(
                    component,
                    port,
                )

    def test_component_state_matches_validated_settings(
        self,
    ) -> None:
        application = self.build_application(
            enabled=False
        )

        self.assertEqual(
            application.settings.renderer.enabled,
            False,
        )
        self.assertIs(
            application.simulation.read_world(),
            application.settings.grid_map,
        )
        self.assertEqual(
            application.clock.now(),
            self.epoch,
        )
        self.assertEqual(
            application.clock.monotonic(),
            0.0,
        )

        snapshot = application.perception.observe()
        status = application.runner.get_status()

        self.assertEqual(
            snapshot.robot_id,
            application.settings.robot.robot_id,
        )
        self.assertEqual(
            snapshot.robot_pose,
            application.settings.robot.initial_pose,
        )
        self.assertEqual(
            snapshot.captured_at,
            self.epoch,
        )
        self.assertEqual(
            status.robot_id,
            application.settings.robot.robot_id,
        )
        self.assertEqual(
            status.robot_pose,
            application.settings.robot.initial_pose,
        )
        self.assertEqual(
            application.monitoring.events_for(
                "mission_unknown"
            ),
            (),
        )

    def test_graphical_renderer_is_selected_when_enabled(
        self,
    ) -> None:
        application = self.build_application(
            enabled=True
        )

        self.assertIsInstance(
            application.renderer,
            PygameRenderer,
        )
        self.assertIsInstance(
            application.renderer,
            RendererPort,
        )
        self.assertTrue(
            application.settings.renderer.enabled
        )

        application.renderer.close()

    def test_separate_assemblies_have_isolated_mutable_state(
        self,
    ) -> None:
        first = self.build_application(
            enabled=False
        )
        second = self.build_application(
            enabled=False
        )

        self.assertIsNot(
            second.simulation,
            first.simulation,
        )
        self.assertIsNot(
            second.clock,
            first.clock,
        )
        self.assertIsNot(
            second.perception,
            first.perception,
        )
        self.assertIsNot(
            second.control,
            first.control,
        )
        self.assertIsNot(
            second.memory,
            first.memory,
        )
        self.assertIsNot(
            second.monitoring,
            first.monitoring,
        )
        self.assertIsNot(
            second.brain,
            first.brain,
        )
        self.assertIsNot(
            second.renderer,
            first.renderer,
        )
        self.assertIsNot(
            second.runner,
            first.runner,
        )

    def test_application_component_references_are_read_only(
        self,
    ) -> None:
        application = self.build_application(
            enabled=False
        )

        with self.assertRaises(FrozenInstanceError):
            application.runner = (  # type: ignore[misc]
                application.runner
            )

    def test_invalid_settings_are_rejected_before_assembly(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "settings",
        ):
            build_simulation_application(  # type: ignore[arg-type]
                settings=None,
                epoch=self.epoch,
            )

    def test_headless_bootstrap_does_not_import_pygame(
        self,
    ) -> None:
        script = "\n".join(
            (
                "import sys",
                "from dataclasses import replace",
                "from datetime import UTC, datetime",
                "from pathlib import Path",
                "from ai_logistics_robot.app import "
                "build_simulation_application",
                "from ai_logistics_robot.app.settings import "
                "load_settings",
                "assert 'pygame' not in sys.modules",
                "settings = load_settings("
                "Path('configs/simulation.yaml'))",
                "settings = replace("
                "settings, renderer=replace("
                "settings.renderer, enabled=False))",
                "application = build_simulation_application(",
                "    settings=settings,",
                "    epoch=datetime("
                "2026, 8, 16, 13, 0, tzinfo=UTC),",
                ")",
                "assert application.renderer.__class__.__name__ "
                "== 'HeadlessRenderer'",
                "assert 'pygame' not in sys.modules",
            )
        )

        result = subprocess.run(
            [
                sys.executable,
                "-c",
                script,
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


if __name__ == "__main__":
    unittest.main()
