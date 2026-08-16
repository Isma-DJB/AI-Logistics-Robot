"""Integration tests for the reference Pygame visualization."""

import os
import unittest
from contextlib import redirect_stdout
from datetime import UTC, datetime
from io import StringIO
from pathlib import Path

import pygame

from ai_logistics_robot.__main__ import main
from ai_logistics_robot.adapters.visualization import PygameRenderer
from ai_logistics_robot.app.settings import Settings, load_settings
from ai_logistics_robot.domain.enums import (
    BrainState,
    MissionStatus,
    PathPhase,
    SafetySeverity,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.planning import AStarPlanner
from ai_logistics_robot.ports import RendererPort


class PygameRendererReferenceTests(unittest.TestCase):
    """Verify visualization against the V1 reference configuration."""

    def setUp(self) -> None:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.quit()

        self.project_root = Path(__file__).resolve().parents[3]
        self.settings: Settings = load_settings(
            self.project_root
            / "configs"
            / "simulation.yaml"
        )
        self.timestamp = datetime(
            2026,
            8,
            16,
            18,
            0,
            tzinfo=UTC,
        )

    def tearDown(self) -> None:
        pygame.quit()

    def test_reference_plan_and_status_render_headlessly(
        self,
    ) -> None:
        world = self.settings.grid_map
        planner = AStarPlanner()
        plan = planner.create_plan(
            mission_id="mission_reference",
            robot_id=self.settings.robot.robot_id,
            start_pose=self.settings.robot.initial_pose,
            authorized_goals=(
                world.authorized_arrival_positions
            ),
            world=world,
            phase=PathPhase.OUTBOUND,
            version=1,
        )
        status = SystemStatus(
            robot_id=self.settings.robot.robot_id,
            observed_at=self.timestamp,
            brain_state=BrainState.OUTBOUND_NAVIGATION,
            robot_pose=self.settings.robot.initial_pose,
            safety_status=SafetyStatus(
                robot_id=self.settings.robot.robot_id,
                updated_at=self.timestamp,
                latched=False,
                severity=SafetySeverity.INFO,
            ),
            mission_id="mission_reference",
            mission_status=MissionStatus.ACTIVE,
            active_plan=plan,
        )
        event = MissionEvent(
            event_id="event_reference_1",
            sequence_number=1,
            mission_id="mission_reference",
            robot_id=self.settings.robot.robot_id,
            occurred_at=self.timestamp,
            source="integration",
            name="reference_plan_rendered",
            brain_state=BrainState.OUTBOUND_NAVIGATION,
        )
        renderer = PygameRenderer(
            settings=self.settings.renderer,
        )

        self.assertIsInstance(
            renderer,
            RendererPort,
        )

        renderer.display_event(event)
        renderer.render(
            world,
            status,
        )

        surface = pygame.display.get_surface()

        self.assertIsNotNone(surface)
        assert surface is not None

        self.assertEqual(
            surface.get_size(),
            (
                world.width
                * self.settings.renderer.cell_size_px
                + self.settings.renderer.status_panel_width_px,
                world.height
                * self.settings.renderer.cell_size_px,
            ),
        )
        self.assertIn(
            plan.goal,
            world.authorized_arrival_positions,
        )
        self.assertEqual(
            renderer.recent_events,
            (event,),
        )
        self.assertIn(
            "Mission: mission_reference [ACTIVE]",
            renderer.visible_lines,
        )
        self.assertIn(
            "Plan: OUTBOUND v1",
            renderer.visible_lines,
        )
        self.assertIn(
            "Event: #1 reference_plan_rendered",
            renderer.visible_lines,
        )
        self.assertIs(
            self.settings.grid_map,
            world,
        )

        renderer.close()

        self.assertTrue(renderer.closed)
        self.assertIsNone(
            pygame.display.get_surface()
        )

    def test_command_line_reports_i_0_7_renderer_milestone(
        self,
    ) -> None:
        output = StringIO()

        with redirect_stdout(output):
            result = main()

        rendered_output = output.getvalue()

        self.assertEqual(result, 0)
        self.assertIn(
            "Implementation Draft I-0.7",
            rendered_output,
        )
        self.assertIn(
            "passive Pygame visualization",
            rendered_output,
        )
        self.assertNotIn(
            "graphical rendering",
            rendered_output.lower(),
        )


if __name__ == "__main__":
    unittest.main()
