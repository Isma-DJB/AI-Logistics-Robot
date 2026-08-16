"""Unit tests for passive Pygame renderer lifecycle rules."""

import os
import unittest
from datetime import UTC, datetime

import pygame

from ai_logistics_robot.adapters.visualization import PygameRenderer
from ai_logistics_robot.app.settings import RendererSettings
from ai_logistics_robot.domain.enums import (
    BrainState,
    Heading,
    MissionStatus,
    PathPhase,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports import RendererPort


class PygameRendererLifecycleTests(unittest.TestCase):
    """Verify protocol, validation, shutdown, and passivity."""

    def setUp(self) -> None:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.quit()

        self.timestamp = datetime(
            2026,
            8,
            16,
            16,
            0,
            tzinfo=UTC,
        )
        self.settings = RendererSettings(
            enabled=True,
            window_title="Renderer Lifecycle Test",
            cell_size_px=32,
            status_panel_width_px=240,
            frames_per_second=30,
            recent_event_limit=3,
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
                    Position(x=1, y=2),
                }
            ),
        )
        self.initial_pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.renderer = PygameRenderer(
            settings=self.settings,
        )

    def tearDown(self) -> None:
        pygame.quit()

    def status(
        self,
        *,
        pose: RobotPose | None = None,
        plan: PathPlan | None = None,
    ) -> SystemStatus:
        """Build a valid status with an optional active plan."""

        selected_pose = (
            self.initial_pose
            if pose is None
            else pose
        )
        safety_status = SafetyStatus(
            robot_id="robot_1",
            updated_at=self.timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )

        if plan is None:
            return SystemStatus(
                robot_id="robot_1",
                observed_at=self.timestamp,
                brain_state=BrainState.WAITING_FOR_MISSION,
                robot_pose=selected_pose,
                safety_status=safety_status,
            )

        return SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=BrainState.OUTBOUND_NAVIGATION,
            robot_pose=selected_pose,
            safety_status=safety_status,
            mission_id=plan.mission_id,
            mission_status=MissionStatus.ACTIVE,
            active_plan=plan,
        )

    def valid_plan(self) -> PathPlan:
        """Build one in-bounds active outbound plan."""

        positions = (
            Position(x=0, y=0),
            Position(x=1, y=0),
            Position(x=2, y=0),
            Position(x=3, y=0),
            Position(x=3, y=1),
            Position(x=3, y=2),
        )

        return PathPlan(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.OUTBOUND,
            version=1,
            positions=positions,
            goal=positions[-1],
        )

    def event(self) -> MissionEvent:
        """Build one immutable event."""

        return MissionEvent(
            event_id="event_1",
            sequence_number=1,
            mission_id="mission_1",
            robot_id="robot_1",
            occurred_at=self.timestamp,
            source="brain",
            name="mission_started",
            brain_state=BrainState.OUTBOUND_PLANNING,
        )

    def test_renderer_satisfies_public_port(self) -> None:
        self.assertIsInstance(
            self.renderer,
            RendererPort,
        )

    def test_constructor_rejects_invalid_settings(self) -> None:
        with self.assertRaises(DomainValidationError):
            PygameRenderer(
                settings=None,  # type: ignore[arg-type]
            )

    def test_open_renderer_rejects_invalid_input_types(
        self,
    ) -> None:
        status = self.status()

        with self.assertRaises(DomainValidationError):
            self.renderer.render(  # type: ignore[arg-type]
                None,
                status,
            )

        with self.assertRaises(DomainValidationError):
            self.renderer.render(  # type: ignore[arg-type]
                self.world,
                None,
            )

        with self.assertRaises(DomainValidationError):
            self.renderer.display_event(  # type: ignore[arg-type]
                None
            )

    def test_disabled_renderer_remains_headless(
        self,
    ) -> None:
        disabled = PygameRenderer(
            settings=RendererSettings(
                enabled=False,
                window_title="Disabled Renderer",
                cell_size_px=32,
                status_panel_width_px=240,
                frames_per_second=30,
                recent_event_limit=3,
            )
        )

        disabled.render(
            self.world,
            self.status(),
        )
        disabled.display_event(
            self.event()
        )

        self.assertIsNone(
            pygame.display.get_surface()
        )
        self.assertEqual(
            disabled.recent_events,
            (),
        )
        self.assertEqual(
            disabled.visible_lines,
            (),
        )
        self.assertFalse(disabled.closed)

    def test_sdl_quit_closes_only_the_renderer(self) -> None:
        world_before = self.world
        status = self.status()

        self.renderer.render(
            self.world,
            status,
        )

        pygame.event.post(
            pygame.event.Event(pygame.QUIT)
        )

        self.renderer.render(
            self.world,
            status,
        )

        self.assertTrue(self.renderer.closed)
        self.assertIsNone(
            pygame.display.get_surface()
        )
        self.assertIs(self.world, world_before)
        self.assertIs(status.robot_pose, self.initial_pose)

    def test_close_is_idempotent(self) -> None:
        self.renderer.render(
            self.world,
            self.status(),
        )

        self.renderer.close()
        self.renderer.close()

        self.assertTrue(self.renderer.closed)
        self.assertIsNone(
            pygame.display.get_surface()
        )

    def test_closed_renderer_operations_are_noops(
        self,
    ) -> None:
        self.renderer.close()

        self.renderer.render(  # type: ignore[arg-type]
            None,
            None,
        )
        self.renderer.display_event(  # type: ignore[arg-type]
            None
        )

        self.assertTrue(self.renderer.closed)
        self.assertEqual(
            self.renderer.recent_events,
            (),
        )
        self.assertEqual(
            self.renderer.visible_lines,
            (),
        )
        self.assertIsNone(
            pygame.display.get_surface()
        )

    def test_pose_outside_world_is_rejected_before_display(
        self,
    ) -> None:
        outside_pose = RobotPose(
            position=Position(x=-1, y=0),
            heading=Heading.NORTH,
        )

        with self.assertRaises(DomainValidationError):
            self.renderer.render(
                self.world,
                self.status(pose=outside_pose),
            )

        self.assertIsNone(
            pygame.display.get_surface()
        )

    def test_plan_outside_world_is_rejected_before_display(
        self,
    ) -> None:
        outside_plan = PathPlan(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.OUTBOUND,
            version=1,
            positions=(
                Position(x=0, y=0),
                Position(x=4, y=0),
            ),
            goal=Position(x=4, y=0),
        )

        with self.assertRaises(DomainValidationError):
            self.renderer.render(
                self.world,
                self.status(plan=outside_plan),
            )

        self.assertIsNone(
            pygame.display.get_surface()
        )

    def test_render_preserves_domain_snapshots(self) -> None:
        plan = self.valid_plan()
        status = self.status(plan=plan)
        obstacles_before = self.world.obstacles
        positions_before = plan.positions
        safety_before = status.safety_status

        self.renderer.display_event(
            self.event()
        )
        self.renderer.render(
            self.world,
            status,
        )

        self.assertIs(
            self.world.obstacles,
            obstacles_before,
        )
        self.assertIs(
            status.active_plan,
            plan,
        )
        self.assertIs(
            plan.positions,
            positions_before,
        )
        self.assertIs(
            status.safety_status,
            safety_before,
        )
        self.assertEqual(
            status.robot_pose,
            self.initial_pose,
        )


if __name__ == "__main__":
    unittest.main()
