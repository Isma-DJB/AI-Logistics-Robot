"""Unit tests for Pygame status, plan, and event visualization."""

import os
import unittest
from datetime import UTC, datetime, timedelta

import pygame

from ai_logistics_robot.adapters.visualization import PygameRenderer
from ai_logistics_robot.app.settings import RendererSettings
from ai_logistics_robot.domain.enums import (
    BrainState,
    FailureReason,
    Heading,
    MissionStatus,
    PathPhase,
    SafetySeverity,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap


class PygameRendererStatusTests(unittest.TestCase):
    """Verify passive plan, status, and event visualization."""

    def setUp(self) -> None:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.quit()

        self.timestamp = datetime(
            2026,
            8,
            16,
            14,
            0,
            tzinfo=UTC,
        )
        self.settings = RendererSettings(
            enabled=True,
            window_title="Renderer Status Test",
            cell_size_px=32,
            status_panel_width_px=260,
            frames_per_second=30,
            recent_event_limit=3,
        )
        self.world = GridMap(
            width=5,
            height=4,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=0, y=0),
            target_position=Position(x=4, y=3),
            obstacles=frozenset(
                {
                    Position(x=2, y=2),
                }
            ),
        )
        self.pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.plan = PathPlan(
            mission_id="mission_1",
            robot_id="robot_1",
            phase=PathPhase.OUTBOUND,
            version=2,
            positions=(
                Position(x=0, y=0),
                Position(x=1, y=0),
                Position(x=2, y=0),
                Position(x=3, y=0),
                Position(x=4, y=0),
                Position(x=4, y=1),
                Position(x=4, y=2),
            ),
            goal=Position(x=4, y=2),
        )
        self.renderer = PygameRenderer(
            settings=self.settings,
        )

    def tearDown(self) -> None:
        pygame.quit()

    def safe_status(self) -> SafetyStatus:
        """Build a confirmed unlatched safety status."""

        return SafetyStatus(
            robot_id="robot_1",
            updated_at=self.timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )

    def active_status(self) -> SystemStatus:
        """Build an active outbound-navigation status."""

        return SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=BrainState.OUTBOUND_NAVIGATION,
            robot_pose=self.pose,
            safety_status=self.safe_status(),
            mission_id="mission_1",
            mission_status=MissionStatus.ACTIVE,
            active_plan=self.plan,
        )

    def safety_stop_status(self) -> SystemStatus:
        """Build a latched terminal safety status."""

        return SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=BrainState.SAFETY_STOP,
            robot_pose=self.pose,
            safety_status=SafetyStatus(
                robot_id="robot_1",
                updated_at=self.timestamp,
                latched=True,
                severity=SafetySeverity.CRITICAL,
                reason=FailureReason.EMERGENCY_STOP,
            ),
            mission_id="mission_1",
            mission_status=MissionStatus.ABORTED,
            latest_error=FailureReason.EMERGENCY_STOP,
        )

    def event(
        self,
        sequence_number: int,
    ) -> MissionEvent:
        """Build one ordered immutable mission event."""

        return MissionEvent(
            event_id=f"event_{sequence_number}",
            sequence_number=sequence_number,
            mission_id="mission_1",
            robot_id="robot_1",
            occurred_at=(
                self.timestamp
                + timedelta(seconds=sequence_number)
            ),
            source="brain",
            name=f"event_{sequence_number}",
            brain_state=BrainState.OUTBOUND_NAVIGATION,
        )

    def display_surface(self) -> pygame.Surface:
        """Return the initialized dummy display surface."""

        surface = pygame.display.get_surface()

        self.assertIsNotNone(surface)
        assert surface is not None

        return surface

    def cell_center(
        self,
        position: Position,
    ) -> tuple[int, int]:
        """Return the expected center of one grid cell."""

        cell_size = self.settings.cell_size_px
        column = position.x - self.world.origin.x
        maximum_y = (
            self.world.origin.y
            + self.world.height
            - 1
        )
        row_from_top = maximum_y - position.y

        return (
            column * cell_size + cell_size // 2,
            row_from_top * cell_size + cell_size // 2,
        )

    def rgb_at(
        self,
        position: tuple[int, int],
    ) -> tuple[int, int, int]:
        """Return one rendered RGB pixel."""

        color = self.display_surface().get_at(position)

        return (
            color.r,
            color.g,
            color.b,
        )

    def test_active_plan_and_goal_have_distinct_overlays(
        self,
    ) -> None:
        self.renderer.render(
            self.world,
            self.active_status(),
        )

        self.assertEqual(
            self.rgb_at(
                self.cell_center(
                    Position(x=2, y=0)
                )
            ),
            PygameRenderer.PLAN_COLOR,
        )
        self.assertEqual(
            self.rgb_at(
                self.cell_center(self.plan.goal)
            ),
            PygameRenderer.GOAL_COLOR,
        )

    def test_status_lines_expose_confirmed_active_state(
        self,
    ) -> None:
        self.renderer.render(
            self.world,
            self.active_status(),
        )

        self.assertEqual(
            self.renderer.visible_lines[:7],
            (
                "Robot: robot_1",
                "State: OUTBOUND_NAVIGATION",
                "Pose: (0, 0) NORTH",
                "Mission: mission_1 [ACTIVE]",
                "Plan: OUTBOUND v2",
                "Safety: SAFE",
                "Error: none",
            ),
        )

    def test_status_lines_distinguish_latched_safety(
        self,
    ) -> None:
        self.renderer.render(
            self.world,
            self.safety_stop_status(),
        )

        self.assertIn(
            "State: SAFETY_STOP",
            self.renderer.visible_lines,
        )
        self.assertIn(
            "Mission: mission_1 [ABORTED]",
            self.renderer.visible_lines,
        )
        self.assertIn(
            "Plan: none",
            self.renderer.visible_lines,
        )
        self.assertIn(
            "Safety: LATCHED (EMERGENCY_STOP)",
            self.renderer.visible_lines,
        )
        self.assertIn(
            "Error: EMERGENCY_STOP",
            self.renderer.visible_lines,
        )

    def test_recent_event_history_is_bounded(
        self,
    ) -> None:
        for sequence_number in range(1, 6):
            self.renderer.display_event(
                self.event(sequence_number)
            )

        self.assertEqual(
            tuple(
                event.sequence_number
                for event in self.renderer.recent_events
            ),
            (3, 4, 5),
        )

    def test_recent_events_are_included_in_visible_lines(
        self,
    ) -> None:
        for sequence_number in range(1, 6):
            self.renderer.display_event(
                self.event(sequence_number)
            )

        self.renderer.render(
            self.world,
            self.active_status(),
        )

        self.assertIn(
            "Recent events:",
            self.renderer.visible_lines,
        )
        self.assertIn(
            "Event: #3 event_3",
            self.renderer.visible_lines,
        )
        self.assertIn(
            "Event: #4 event_4",
            self.renderer.visible_lines,
        )
        self.assertIn(
            "Event: #5 event_5",
            self.renderer.visible_lines,
        )
        self.assertNotIn(
            "Event: #2 event_2",
            self.renderer.visible_lines,
        )


if __name__ == "__main__":
    unittest.main()
