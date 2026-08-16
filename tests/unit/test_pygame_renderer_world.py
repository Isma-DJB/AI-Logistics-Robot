"""Unit tests for deterministic Pygame world rendering."""

import os
import unittest
from datetime import UTC, datetime

import pygame

from ai_logistics_robot.adapters.visualization import PygameRenderer
from ai_logistics_robot.app.settings import RendererSettings
from ai_logistics_robot.domain.enums import (
    BrainState,
    Heading,
    SafetySeverity,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap


class PygameRendererWorldTests(unittest.TestCase):
    """Verify deterministic world geometry and visual semantics."""

    def setUp(self) -> None:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.quit()

        self.timestamp = datetime(
            2026,
            8,
            16,
            12,
            0,
            tzinfo=UTC,
        )
        self.renderer_settings = RendererSettings(
            enabled=True,
            window_title="Renderer Unit Test",
            cell_size_px=24,
            status_panel_width_px=180,
            frames_per_second=30,
            recent_event_limit=6,
        )
        self.world = GridMap(
            width=5,
            height=4,
            cell_size_cm=20,
            origin=Position(x=-2, y=3),
            base_position=Position(x=-2, y=3),
            target_position=Position(x=2, y=6),
            obstacles=frozenset(
                {
                    Position(x=0, y=5),
                }
            ),
        )
        self.robot_position = Position(x=-1, y=4)
        self.renderer = PygameRenderer(
            settings=self.renderer_settings,
        )

    def tearDown(self) -> None:
        pygame.quit()

    def status(
        self,
        heading: Heading = Heading.NORTH,
    ) -> SystemStatus:
        """Build one valid immutable status snapshot."""

        return SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=BrainState.WAITING_FOR_MISSION,
            robot_pose=RobotPose(
                position=self.robot_position,
                heading=heading,
            ),
            safety_status=SafetyStatus(
                robot_id="robot_1",
                updated_at=self.timestamp,
                latched=False,
                severity=SafetySeverity.INFO,
            ),
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
        """Calculate the expected origin-aware cell center."""

        cell_size = self.renderer_settings.cell_size_px
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

    def heading_tip(
        self,
        heading: Heading,
    ) -> tuple[int, int]:
        """Return the expected heading-indicator endpoint."""

        center_x, center_y = self.cell_center(
            self.robot_position
        )
        cell_size = self.renderer_settings.cell_size_px
        offset = cell_size // 2 - max(
            3,
            cell_size // 8,
        )

        offsets = {
            Heading.NORTH: (0, -offset),
            Heading.EAST: (offset, 0),
            Heading.SOUTH: (0, offset),
            Heading.WEST: (-offset, 0),
        }
        delta_x, delta_y = offsets[heading]

        return (
            center_x + delta_x,
            center_y + delta_y,
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

    def test_window_size_is_derived_from_world_and_settings(
        self,
    ) -> None:
        self.renderer.render(
            self.world,
            self.status(),
        )

        self.assertEqual(
            self.display_surface().get_size(),
            (
                self.world.width
                * self.renderer_settings.cell_size_px
                + self.renderer_settings.status_panel_width_px,
                self.world.height
                * self.renderer_settings.cell_size_px,
            ),
        )

    def test_world_cells_have_distinct_visual_semantics(
        self,
    ) -> None:
        self.renderer.render(
            self.world,
            self.status(),
        )

        expected_cells = (
            (
                Position(x=-2, y=6),
                PygameRenderer.TRAVERSABLE_COLOR,
            ),
            (
                self.world.base_position,
                PygameRenderer.BASE_COLOR,
            ),
            (
                self.world.target_position,
                PygameRenderer.TARGET_COLOR,
            ),
            (
                Position(x=0, y=5),
                PygameRenderer.OBSTACLE_COLOR,
            ),
            (
                Position(x=2, y=5),
                PygameRenderer.ARRIVAL_COLOR,
            ),
            (
                Position(x=1, y=6),
                PygameRenderer.ARRIVAL_COLOR,
            ),
            (
                self.robot_position,
                PygameRenderer.ROBOT_COLOR,
            ),
        )

        for position, expected_color in expected_cells:
            with self.subTest(position=position):
                self.assertEqual(
                    self.rgb_at(
                        self.cell_center(position)
                    ),
                    expected_color,
                )

    def test_shifted_origin_and_increasing_y_are_rendered(
        self,
    ) -> None:
        self.renderer.render(
            self.world,
            self.status(),
        )

        base_center = self.cell_center(
            self.world.base_position
        )
        target_center = self.cell_center(
            self.world.target_position
        )

        self.assertEqual(
            self.rgb_at(base_center),
            PygameRenderer.BASE_COLOR,
        )
        self.assertEqual(
            self.rgb_at(target_center),
            PygameRenderer.TARGET_COLOR,
        )
        self.assertGreater(
            base_center[1],
            target_center[1],
        )

    def test_all_cardinal_headings_are_drawn_deterministically(
        self,
    ) -> None:
        opposites = {
            Heading.NORTH: Heading.SOUTH,
            Heading.EAST: Heading.WEST,
            Heading.SOUTH: Heading.NORTH,
            Heading.WEST: Heading.EAST,
        }

        for heading in Heading:
            with self.subTest(heading=heading):
                self.renderer.render(
                    self.world,
                    self.status(heading),
                )

                self.assertEqual(
                    self.rgb_at(
                        self.heading_tip(heading)
                    ),
                    PygameRenderer.HEADING_COLOR,
                )
                self.assertNotEqual(
                    self.rgb_at(
                        self.heading_tip(
                            opposites[heading]
                        )
                    ),
                    PygameRenderer.HEADING_COLOR,
                )

    def test_repeated_rendering_produces_identical_pixels(
        self,
    ) -> None:
        status = self.status(Heading.WEST)

        self.renderer.render(
            self.world,
            status,
        )
        first_pixels = pygame.image.tostring(
            self.display_surface(),
            "RGB",
        )

        self.renderer.render(
            self.world,
            status,
        )
        second_pixels = pygame.image.tostring(
            self.display_surface(),
            "RGB",
        )

        self.assertEqual(
            first_pixels,
            second_pixels,
        )


if __name__ == "__main__":
    unittest.main()
