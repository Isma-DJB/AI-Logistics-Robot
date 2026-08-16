"""Passive deterministic Pygame visualization adapter."""

from collections import deque
from typing import ClassVar

import pygame

from ai_logistics_robot.app.settings import RendererSettings
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap

Color = tuple[int, int, int]


class PygameRenderer:
    """Render immutable world state without influencing control."""

    BACKGROUND_COLOR: ClassVar[Color] = (16, 22, 30)
    PANEL_COLOR: ClassVar[Color] = (25, 34, 45)
    TRAVERSABLE_COLOR: ClassVar[Color] = (220, 227, 233)
    GRID_COLOR: ClassVar[Color] = (75, 86, 98)
    OBSTACLE_COLOR: ClassVar[Color] = (55, 62, 70)
    BASE_COLOR: ClassVar[Color] = (67, 160, 71)
    TARGET_COLOR: ClassVar[Color] = (211, 84, 80)
    ARRIVAL_COLOR: ClassVar[Color] = (240, 190, 70)
    ROBOT_COLOR: ClassVar[Color] = (38, 120, 210)
    HEADING_COLOR: ClassVar[Color] = (255, 255, 255)
    PLAN_COLOR: ClassVar[Color] = (145, 100, 215)
    GOAL_COLOR: ClassVar[Color] = (255, 145, 45)
    TEXT_COLOR: ClassVar[Color] = (238, 242, 246)
    MUTED_TEXT_COLOR: ClassVar[Color] = (165, 178, 192)
    ALERT_TEXT_COLOR: ClassVar[Color] = (255, 105, 100)

    def __init__(
        self,
        *,
        settings: RendererSettings,
    ) -> None:
        """Store validated settings without opening a window."""

        if not isinstance(settings, RendererSettings):
            raise DomainValidationError(
                "settings must be a RendererSettings instance."
            )

        self._settings = settings
        self._surface: pygame.Surface | None = None
        self._clock: pygame.time.Clock | None = None
        self._font: pygame.font.Font | None = None
        self._recent_events: deque[MissionEvent] = deque(
            maxlen=settings.recent_event_limit
        )
        self._visible_lines: tuple[str, ...] = ()
        self._closed = False

    @property
    def closed(self) -> bool:
        """Report whether visualization has been closed."""

        return self._closed

    @property
    def recent_events(self) -> tuple[MissionEvent, ...]:
        """Return the bounded immutable event view."""

        return tuple(self._recent_events)

    @property
    def visible_lines(self) -> tuple[str, ...]:
        """Return the text prepared for the latest rendered panel."""

        return self._visible_lines

    def render(
        self,
        world: GridMap,
        status: SystemStatus,
    ) -> None:
        """Render one immutable world and status snapshot."""

        if not isinstance(world, GridMap):
            raise DomainValidationError(
                "world must be a GridMap instance."
            )

        if not isinstance(status, SystemStatus):
            raise DomainValidationError(
                "status must be a SystemStatus instance."
            )

        if not self._settings.enabled or self._closed:
            return

        self._initialize_display(world)

        if self._process_events():
            return

        surface = self._require_surface()
        grid_width = (
            world.width
            * self._settings.cell_size_px
        )
        window_height = (
            world.height
            * self._settings.cell_size_px
        )

        surface.fill(self.BACKGROUND_COLOR)

        pygame.draw.rect(
            surface,
            self.PANEL_COLOR,
            pygame.Rect(
                grid_width,
                0,
                self._settings.status_panel_width_px,
                window_height,
            ),
        )

        self._draw_world(
            surface,
            world,
        )
        self._draw_active_plan(
            surface,
            world,
            status,
        )
        self._draw_robot(
            surface,
            world,
            status,
        )
        self._draw_status_panel(
            surface,
            world,
            status,
        )

        pygame.display.flip()

        if self._clock is not None:
            self._clock.tick(
                self._settings.frames_per_second
            )

    def display_event(
        self,
        event: MissionEvent,
    ) -> None:
        """Accept one immutable event without changing control state."""

        if not isinstance(event, MissionEvent):
            raise DomainValidationError(
                "event must be a MissionEvent instance."
            )

        if not self._settings.enabled or self._closed:
            return

        self._recent_events.append(event)

    def close(self) -> None:
        """Close visualization resources idempotently."""

        if self._closed:
            return

        if pygame.display.get_init():
            pygame.display.quit()

        self._surface = None
        self._clock = None
        self._font = None
        self._closed = True

    def _initialize_display(
        self,
        world: GridMap,
    ) -> None:
        """Create or resize the display for the current world."""

        expected_size = (
            world.width
            * self._settings.cell_size_px
            + self._settings.status_panel_width_px,
            world.height
            * self._settings.cell_size_px,
        )

        if not pygame.display.get_init():
            pygame.display.init()

        if not pygame.font.get_init():
            pygame.font.init()

        current_surface = pygame.display.get_surface()

        if (
            current_surface is None
            or current_surface.get_size() != expected_size
        ):
            self._surface = pygame.display.set_mode(
                expected_size
            )
        else:
            self._surface = current_surface

        pygame.display.set_caption(
            self._settings.window_title
        )

        if self._clock is None:
            self._clock = pygame.time.Clock()

        if self._font is None:
            font_size = max(
                14,
                min(
                    22,
                    self._settings.cell_size_px // 2,
                ),
            )
            self._font = pygame.font.Font(
                None,
                font_size,
            )

    def _process_events(self) -> bool:
        """Close only the renderer when SDL requests termination."""

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.close()
                return True

        return False

    def _require_surface(self) -> pygame.Surface:
        """Return the initialized display surface."""

        if self._surface is None:
            raise RuntimeError(
                "the renderer display is not initialized."
            )

        return self._surface

    def _draw_world(
        self,
        surface: pygame.Surface,
        world: GridMap,
    ) -> None:
        """Draw grid cells and their confirmed semantics."""

        arrival_positions = frozenset(
            world.authorized_arrival_positions
        )

        for y_coordinate in range(
            world.origin.y,
            world.origin.y + world.height,
        ):
            for x_coordinate in range(
                world.origin.x,
                world.origin.x + world.width,
            ):
                position = Position(
                    x=x_coordinate,
                    y=y_coordinate,
                )
                rectangle = self._cell_rectangle(
                    world,
                    position,
                )

                pygame.draw.rect(
                    surface,
                    self._cell_color(
                        world,
                        position,
                        arrival_positions,
                    ),
                    rectangle,
                )
                pygame.draw.rect(
                    surface,
                    self.GRID_COLOR,
                    rectangle,
                    width=1,
                )

    def _cell_color(
        self,
        world: GridMap,
        position: Position,
        arrival_positions: frozenset[Position],
    ) -> Color:
        """Return the visual color for one domain cell."""

        if world.is_obstacle(position):
            return self.OBSTACLE_COLOR

        if position == world.target_position:
            return self.TARGET_COLOR

        if position == world.base_position:
            return self.BASE_COLOR

        if position in arrival_positions:
            return self.ARRIVAL_COLOR

        return self.TRAVERSABLE_COLOR

    def _cell_rectangle(
        self,
        world: GridMap,
        position: Position,
    ) -> pygame.Rect:
        """Convert one domain position into a screen rectangle."""

        cell_size = self._settings.cell_size_px
        column = position.x - world.origin.x
        maximum_y = (
            world.origin.y
            + world.height
            - 1
        )
        row_from_top = maximum_y - position.y

        return pygame.Rect(
            column * cell_size,
            row_from_top * cell_size,
            cell_size,
            cell_size,
        )

    def _draw_active_plan(
        self,
        surface: pygame.Surface,
        world: GridMap,
        status: SystemStatus,
    ) -> None:
        """Draw the informational active-plan overlay."""

        plan = status.active_plan

        if plan is None:
            return

        points = tuple(
            self._cell_rectangle(
                world,
                position,
            ).center
            for position in plan.positions
        )
        line_width = max(
            2,
            self._settings.cell_size_px // 10,
        )
        marker_radius = max(
            2,
            self._settings.cell_size_px // 8,
        )
        goal_radius = max(
            3,
            self._settings.cell_size_px // 6,
        )

        if len(points) > 1:
            pygame.draw.lines(
                surface,
                self.PLAN_COLOR,
                False,
                points,
                width=line_width,
            )

        for point in points:
            pygame.draw.circle(
                surface,
                self.PLAN_COLOR,
                point,
                marker_radius,
            )

        pygame.draw.circle(
            surface,
            self.GOAL_COLOR,
            self._cell_rectangle(
                world,
                plan.goal,
            ).center,
            goal_radius,
        )

    def _draw_robot(
        self,
        surface: pygame.Surface,
        world: GridMap,
        status: SystemStatus,
    ) -> None:
        """Draw confirmed robot position and heading."""

        rectangle = self._cell_rectangle(
            world,
            status.robot_pose.position,
        )
        center = rectangle.center
        tip = self._heading_tip(
            rectangle,
            status.robot_pose.heading,
        )
        line_width = max(
            2,
            self._settings.cell_size_px // 10,
        )
        robot_radius = max(
            3,
            self._settings.cell_size_px // 4,
        )

        pygame.draw.line(
            surface,
            self.HEADING_COLOR,
            center,
            tip,
            width=line_width,
        )
        pygame.draw.circle(
            surface,
            self.ROBOT_COLOR,
            center,
            robot_radius,
        )

    def _draw_status_panel(
        self,
        surface: pygame.Surface,
        world: GridMap,
        status: SystemStatus,
    ) -> None:
        """Draw confirmed status and recent event text."""

        status_lines = self._status_lines(status)
        event_lines = self._event_lines()

        self._visible_lines = (
            status_lines
            + ("",)
            + event_lines
        )

        font = self._require_font()
        padding = max(
            8,
            self._settings.cell_size_px // 4,
        )
        left = (
            world.width
            * self._settings.cell_size_px
            + padding
        )
        top = padding
        line_height = font.get_linesize()

        for line in self._visible_lines:
            if line:
                text_surface = font.render(
                    line,
                    True,
                    self._line_color(line),
                )
                surface.blit(
                    text_surface,
                    (left, top),
                )

            top += line_height

    def _status_lines(
        self,
        status: SystemStatus,
    ) -> tuple[str, ...]:
        """Build deterministic confirmed-status text."""

        pose = status.robot_pose

        if (
            status.mission_id is None
            or status.mission_status is None
        ):
            mission_line = "Mission: none"
        else:
            mission_line = (
                f"Mission: {status.mission_id} "
                f"[{status.mission_status.value}]"
            )

        if status.active_plan is None:
            plan_line = "Plan: none"
        else:
            plan_line = (
                f"Plan: {status.active_plan.phase.value} "
                f"v{status.active_plan.version}"
            )

        if status.safety_status.latched:
            reason = status.safety_status.reason
            reason_text = (
                reason.value
                if reason is not None
                else "UNKNOWN"
            )
            safety_line = (
                f"Safety: LATCHED ({reason_text})"
            )
        else:
            safety_line = "Safety: SAFE"

        error_line = (
            "Error: none"
            if status.latest_error is None
            else f"Error: {status.latest_error.value}"
        )

        return (
            f"Robot: {status.robot_id}",
            f"State: {status.brain_state.value}",
            (
                f"Pose: ({pose.position.x}, "
                f"{pose.position.y}) "
                f"{pose.heading.value}"
            ),
            mission_line,
            plan_line,
            safety_line,
            error_line,
        )

    def _event_lines(self) -> tuple[str, ...]:
        """Build deterministic recent-event text."""

        if not self._recent_events:
            return (
                "Recent events:",
                "Event: none",
            )

        return (
            "Recent events:",
            *(
                (
                    f"Event: #{event.sequence_number} "
                    f"{event.name}"
                )
                for event in self._recent_events
            ),
        )

    def _line_color(
        self,
        line: str,
    ) -> Color:
        """Select a passive text color by displayed severity."""

        if line.startswith("Safety: LATCHED"):
            return self.ALERT_TEXT_COLOR

        if (
            line.startswith("Error:")
            and line != "Error: none"
        ):
            return self.ALERT_TEXT_COLOR

        if line == "Recent events:":
            return self.MUTED_TEXT_COLOR

        return self.TEXT_COLOR

    def _require_font(self) -> pygame.font.Font:
        """Return the initialized renderer font."""

        if self._font is None:
            raise RuntimeError(
                "the renderer font is not initialized."
            )

        return self._font

    def _heading_tip(
        self,
        rectangle: pygame.Rect,
        heading: Heading,
    ) -> tuple[int, int]:
        """Calculate the endpoint of a cardinal heading marker."""

        center_x, center_y = rectangle.center
        offset = (
            self._settings.cell_size_px // 2
            - max(
                3,
                self._settings.cell_size_px // 8,
            )
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
