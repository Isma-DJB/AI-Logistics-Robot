"""Unit tests for the passive headless renderer."""

import subprocess
import sys
import unittest
from datetime import UTC, datetime

from ai_logistics_robot.adapters.simulation import (
    HeadlessRenderer,
)
from ai_logistics_robot.domain.enums import (
    BrainState,
    Heading,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports import RendererPort


class HeadlessRendererTests(unittest.TestCase):
    """Verify passive rendering without SDL or Pygame."""

    def setUp(self) -> None:
        self.timestamp = datetime(
            2026,
            8,
            16,
            11,
            0,
            tzinfo=UTC,
        )
        self.pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.world = GridMap(
            width=3,
            height=3,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=0, y=0),
            target_position=Position(x=2, y=2),
            obstacles=frozenset(),
        )
        self.status = SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=BrainState.WAITING_FOR_MISSION,
            robot_pose=self.pose,
            safety_status=SafetyStatus(
                robot_id="robot_1",
                updated_at=self.timestamp,
                latched=False,
                severity=SafetySeverity.INFO,
            ),
        )
        self.event = MissionEvent(
            event_id="event_1",
            sequence_number=1,
            mission_id="mission_1",
            robot_id="robot_1",
            occurred_at=self.timestamp,
            source="brain",
            name="mission_started",
            brain_state=BrainState.OUTBOUND_PLANNING,
        )
        self.renderer = HeadlessRenderer()

    def test_renderer_satisfies_public_port(self) -> None:
        self.assertIsInstance(
            self.renderer,
            RendererPort,
        )

    def test_rendered_frames_preserve_snapshot_identity(
        self,
    ) -> None:
        self.renderer.render(
            self.world,
            self.status,
        )
        self.renderer.render(
            self.world,
            self.status,
        )

        self.assertEqual(
            self.renderer.rendered_frames,
            (
                (self.world, self.status),
                (self.world, self.status),
            ),
        )
        self.assertIs(
            self.renderer.rendered_frames[0][0],
            self.world,
        )
        self.assertIs(
            self.renderer.rendered_frames[0][1],
            self.status,
        )

    def test_displayed_events_preserve_identity_and_order(
        self,
    ) -> None:
        self.renderer.display_event(self.event)
        self.renderer.display_event(self.event)

        self.assertEqual(
            self.renderer.displayed_events,
            (self.event, self.event),
        )
        self.assertIs(
            self.renderer.displayed_events[0],
            self.event,
        )

    def test_renderer_rejects_invalid_public_inputs(
        self,
    ) -> None:
        with self.assertRaises(DomainValidationError):
            self.renderer.render(  # type: ignore[arg-type]
                None,
                self.status,
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

        self.assertEqual(
            self.renderer.rendered_frames,
            (),
        )
        self.assertEqual(
            self.renderer.displayed_events,
            (),
        )

    def test_headless_import_does_not_import_pygame(
        self,
    ) -> None:
        script = (
            "import sys; "
            "from ai_logistics_robot.adapters.simulation "
            "import HeadlessRenderer; "
            "HeadlessRenderer(); "
            "assert 'pygame' not in sys.modules"
        )

        result = subprocess.run(
            [
                sys.executable,
                "-c",
                script,
            ],
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
