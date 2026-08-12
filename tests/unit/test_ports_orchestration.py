"""Unit tests for Brain, Renderer, and public port exports."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot import ports
from ai_logistics_robot.domain.enums import (
    BrainState,
    Heading,
    SafetySeverity,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap
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


class CompatibleBrain:
    """Minimal structural implementation of BrainPort."""

    def __init__(self, status: SystemStatus) -> None:
        self.status = status
        self.update_count = 0
        self.reset_called = False

    def update(self) -> None:
        """Record one deterministic orchestration cycle."""

        self.update_count += 1

    def get_status(self) -> SystemStatus:
        """Return the configured read-only status."""

        return self.status

    def reset(self) -> None:
        """Record restoration of the initial brain state."""

        self.update_count = 0
        self.reset_called = True


class IncompleteBrain:
    """Object intentionally missing BrainPort operations."""


class CompatibleRenderer:
    """Minimal structural implementation of RendererPort."""

    def __init__(self) -> None:
        self.render_calls: list[tuple[GridMap, SystemStatus]] = []
        self.events: list[MissionEvent] = []

    def render(
        self,
        world: GridMap,
        status: SystemStatus,
    ) -> None:
        """Record one immutable render request."""

        self.render_calls.append((world, status))

    def display_event(self, event: MissionEvent) -> None:
        """Record one immutable event-display request."""

        self.events.append(event)


class IncompleteRenderer:
    """Object intentionally missing RendererPort operations."""


class OrchestrationPortTests(unittest.TestCase):
    """Verify orchestration ports and their public exports."""

    def setUp(self) -> None:
        self.observed_at = datetime(2026, 8, 12, tzinfo=UTC)
        self.brain_state = next(iter(BrainState))
        self.pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.safety_status = SafetyStatus(
            robot_id="robot_1",
            updated_at=self.observed_at,
            latched=False,
            severity=SafetySeverity.INFO,
        )
        self.status = SystemStatus(
            robot_id="robot_1",
            observed_at=self.observed_at,
            brain_state=self.brain_state,
            robot_pose=self.pose,
            safety_status=self.safety_status,
        )
        self.world = GridMap(
            width=10,
            height=10,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=Position(x=8, y=7),
        )
        self.event = MissionEvent(
            event_id="event_1",
            sequence_number=1,
            mission_id="mission_1",
            robot_id="robot_1",
            occurred_at=self.observed_at,
            source="brain",
            name="cycle_completed",
            brain_state=self.brain_state,
        )

    def test_brain_updates_and_returns_read_only_status(self) -> None:
        brain = CompatibleBrain(self.status)

        self.assertIsInstance(brain, BrainPort)

        brain.update()

        self.assertEqual(brain.update_count, 1)
        self.assertIs(brain.get_status(), self.status)

    def test_brain_resets_deterministic_cycle_state(self) -> None:
        brain = CompatibleBrain(self.status)
        brain.update()
        brain.reset()

        self.assertTrue(brain.reset_called)
        self.assertEqual(brain.update_count, 0)

    def test_incomplete_brain_is_rejected(self) -> None:
        self.assertNotIsInstance(IncompleteBrain(), BrainPort)

    def test_renderer_receives_state_and_event_without_control(self) -> None:
        renderer = CompatibleRenderer()

        self.assertIsInstance(renderer, RendererPort)

        renderer.render(self.world, self.status)
        renderer.display_event(self.event)

        self.assertEqual(
            renderer.render_calls,
            [(self.world, self.status)],
        )
        self.assertEqual(renderer.events, [self.event])

    def test_incomplete_renderer_is_rejected(self) -> None:
        self.assertNotIsInstance(
            IncompleteRenderer(),
            RendererPort,
        )

    def test_all_nine_ports_are_publicly_exported(self) -> None:
        expected_exports = {
            "BrainPort": BrainPort,
            "ClockPort": ClockPort,
            "ControlPort": ControlPort,
            "MemoryPort": MemoryPort,
            "MonitoringPort": MonitoringPort,
            "PerceptionPort": PerceptionPort,
            "PlanningPort": PlanningPort,
            "RendererPort": RendererPort,
            "SimulationPort": SimulationPort,
        }

        self.assertEqual(set(ports.__all__), set(expected_exports))

        for name, contract in expected_exports.items():
            self.assertIs(getattr(ports, name), contract)


if __name__ == "__main__":
    unittest.main()