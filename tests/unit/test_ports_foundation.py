"""Unit tests for Clock, Perception, and Planning ports."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.domain.enums import Heading, PathPhase
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports.clock_port import ClockPort
from ai_logistics_robot.ports.perception_port import PerceptionPort
from ai_logistics_robot.ports.planning_port import PlanningPort


class CompatibleClock:
    """Minimal structural implementation of ClockPort."""

    def __init__(self) -> None:
        self.deadline: float | None = None

    def now(self) -> datetime:
        """Return deterministic wall-clock time."""

        return datetime(2026, 8, 12, tzinfo=UTC)

    def monotonic(self) -> float:
        """Return deterministic monotonic time."""

        return 10.0

    def wait_until(self, deadline: float) -> None:
        """Record the requested deadline without actually waiting."""

        self.deadline = deadline


class IncompleteClock:
    """Object intentionally missing two ClockPort operations."""

    def now(self) -> datetime:
        """Return a deterministic timestamp."""

        return datetime(2026, 8, 12, tzinfo=UTC)


class CompatiblePerception:
    """Minimal structural implementation of PerceptionPort."""

    def __init__(self, snapshot: PerceptionSnapshot) -> None:
        self.snapshot = snapshot

    def observe(self) -> PerceptionSnapshot:
        """Return the configured snapshot."""

        return self.snapshot


class IncompletePerception:
    """Object intentionally missing observe()."""


class CompatiblePlanning:
    """Minimal structural implementation of PlanningPort."""

    def create_plan(
        self,
        *,
        mission_id: str,
        robot_id: str,
        start_pose: RobotPose,
        authorized_goals: tuple[Position, ...],
        world: GridMap,
        phase: PathPhase,
        version: int,
    ) -> PathPlan:
        """Return a deterministic path to the first authorized goal."""

        goal = authorized_goals[0]

        return PathPlan(
            mission_id=mission_id,
            robot_id=robot_id,
            phase=phase,
            version=version,
            positions=(start_pose.position, goal),
            goal=goal,
        )


class IncompletePlanning:
    """Object intentionally missing create_plan()."""


class FoundationPortTests(unittest.TestCase):
    """Verify structural compatibility and typed results."""

    def setUp(self) -> None:
        self.pose = RobotPose(
            position=Position(x=1, y=1),
            heading=Heading.NORTH,
        )
        self.world = GridMap(
            width=10,
            height=10,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=Position(x=8, y=7),
        )

    def test_compatible_clock_satisfies_runtime_protocol(self) -> None:
        clock = CompatibleClock()

        self.assertIsInstance(clock, ClockPort)
        self.assertEqual(clock.monotonic(), 10.0)
        self.assertIsNotNone(clock.now().utcoffset())

        clock.wait_until(12.5)

        self.assertEqual(clock.deadline, 12.5)

    def test_incomplete_clock_is_rejected(self) -> None:
        self.assertNotIsInstance(IncompleteClock(), ClockPort)

    def test_compatible_perception_satisfies_runtime_protocol(
        self,
    ) -> None:
        snapshot = PerceptionSnapshot(
            robot_id="robot_1",
            captured_at=datetime(2026, 8, 12, tzinfo=UTC),
            robot_pose=self.pose,
            observations=(),
            target_active=False,
            hazard_detected=False,
        )
        perception = CompatiblePerception(snapshot)

        self.assertIsInstance(perception, PerceptionPort)
        self.assertIs(perception.observe(), snapshot)

    def test_incomplete_perception_is_rejected(self) -> None:
        self.assertNotIsInstance(
            IncompletePerception(),
            PerceptionPort,
        )

    def test_compatible_planning_satisfies_runtime_protocol(
        self,
    ) -> None:
        planning = CompatiblePlanning()
        authorized_goals = (
            Position(x=8, y=8),
            Position(x=9, y=7),
        )

        self.assertIsInstance(planning, PlanningPort)

        plan = planning.create_plan(
            mission_id="mission_1",
            robot_id="robot_1",
            start_pose=self.pose,
            authorized_goals=authorized_goals,
            world=self.world,
            phase=PathPhase.OUTBOUND,
            version=1,
        )

        self.assertEqual(plan.goal, authorized_goals[0])
        self.assertEqual(plan.positions[0], self.pose.position)
        self.assertEqual(plan.version, 1)

    def test_incomplete_planning_is_rejected(self) -> None:
        self.assertNotIsInstance(
            IncompletePlanning(),
            PlanningPort,
        )


if __name__ == "__main__":
    unittest.main()