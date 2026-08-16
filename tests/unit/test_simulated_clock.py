"""Unit tests for the deterministic GridWorld clock."""

import unittest
from datetime import UTC, datetime, timedelta
from math import inf, nan

from ai_logistics_robot.adapters.simulation import (
    GridWorld,
    SimulatedClock,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports import ClockPort


class SimulatedClockTests(unittest.TestCase):
    """Verify one clock backed by confirmed GridWorld time."""

    def setUp(self) -> None:
        self.epoch = datetime(
            2026,
            8,
            16,
            8,
            0,
            tzinfo=UTC,
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
        self.simulation = GridWorld(
            world=self.world,
            robot_id="robot_1",
            initial_pose=RobotPose(
                position=Position(x=0, y=0),
                heading=Heading.NORTH,
            ),
        )
        self.clock = SimulatedClock(
            simulation=self.simulation,
            epoch=self.epoch,
        )

    def test_clock_satisfies_public_port(self) -> None:
        self.assertIsInstance(
            self.clock,
            ClockPort,
        )

    def test_initial_time_comes_from_epoch_and_grid_world(
        self,
    ) -> None:
        self.assertEqual(
            self.clock.now(),
            self.epoch,
        )
        self.assertEqual(
            self.clock.monotonic(),
            0.0,
        )

    def test_wait_until_advances_only_required_duration(
        self,
    ) -> None:
        self.clock.wait_until(2.5)

        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            2.5,
        )
        self.assertEqual(
            self.clock.monotonic(),
            2.5,
        )
        self.assertEqual(
            self.clock.now(),
            self.epoch + timedelta(seconds=2.5),
        )

        self.clock.wait_until(2.5)

        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            2.5,
        )

    def test_direct_simulation_time_is_reflected(
        self,
    ) -> None:
        self.simulation.advance_time(1.25)

        self.assertEqual(
            self.clock.monotonic(),
            1.25,
        )
        self.assertEqual(
            self.clock.now(),
            self.epoch + timedelta(seconds=1.25),
        )

    def test_grid_world_reset_restores_clock_epoch(
        self,
    ) -> None:
        self.clock.wait_until(4.0)

        self.simulation.reset()

        self.assertEqual(
            self.clock.monotonic(),
            0.0,
        )
        self.assertEqual(
            self.clock.now(),
            self.epoch,
        )

    def test_backward_and_invalid_deadlines_are_rejected(
        self,
    ) -> None:
        self.clock.wait_until(2.0)

        invalid_deadlines = (
            1.0,
            -1.0,
            True,
            "3.0",
            None,
            inf,
            nan,
        )

        for deadline in invalid_deadlines:
            with self.subTest(deadline=deadline):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.clock.wait_until(  # type: ignore[arg-type]
                        deadline
                    )

        self.assertEqual(
            self.clock.monotonic(),
            2.0,
        )

    def test_constructor_rejects_invalid_dependencies(
        self,
    ) -> None:
        with self.assertRaises(DomainValidationError):
            SimulatedClock(
                simulation=None,  # type: ignore[arg-type]
                epoch=self.epoch,
            )

        invalid_epochs = (
            datetime(2026, 8, 16, 8, 0),
            "2026-08-16",
            None,
        )

        for epoch in invalid_epochs:
            with self.subTest(epoch=epoch):
                with self.assertRaises(
                    DomainValidationError
                ):
                    SimulatedClock(
                        simulation=self.simulation,
                        epoch=epoch,  # type: ignore[arg-type]
                    )


if __name__ == "__main__":
    unittest.main()
