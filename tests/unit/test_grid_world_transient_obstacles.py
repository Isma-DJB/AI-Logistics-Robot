"""Unit tests for deterministic transient GridWorld obstacles."""

import unittest

from ai_logistics_robot.adapters.simulation import GridWorld
from ai_logistics_robot.domain.commands import MotionCommand
from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    FailureReason,
    Heading,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap


class GridWorldTransientObstacleTests(unittest.TestCase):
    """Verify scenario-controlled unplanned movement rejection."""

    def setUp(self) -> None:
        self.initial_pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.configured_obstacle = Position(
            x=2,
            y=2,
        )
        self.world = GridMap(
            width=4,
            height=4,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=self.initial_pose.position,
            target_position=Position(x=3, y=3),
            obstacles=frozenset(
                {
                    self.configured_obstacle,
                }
            ),
        )
        self.simulation = GridWorld(
            world=self.world,
            robot_id="robot_1",
            initial_pose=self.initial_pose,
        )
        self.forward = MotionCommand(
            robot_id="robot_1",
            command_type=CommandType.MOVE_FORWARD,
        )

    def test_default_transient_state_preserves_configured_world(
        self,
    ) -> None:
        self.assertEqual(
            self.simulation.transient_obstacles,
            frozenset(),
        )
        self.assertIs(
            self.simulation.read_world(),
            self.world,
        )
        self.assertEqual(
            self.world.obstacles,
            frozenset(
                {
                    self.configured_obstacle,
                }
            ),
        )

    def test_transient_obstacle_is_visible_without_mutating_config(
        self,
    ) -> None:
        transient = Position(x=0, y=1)

        self.simulation.set_transient_obstacles(
            frozenset({transient})
        )

        visible_world = self.simulation.read_world()

        self.assertEqual(
            self.simulation.transient_obstacles,
            frozenset({transient}),
        )
        self.assertIsNot(
            visible_world,
            self.world,
        )
        self.assertEqual(
            visible_world.obstacles,
            frozenset(
                {
                    self.configured_obstacle,
                    transient,
                }
            ),
        )
        self.assertEqual(
            self.world.obstacles,
            frozenset(
                {
                    self.configured_obstacle,
                }
            ),
        )

    def test_transient_obstacle_rejects_forward_atomically(
        self,
    ) -> None:
        transient = Position(x=0, y=1)
        pose_before = self.simulation.current_pose

        self.simulation.set_transient_obstacles(
            frozenset({transient})
        )

        result = self.simulation.apply_command(
            self.forward
        )

        self.assertIs(
            result.status,
            CommandStatus.FAILED,
        )
        self.assertIs(
            result.failure_reason,
            FailureReason.BLOCKED,
        )
        self.assertIs(
            result.pose_before,
            pose_before,
        )
        self.assertIs(
            result.pose_after,
            pose_before,
        )
        self.assertIs(
            self.simulation.current_pose,
            pose_before,
        )

    def test_replacing_transient_obstacles_is_deterministic(
        self,
    ) -> None:
        first = Position(x=0, y=1)
        second = Position(x=1, y=0)

        self.simulation.set_transient_obstacles(
            frozenset({first})
        )
        self.simulation.set_transient_obstacles(
            frozenset({second})
        )

        self.assertEqual(
            self.simulation.transient_obstacles,
            frozenset({second}),
        )
        self.assertNotIn(
            first,
            self.simulation.read_world().obstacles,
        )
        self.assertIn(
            second,
            self.simulation.read_world().obstacles,
        )

    def test_reset_clears_transient_obstacles(
        self,
    ) -> None:
        transient = Position(x=0, y=1)

        self.simulation.set_transient_obstacles(
            frozenset({transient})
        )
        blocked = self.simulation.apply_command(
            self.forward
        )

        self.assertIs(
            blocked.status,
            CommandStatus.FAILED,
        )

        self.simulation.advance_time(2.0)
        self.simulation.reset()

        self.assertEqual(
            self.simulation.transient_obstacles,
            frozenset(),
        )
        self.assertIs(
            self.simulation.read_world(),
            self.world,
        )
        self.assertIs(
            self.simulation.current_pose,
            self.initial_pose,
        )
        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            0.0,
        )

        accepted = self.simulation.apply_command(
            self.forward
        )

        self.assertIs(
            accepted.status,
            CommandStatus.SUCCESS,
        )
        self.assertEqual(
            accepted.pose_after.position,
            transient,
        )

    def test_invalid_transient_sets_are_rejected_atomically(
        self,
    ) -> None:
        accepted = frozenset(
            {
                Position(x=0, y=1),
            }
        )

        self.simulation.set_transient_obstacles(
            accepted
        )

        invalid_values = (
            [],
            (Position(x=1, y=0),),
            frozenset({object()}),
            frozenset(
                {
                    self.initial_pose.position,
                }
            ),
            frozenset(
                {
                    Position(x=-1, y=0),
                }
            ),
            frozenset(
                {
                    self.configured_obstacle,
                }
            ),
        )

        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.simulation.set_transient_obstacles(
                        value  # type: ignore[arg-type]
                    )

                self.assertEqual(
                    self.simulation.transient_obstacles,
                    accepted,
                )


if __name__ == "__main__":
    unittest.main()
