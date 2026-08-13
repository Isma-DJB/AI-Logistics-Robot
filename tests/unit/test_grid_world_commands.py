"""Unit tests for normal GridWorld command execution."""

import unittest

from ai_logistics_robot.adapters.simulation.grid_world import GridWorld
from ai_logistics_robot.domain.commands import MotionCommand
from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    Heading,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap


class GridWorldCommandTests(unittest.TestCase):
    """Verify successful stop, rotation, and movement commands."""

    def setUp(self) -> None:
        self.robot_id = "robot_1"
        self.world = GridMap(
            width=7,
            height=7,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=3, y=3),
            target_position=Position(x=6, y=6),
            obstacles=frozenset({Position(x=1, y=1)}),
        )

    def build_simulation(
        self,
        heading: Heading = Heading.NORTH,
    ) -> GridWorld:
        """Build a simulation at the safe center of the test grid."""

        return GridWorld(
            world=self.world,
            robot_id=self.robot_id,
            initial_pose=RobotPose(
                position=Position(x=3, y=3),
                heading=heading,
            ),
        )

    def command(self, command_type: CommandType) -> MotionCommand:
        """Build one command for the configured robot."""

        return MotionCommand(
            robot_id=self.robot_id,
            command_type=command_type,
        )

    def test_stop_succeeds_without_changing_pose(self) -> None:
        simulation = self.build_simulation()
        pose_before = simulation.current_pose
        command = self.command(CommandType.STOP)

        result = simulation.apply_command(command)

        self.assertIs(result.status, CommandStatus.SUCCESS)
        self.assertEqual(result.pose_before, pose_before)
        self.assertEqual(result.pose_after, pose_before)
        self.assertIsNone(result.failure_reason)
        self.assertEqual(simulation.current_pose, pose_before)

    def test_left_turn_follows_cardinal_cycle(self) -> None:
        simulation = self.build_simulation()
        command = self.command(CommandType.TURN_LEFT)
        expected_headings = (
            Heading.WEST,
            Heading.SOUTH,
            Heading.EAST,
            Heading.NORTH,
        )

        for expected_heading in expected_headings:
            pose_before = simulation.current_pose

            result = simulation.apply_command(command)

            self.assertIs(result.status, CommandStatus.SUCCESS)
            self.assertEqual(result.pose_before, pose_before)
            self.assertEqual(
                result.pose_after.position,
                pose_before.position,
            )
            self.assertIs(
                result.pose_after.heading,
                expected_heading,
            )
            self.assertEqual(
                simulation.current_pose,
                result.pose_after,
            )

    def test_right_turn_follows_cardinal_cycle(self) -> None:
        simulation = self.build_simulation()
        command = self.command(CommandType.TURN_RIGHT)
        expected_headings = (
            Heading.EAST,
            Heading.SOUTH,
            Heading.WEST,
            Heading.NORTH,
        )

        for expected_heading in expected_headings:
            pose_before = simulation.current_pose

            result = simulation.apply_command(command)

            self.assertIs(result.status, CommandStatus.SUCCESS)
            self.assertEqual(result.pose_before, pose_before)
            self.assertEqual(
                result.pose_after.position,
                pose_before.position,
            )
            self.assertIs(
                result.pose_after.heading,
                expected_heading,
            )
            self.assertEqual(
                simulation.current_pose,
                result.pose_after,
            )

    def test_forward_movement_supports_every_heading(self) -> None:
        cases = (
            (Heading.NORTH, Position(x=3, y=4)),
            (Heading.EAST, Position(x=4, y=3)),
            (Heading.SOUTH, Position(x=3, y=2)),
            (Heading.WEST, Position(x=2, y=3)),
        )

        for heading, expected_position in cases:
            with self.subTest(heading=heading):
                simulation = self.build_simulation(heading)
                command = self.command(
                    CommandType.MOVE_FORWARD
                )

                result = simulation.apply_command(command)

                self.assertIs(
                    result.status,
                    CommandStatus.SUCCESS,
                )
                self.assertEqual(
                    result.pose_after,
                    RobotPose(
                        position=expected_position,
                        heading=heading,
                    ),
                )
                self.assertEqual(
                    simulation.current_pose,
                    result.pose_after,
                )

    def test_commands_use_latest_confirmed_pose(self) -> None:
        simulation = self.build_simulation()

        turn_result = simulation.apply_command(
            self.command(CommandType.TURN_RIGHT)
        )
        move_result = simulation.apply_command(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertEqual(
            move_result.pose_before,
            turn_result.pose_after,
        )
        self.assertEqual(
            move_result.pose_after,
            RobotPose(
                position=Position(x=4, y=3),
                heading=Heading.EAST,
            ),
        )

    def test_commands_do_not_advance_simulated_time(self) -> None:
        simulation = self.build_simulation()
        simulation.advance_time(2.5)

        simulation.apply_command(
            self.command(CommandType.MOVE_FORWARD)
        )
        simulation.apply_command(
            self.command(CommandType.TURN_LEFT)
        )
        simulation.apply_command(
            self.command(CommandType.STOP)
        )

        self.assertEqual(
            simulation.elapsed_time_seconds,
            2.5,
        )

    def test_result_retains_the_supplied_command(self) -> None:
        simulation = self.build_simulation()
        command = self.command(CommandType.STOP)

        result = simulation.apply_command(command)

        self.assertIs(result.command, command)


if __name__ == "__main__":
    unittest.main()