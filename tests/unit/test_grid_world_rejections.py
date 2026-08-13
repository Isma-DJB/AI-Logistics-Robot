"""Unit tests for GridWorld command rejection and atomic state."""

import unittest

from ai_logistics_robot.adapters.simulation.grid_world import GridWorld
from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    FailureReason,
    Heading,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap


class GridWorldRejectionTests(unittest.TestCase):
    """Verify invalid commands, collisions, and boundary rejection."""

    def setUp(self) -> None:
        self.robot_id = "robot_1"
        self.world = GridMap(
            width=5,
            height=5,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=2, y=2),
            target_position=Position(x=4, y=4),
            obstacles=frozenset({Position(x=2, y=3)}),
        )

    def build_simulation(
        self,
        *,
        world: GridMap | None = None,
        position: Position | None = None,
        heading: Heading = Heading.NORTH,
    ) -> GridWorld:
        """Build one simulation from explicit test state."""

        selected_world = self.world if world is None else world
        selected_position = (
            selected_world.base_position
            if position is None
            else position
        )

        return GridWorld(
            world=selected_world,
            robot_id=self.robot_id,
            initial_pose=RobotPose(
                position=selected_position,
                heading=heading,
            ),
        )

    def command(
        self,
        command_type: CommandType,
        *,
        robot_id: str | None = None,
    ) -> MotionCommand:
        """Build one valid domain command."""

        return MotionCommand(
            robot_id=(
                self.robot_id
                if robot_id is None
                else robot_id
            ),
            command_type=command_type,
        )

    def assert_failed_movement(
        self,
        simulation: GridWorld,
        expected_reason: FailureReason,
    ) -> CommandResult:
        """Verify one normalized failure without state mutation."""

        pose_before = simulation.current_pose
        command = self.command(CommandType.MOVE_FORWARD)

        result = simulation.apply_command(command)

        self.assertIs(result.command, command)
        self.assertIs(result.status, CommandStatus.FAILED)
        self.assertIs(result.failure_reason, expected_reason)
        self.assertEqual(result.pose_before, pose_before)
        self.assertEqual(result.pose_after, pose_before)
        self.assertEqual(simulation.current_pose, pose_before)

        return result

    def test_invalid_command_type_is_rejected_atomically(self) -> None:
        simulation = self.build_simulation()
        pose_before = simulation.current_pose

        with self.assertRaises(DomainValidationError):
            simulation.apply_command(  # type: ignore[arg-type]
                "MOVE_FORWARD"
            )

        self.assertEqual(simulation.current_pose, pose_before)

    def test_command_for_another_robot_is_rejected_atomically(
        self,
    ) -> None:
        simulation = self.build_simulation()
        pose_before = simulation.current_pose
        foreign_command = self.command(
            CommandType.STOP,
            robot_id="robot_2",
        )

        with self.assertRaises(DomainValidationError):
            simulation.apply_command(foreign_command)

        self.assertEqual(simulation.current_pose, pose_before)

    def test_every_boundary_rejects_forward_movement(self) -> None:
        shifted_world = GridMap(
            width=3,
            height=3,
            cell_size_cm=20,
            origin=Position(x=5, y=7),
            base_position=Position(x=6, y=8),
            target_position=Position(x=7, y=9),
        )
        cases = (
            (Position(x=5, y=9), Heading.NORTH),
            (Position(x=7, y=7), Heading.EAST),
            (Position(x=5, y=7), Heading.SOUTH),
            (Position(x=5, y=8), Heading.WEST),
        )

        for position, heading in cases:
            with self.subTest(
                position=position,
                heading=heading,
            ):
                simulation = self.build_simulation(
                    world=shifted_world,
                    position=position,
                    heading=heading,
                )

                self.assert_failed_movement(
                    simulation,
                    FailureReason.OUT_OF_BOUNDS,
                )

    def test_obstacle_rejects_forward_movement(self) -> None:
        simulation = self.build_simulation()

        self.assert_failed_movement(
            simulation,
            FailureReason.BLOCKED,
        )

    def test_target_cell_rejects_forward_movement(self) -> None:
        simulation = self.build_simulation(
            position=Position(x=4, y=3),
            heading=Heading.NORTH,
        )

        self.assert_failed_movement(
            simulation,
            FailureReason.BLOCKED,
        )

    def test_failed_movement_preserves_simulated_time(self) -> None:
        simulation = self.build_simulation()
        simulation.advance_time(2.5)

        self.assert_failed_movement(
            simulation,
            FailureReason.BLOCKED,
        )

        self.assertEqual(
            simulation.elapsed_time_seconds,
            2.5,
        )

    def test_valid_commands_continue_after_rejected_movement(
        self,
    ) -> None:
        simulation = self.build_simulation()

        failed_result = self.assert_failed_movement(
            simulation,
            FailureReason.BLOCKED,
        )
        turn_result = simulation.apply_command(
            self.command(CommandType.TURN_RIGHT)
        )
        move_result = simulation.apply_command(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertEqual(
            turn_result.pose_before,
            failed_result.pose_after,
        )
        self.assertEqual(
            move_result.pose_after,
            RobotPose(
                position=Position(x=3, y=2),
                heading=Heading.EAST,
            ),
        )
        self.assertEqual(
            simulation.current_pose,
            move_result.pose_after,
        )


if __name__ == "__main__":
    unittest.main()