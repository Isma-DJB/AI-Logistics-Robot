"""Integration tests for the V1 reference GridWorld scenario."""

import unittest
from pathlib import Path

from ai_logistics_robot.adapters.simulation import GridWorld
from ai_logistics_robot.app.settings import Settings, load_settings
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
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.ports import SimulationPort


class GridWorldReferenceScenarioTests(unittest.TestCase):
    """Verify GridWorld against the validated V1 configuration."""

    def setUp(self) -> None:
        self.project_root = Path(__file__).resolve().parents[3]
        self.settings: Settings = load_settings(
            self.project_root / "configs" / "simulation.yaml"
        )
        self.simulation = GridWorld(
            world=self.settings.grid_map,
            robot_id=self.settings.robot.robot_id,
            initial_pose=self.settings.robot.initial_pose,
        )

    def command(self, command_type: CommandType) -> MotionCommand:
        """Build one command for the configured reference robot."""

        return MotionCommand(
            robot_id=self.settings.robot.robot_id,
            command_type=command_type,
        )

    def run_reference_fragment(
        self,
    ) -> tuple[CommandResult, ...]:
        """Execute one deterministic safe fragment."""

        command_types = (
            CommandType.MOVE_FORWARD,
            CommandType.TURN_RIGHT,
            CommandType.MOVE_FORWARD,
            CommandType.STOP,
        )

        results = tuple(
            self.simulation.apply_command(
                self.command(command_type)
            )
            for command_type in command_types
        )
        self.simulation.advance_time(
            self.settings.mission.collection_duration_s
        )

        return results

    def test_grid_world_is_public_and_satisfies_port(self) -> None:
        self.assertIsInstance(
            self.simulation,
            SimulationPort,
        )

    def test_reference_settings_initialize_world_state(self) -> None:
        self.assertIs(
            self.simulation.read_world(),
            self.settings.grid_map,
        )
        self.assertEqual(
            self.simulation.current_pose,
            RobotPose(
                position=Position(x=1, y=1),
                heading=Heading.NORTH,
            ),
        )
        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            0.0,
        )

    def test_reference_fragment_replays_deterministically(
        self,
    ) -> None:
        first_results = self.run_reference_fragment()
        first_pose = self.simulation.current_pose
        first_time = self.simulation.elapsed_time_seconds

        self.simulation.reset()

        second_results = self.run_reference_fragment()

        self.assertEqual(first_results, second_results)
        self.assertEqual(
            first_pose,
            RobotPose(
                position=Position(x=2, y=2),
                heading=Heading.EAST,
            ),
        )
        self.assertEqual(
            self.simulation.current_pose,
            first_pose,
        )
        self.assertEqual(first_time, 3.0)
        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            first_time,
        )

    def test_reference_obstacle_blocks_movement(self) -> None:
        first_result = self.simulation.apply_command(
            self.command(CommandType.MOVE_FORWARD)
        )
        second_result = self.simulation.apply_command(
            self.command(CommandType.MOVE_FORWARD)
        )
        blocked_result = self.simulation.apply_command(
            self.command(CommandType.MOVE_FORWARD)
        )

        self.assertIs(
            first_result.status,
            CommandStatus.SUCCESS,
        )
        self.assertIs(
            second_result.status,
            CommandStatus.SUCCESS,
        )
        self.assertIs(
            blocked_result.status,
            CommandStatus.FAILED,
        )
        self.assertIs(
            blocked_result.failure_reason,
            FailureReason.BLOCKED,
        )
        self.assertEqual(
            blocked_result.pose_after,
            RobotPose(
                position=Position(x=1, y=3),
                heading=Heading.NORTH,
            ),
        )
        self.assertEqual(
            self.simulation.current_pose,
            blocked_result.pose_after,
        )


if __name__ == "__main__":
    unittest.main()
