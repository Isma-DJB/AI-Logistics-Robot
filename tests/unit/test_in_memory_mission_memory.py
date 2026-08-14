"""Unit tests for the in-memory mission path lifecycle."""

import unittest

from ai_logistics_robot.domain.enums import Heading, PathPhase
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.memory.in_memory_mission_memory import (
    InMemoryMissionMemory,
)
from ai_logistics_robot.ports.memory_port import MemoryPort


class InMemoryMissionMemoryTests(unittest.TestCase):
    """Verify nominal in-memory mission and pose recording."""

    def setUp(self) -> None:
        self.mission = Mission(
            mission_id="mission_1",
            robot_id="robot_1",
            target_id="target_1",
            target_position=Position(x=4, y=4),
            base_position=Position(x=0, y=0),
        )
        self.pose_0 = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.pose_1 = RobotPose(
            position=Position(x=0, y=1),
            heading=Heading.NORTH,
        )
        self.pose_1_east = RobotPose(
            position=Position(x=0, y=1),
            heading=Heading.EAST,
        )
        self.return_pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.SOUTH,
        )
        self.memory = InMemoryMissionMemory()

    def test_memory_satisfies_runtime_protocol(self) -> None:
        self.assertIsInstance(self.memory, MemoryPort)

    def test_start_initializes_empty_recording_state(
        self,
    ) -> None:
        self.memory.start(self.mission)

        self.assertIs(
            self.memory.active_mission,
            self.mission,
        )
        self.assertIsNone(self.memory.completed_mission)
        self.assertEqual(self.memory.outbound_poses, ())
        self.assertEqual(self.memory.return_poses, ())
        self.assertEqual(self.memory.events, ())

    def test_record_pose_separates_navigation_phases(
        self,
    ) -> None:
        self.memory.start(self.mission)

        self.memory.record_pose(
            PathPhase.OUTBOUND,
            self.pose_0,
        )
        self.memory.record_pose(
            PathPhase.OUTBOUND,
            self.pose_1,
        )
        self.memory.record_pose(
            PathPhase.RETURN,
            self.return_pose,
        )

        self.assertEqual(
            self.memory.outbound_poses,
            (self.pose_0, self.pose_1),
        )
        self.assertEqual(
            self.memory.return_poses,
            (self.return_pose,),
        )

    def test_build_return_path_reverses_exact_outbound_history(
        self,
    ) -> None:
        self.memory.start(self.mission)

        for pose in (
            self.pose_0,
            self.pose_1,
            self.pose_1_east,
        ):
            self.memory.record_pose(
                PathPhase.OUTBOUND,
                pose,
            )

        return_path = self.memory.build_return_path()

        self.assertEqual(
            return_path.mission_id,
            self.mission.mission_id,
        )
        self.assertEqual(
            return_path.robot_id,
            self.mission.robot_id,
        )
        self.assertIs(
            return_path.phase,
            PathPhase.RETURN,
        )
        self.assertEqual(
            return_path.confirmed_poses,
            (
                self.pose_1_east,
                self.pose_1,
                self.pose_0,
            ),
        )

    def test_return_path_ignores_return_phase_history(
        self,
    ) -> None:
        self.memory.start(self.mission)
        self.memory.record_pose(
            PathPhase.OUTBOUND,
            self.pose_0,
        )
        self.memory.record_pose(
            PathPhase.RETURN,
            self.return_pose,
        )

        return_path = self.memory.build_return_path()

        self.assertEqual(
            return_path.confirmed_poses,
            (self.pose_0,),
        )
        self.assertEqual(
            self.memory.return_poses,
            (self.return_pose,),
        )

    def test_empty_outbound_history_builds_empty_return_path(
        self,
    ) -> None:
        self.memory.start(self.mission)

        return_path = self.memory.build_return_path()

        self.assertEqual(return_path.confirmed_poses, ())
        self.assertEqual(return_path.confirmed_positions, ())

    def test_return_path_build_is_repeatable_and_read_only(
        self,
    ) -> None:
        self.memory.start(self.mission)
        self.memory.record_pose(
            PathPhase.OUTBOUND,
            self.pose_0,
        )
        self.memory.record_pose(
            PathPhase.OUTBOUND,
            self.pose_1,
        )

        first_path = self.memory.build_return_path()
        second_path = self.memory.build_return_path()

        self.assertEqual(first_path, second_path)
        self.assertEqual(
            self.memory.outbound_poses,
            (self.pose_0, self.pose_1),
        )

    def test_duplicate_positions_and_headings_are_preserved(
        self,
    ) -> None:
        self.memory.start(self.mission)

        self.memory.record_pose(
            PathPhase.OUTBOUND,
            self.pose_1,
        )
        self.memory.record_pose(
            PathPhase.OUTBOUND,
            self.pose_1_east,
        )

        self.assertEqual(
            self.memory.outbound_poses,
            (self.pose_1, self.pose_1_east),
        )

    def test_public_state_properties_are_read_only(
        self,
    ) -> None:
        self.memory.start(self.mission)

        with self.assertRaises(AttributeError):
            self.memory.active_mission = None  # type: ignore[misc]

        with self.assertRaises(AttributeError):
            self.memory.outbound_poses = ()  # type: ignore[misc]

        with self.assertRaises(AttributeError):
            self.memory.events = ()  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()
