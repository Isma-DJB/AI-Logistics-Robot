"""Unit tests for deterministic GridWorld perception."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.adapters.simulation import (
    GridWorld,
    GridWorldPerception,
    SimulatedClock,
)
from ai_logistics_robot.domain.commands import MotionCommand
from ai_logistics_robot.domain.enums import (
    CommandType,
    Heading,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.perception import Observation
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports import PerceptionPort


class GridWorldPerceptionTests(unittest.TestCase):
    """Verify normalized externally controlled simulation input."""

    def setUp(self) -> None:
        self.epoch = datetime(
            2026,
            8,
            16,
            9,
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
        self.initial_pose = RobotPose(
            position=Position(x=0, y=0),
            heading=Heading.NORTH,
        )
        self.simulation = GridWorld(
            world=self.world,
            robot_id="robot_1",
            initial_pose=self.initial_pose,
        )
        self.clock = SimulatedClock(
            simulation=self.simulation,
            epoch=self.epoch,
        )
        self.perception = GridWorldPerception(
            robot_id="robot_1",
            simulation=self.simulation,
            clock=self.clock,
        )

    def test_perception_satisfies_public_port(self) -> None:
        self.assertIsInstance(
            self.perception,
            PerceptionPort,
        )

    def test_default_snapshot_is_safe_and_inactive(
        self,
    ) -> None:
        snapshot = self.perception.observe()

        self.assertEqual(
            snapshot.robot_id,
            "robot_1",
        )
        self.assertEqual(
            snapshot.captured_at,
            self.epoch,
        )
        self.assertIs(
            snapshot.robot_pose,
            self.initial_pose,
        )
        self.assertEqual(
            snapshot.observations,
            (),
        )
        self.assertFalse(snapshot.target_active)
        self.assertFalse(snapshot.hazard_detected)

    def test_explicit_simulation_inputs_are_normalized(
        self,
    ) -> None:
        observation = Observation(
            kind="obstacle",
            position=Position(x=1, y=1),
            confidence=1.0,
        )

        self.perception.set_target_active(True)
        self.perception.set_hazard_detected(True)
        self.perception.set_observations(
            (observation,)
        )

        snapshot = self.perception.observe()

        self.assertTrue(snapshot.target_active)
        self.assertTrue(snapshot.hazard_detected)
        self.assertEqual(
            snapshot.observations,
            (observation,),
        )
        self.assertIs(
            snapshot.observations[0],
            observation,
        )

    def test_snapshot_tracks_confirmed_pose_and_time(
        self,
    ) -> None:
        command = MotionCommand(
            robot_id="robot_1",
            command_type=CommandType.MOVE_FORWARD,
        )

        result = self.simulation.apply_command(command)
        self.clock.wait_until(2.0)
        snapshot = self.perception.observe()

        self.assertIs(
            snapshot.robot_pose,
            result.pose_after,
        )
        self.assertEqual(
            snapshot.robot_pose.position,
            Position(x=0, y=1),
        )
        self.assertEqual(
            snapshot.captured_at,
            self.epoch.replace(second=2),
        )

    def test_observe_is_passive(self) -> None:
        pose_before = self.simulation.current_pose
        time_before = self.simulation.elapsed_time_seconds

        first = self.perception.observe()
        second = self.perception.observe()

        self.assertEqual(first, second)
        self.assertIs(
            self.simulation.current_pose,
            pose_before,
        )
        self.assertEqual(
            self.simulation.elapsed_time_seconds,
            time_before,
        )

    def test_input_setters_reject_invalid_values(
        self,
    ) -> None:
        for value in (1, "true", None):
            with self.subTest(
                setter="target",
                value=value,
            ):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.perception.set_target_active(
                        value  # type: ignore[arg-type]
                    )

            with self.subTest(
                setter="hazard",
                value=value,
            ):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.perception.set_hazard_detected(
                        value  # type: ignore[arg-type]
                    )

        invalid_observations = (
            [],
            (object(),),
            None,
        )

        for observations in invalid_observations:
            with self.subTest(
                observations=observations
            ):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.perception.set_observations(
                        observations  # type: ignore[arg-type]
                    )

    def test_constructor_rejects_invalid_dependencies(
        self,
    ) -> None:
        cases = (
            {
                "robot_id": "",
                "simulation": self.simulation,
                "clock": self.clock,
            },
            {
                "robot_id": "robot_1",
                "simulation": None,
                "clock": self.clock,
            },
            {
                "robot_id": "robot_1",
                "simulation": self.simulation,
                "clock": None,
            },
        )

        for case in cases:
            with self.subTest(case=case):
                with self.assertRaises(
                    DomainValidationError
                ):
                    GridWorldPerception(
                        **case,  # type: ignore[arg-type]
                    )


if __name__ == "__main__":
    unittest.main()
