"""Unit tests for immutable validated V1 settings."""

import unittest
from dataclasses import FrozenInstanceError

from ai_logistics_robot.app.settings import (
    MissionSettings,
    RobotSettings,
    ScenarioSettings,
    Settings,
    TargetSettings,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap


class SettingsTests(unittest.TestCase):
    """Verify individual settings and cross-object consistency."""

    def valid_settings(self) -> Settings:
        """Build the approved V1 reference settings."""

        target_position = Position(x=8, y=7)
        grid_map = GridMap(
            width=10,
            height=10,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=1, y=1),
            target_position=target_position,
            obstacles=frozenset(
                {
                    Position(x=1, y=4),
                    Position(x=2, y=4),
                    Position(x=3, y=4),
                    Position(x=5, y=6),
                }
            ),
        )

        return Settings(
            schema_version=1,
            scenario=ScenarioSettings(
                scenario_id="v1-reference",
                clock="simulated",
                random_seed=1001,
            ),
            grid_map=grid_map,
            robot=RobotSettings(
                robot_id="robot_1",
                initial_pose=RobotPose(
                    position=Position(x=1, y=1),
                    heading=Heading.NORTH,
                ),
                simulated_footprint_cells=1,
                safety_margin_cm=None,
            ),
            target=TargetSettings(
                target_id="target_1",
                target_position=target_position,
                arrival_policy="adjacent_safe_cell",
                debounce_ms=None,
                confidence_threshold=None,
            ),
            mission=MissionSettings(
                collection_duration_s=3,
                maximum_replans=None,
                timeout_s=None,
            ),
        )

    def test_reference_settings_accept_approved_values(self) -> None:
        settings = self.valid_settings()

        self.assertEqual(
            settings.scenario.scenario_id,
            "v1-reference",
        )
        self.assertEqual(settings.robot.robot_id, "robot_1")
        self.assertIsNone(settings.robot.safety_margin_cm)
        self.assertEqual(
            settings.grid_map.authorized_arrival_positions,
            (
                Position(x=8, y=8),
                Position(x=9, y=7),
                Position(x=8, y=6),
                Position(x=7, y=7),
            ),
        )

    def test_unknown_operational_values_remain_none(self) -> None:
        settings = self.valid_settings()

        self.assertIsNone(settings.target.debounce_ms)
        self.assertIsNone(
            settings.target.confidence_threshold
        )
        self.assertIsNone(settings.mission.maximum_replans)
        self.assertIsNone(settings.mission.timeout_s)

    def test_schema_version_must_be_positive_integer(self) -> None:
        valid = self.valid_settings()

        for value in (0, -1, 1.5, True):
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    Settings(  # type: ignore[arg-type]
                        schema_version=value,
                        scenario=valid.scenario,
                        grid_map=valid.grid_map,
                        robot=valid.robot,
                        target=valid.target,
                        mission=valid.mission,
                    )

    def test_scenario_requires_valid_identity_and_seed(self) -> None:
        with self.assertRaises(DomainValidationError):
            ScenarioSettings(
                scenario_id=" ",
                clock="simulated",
            )

        with self.assertRaises(DomainValidationError):
            ScenarioSettings(  # type: ignore[arg-type]
                scenario_id="v1-reference",
                clock="simulated",
                random_seed=True,
            )

    def test_robot_footprint_must_be_positive_integer(self) -> None:
        for value in (0, -1, 1.5, True):
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    RobotSettings(  # type: ignore[arg-type]
                        robot_id="robot_1",
                        initial_pose=RobotPose(
                            position=Position(x=1, y=1),
                            heading=Heading.NORTH,
                        ),
                        simulated_footprint_cells=value,
                    )

    def test_unknown_safety_margin_is_not_guessed(self) -> None:
        robot = RobotSettings(
            robot_id="robot_1",
            initial_pose=RobotPose(
                position=Position(x=1, y=1),
                heading=Heading.NORTH,
            ),
            simulated_footprint_cells=1,
            safety_margin_cm=None,
        )

        self.assertIsNone(robot.safety_margin_cm)

    def test_confidence_threshold_must_be_in_range(self) -> None:
        for value in (-0.1, 1.1, float("nan"), True):
            with self.subTest(value=value):
                with self.assertRaises(DomainValidationError):
                    TargetSettings(  # type: ignore[arg-type]
                        target_id="target_1",
                        target_position=Position(x=8, y=7),
                        arrival_policy="adjacent_safe_cell",
                        confidence_threshold=value,
                    )

    def test_mission_limits_must_be_valid_when_known(self) -> None:
        with self.assertRaises(DomainValidationError):
            MissionSettings(
                collection_duration_s=-1,
            )

        with self.assertRaises(DomainValidationError):
            MissionSettings(
                collection_duration_s=3,
                maximum_replans=0,
            )

        with self.assertRaises(DomainValidationError):
            MissionSettings(
                collection_duration_s=3,
                timeout_s=-1,
            )

    def test_initial_pose_must_lie_inside_grid(self) -> None:
        valid = self.valid_settings()
        robot = RobotSettings(
            robot_id="robot_1",
            initial_pose=RobotPose(
                position=Position(x=10, y=10),
                heading=Heading.NORTH,
            ),
            simulated_footprint_cells=1,
        )

        with self.assertRaises(DomainValidationError):
            Settings(
                schema_version=1,
                scenario=valid.scenario,
                grid_map=valid.grid_map,
                robot=robot,
                target=valid.target,
                mission=valid.mission,
            )

    def test_initial_pose_must_be_traversable(self) -> None:
        valid = self.valid_settings()
        robot = RobotSettings(
            robot_id="robot_1",
            initial_pose=RobotPose(
                position=Position(x=1, y=4),
                heading=Heading.NORTH,
            ),
            simulated_footprint_cells=1,
        )

        with self.assertRaises(DomainValidationError):
            Settings(
                schema_version=1,
                scenario=valid.scenario,
                grid_map=valid.grid_map,
                robot=robot,
                target=valid.target,
                mission=valid.mission,
            )

    def test_target_position_must_match_grid_map(self) -> None:
        valid = self.valid_settings()
        target = TargetSettings(
            target_id="target_1",
            target_position=Position(x=7, y=7),
            arrival_policy="adjacent_safe_cell",
        )

        with self.assertRaises(DomainValidationError):
            Settings(
                schema_version=1,
                scenario=valid.scenario,
                grid_map=valid.grid_map,
                robot=valid.robot,
                target=target,
                mission=valid.mission,
            )

    def test_settings_are_immutable(self) -> None:
        settings = self.valid_settings()

        with self.assertRaises(FrozenInstanceError):
            settings.schema_version = 2  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()