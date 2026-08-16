"""Reference integration tests for the public simulation assembly."""

import unittest
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from pathlib import Path

import ai_logistics_robot.app as app_package
from ai_logistics_robot.adapters.simulation import HeadlessRenderer
from ai_logistics_robot.app import (
    MissionRunner,
    SimulationApplication,
    build_simulation_application,
)
from ai_logistics_robot.app.settings import Settings, load_settings
from ai_logistics_robot.brain import DeterministicBrain
from ai_logistics_robot.control import SafeRobotControl
from ai_logistics_robot.domain.enums import (
    BrainState,
    MissionStatus,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.domain.status import SystemStatus

_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
_REFERENCE_CONFIG = (
    _REPOSITORY_ROOT
    / "configs"
    / "simulation.yaml"
)
_REFERENCE_EPOCH = datetime(
    2026,
    8,
    16,
    8,
    0,
    tzinfo=UTC,
)


@dataclass(frozen=True, slots=True)
class ReplayTrace:
    """Comparable result of one complete public application replay."""

    status: SystemStatus
    completed_mission: Mission
    outbound_poses: tuple[RobotPose, ...]
    return_poses: tuple[RobotPose, ...]
    events: tuple[MissionEvent, ...]
    displayed_events: tuple[MissionEvent, ...]
    elapsed_time_seconds: float


class ReferenceMissionIntegrationTests(unittest.TestCase):
    """Verify complete integration through reusable implementations."""

    def build_application(self) -> SimulationApplication:
        """Build one isolated headless reference application."""

        loaded: Settings = load_settings(
            _REFERENCE_CONFIG
        )
        settings = replace(
            loaded,
            renderer=replace(
                loaded.renderer,
                enabled=False,
            ),
        )

        return build_simulation_application(
            settings=settings,
            epoch=_REFERENCE_EPOCH,
        )

    def run_reference_mission(self) -> ReplayTrace:
        """Activate and complete one deterministic public mission."""

        application = self.build_application()
        application.perception.set_target_active(False)

        for _ in range(10):
            status = application.runner.start(
                maximum_cycles=1
            )

            if (
                status.brain_state
                is BrainState.WAITING_FOR_MISSION
            ):
                break
        else:
            self.fail(
                "the reference application did not reach "
                "its waiting state"
            )

        inactive_status = application.runner.start(
            maximum_cycles=1
        )

        self.assertIs(
            inactive_status.brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertIsNone(inactive_status.mission_id)

        application.perception.set_target_active(True)

        for _ in range(500):
            status = application.runner.start(
                maximum_cycles=1
            )

            if (
                status.brain_state
                is BrainState.WAITING_FOR_MISSION
                and status.mission_status
                is MissionStatus.SUCCESS
            ):
                break
        else:
            self.fail(
                "the reference mission did not complete "
                "within 500 application cycles"
            )

        completed_mission = (
            application.memory.completed_mission
        )

        self.assertIsNotNone(completed_mission)
        assert completed_mission is not None
        self.assertEqual(
            status.robot_pose,
            application.settings.robot.initial_pose,
        )
        self.assertEqual(
            application.simulation.current_pose,
            application.settings.robot.initial_pose,
        )
        self.assertTrue(
            all(
                pose.position
                != application.settings.grid_map
                .target_position
                for pose in (
                    *application.memory.outbound_poses,
                    *application.memory.return_poses,
                )
            )
        )

        mission_id = status.mission_id

        self.assertIsNotNone(mission_id)
        assert mission_id is not None

        events = application.monitoring.events_for(
            mission_id
        )

        self.assertEqual(
            application.memory.events,
            events,
        )

        for stored, published in zip(
            application.memory.events,
            events,
            strict=True,
        ):
            self.assertIs(stored, published)

        renderer = application.renderer

        self.assertIsInstance(
            renderer,
            HeadlessRenderer,
        )
        assert isinstance(renderer, HeadlessRenderer)

        self.assertEqual(
            renderer.displayed_events,
            events,
        )

        return ReplayTrace(
            status=status,
            completed_mission=completed_mission,
            outbound_poses=(
                application.memory.outbound_poses
            ),
            return_poses=(
                application.memory.return_poses
            ),
            events=events,
            displayed_events=(
                renderer.displayed_events
            ),
            elapsed_time_seconds=(
                application.clock.monotonic()
            ),
        )

    def test_public_application_assembly_is_exported(
        self,
    ) -> None:
        application = self.build_application()

        self.assertEqual(
            app_package.__all__,
            (
                "MissionRunner",
                "SimulationApplication",
                "build_simulation_application",
            ),
        )
        self.assertIsInstance(
            application,
            SimulationApplication,
        )
        self.assertIsInstance(
            application.runner,
            MissionRunner,
        )
        self.assertIsInstance(
            application.brain,
            DeterministicBrain,
        )
        self.assertIsInstance(
            application.control,
            SafeRobotControl,
        )
        self.assertIsInstance(
            application.renderer,
            HeadlessRenderer,
        )
        self.assertTrue(application.runner.configured)

    def test_reference_configuration_completes_successfully(
        self,
    ) -> None:
        trace = self.run_reference_mission()

        self.assertIs(
            trace.status.brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertIs(
            trace.completed_mission.status,
            MissionStatus.SUCCESS,
        )
        self.assertTrue(
            trace.completed_mission.collection_completed
        )
        self.assertTrue(
            trace.completed_mission.base_arrival_confirmed
        )
        self.assertIsNone(
            trace.completed_mission.terminal_reason
        )
        self.assertIsNone(trace.status.latest_error)
        self.assertFalse(
            trace.status.safety_status.latched
        )
        self.assertTrue(trace.outbound_poses)
        self.assertTrue(trace.return_poses)
        self.assertEqual(
            trace.events[0].name,
            "mission_started",
        )
        self.assertEqual(
            trace.events[-1].name,
            "mission_completed",
        )
        self.assertIn(
            "outbound_plan_created",
            tuple(
                event.name
                for event in trace.events
            ),
        )
        self.assertIn(
            "collection_completed",
            tuple(
                event.name
                for event in trace.events
            ),
        )
        self.assertIn(
            "return_path_prepared",
            tuple(
                event.name
                for event in trace.events
            ),
        )
        self.assertEqual(
            trace.displayed_events,
            trace.events,
        )
        self.assertEqual(
            trace.elapsed_time_seconds,
            3.0,
        )

    def test_reference_mission_replays_identically(
        self,
    ) -> None:
        first_trace = self.run_reference_mission()
        second_trace = self.run_reference_mission()

        self.assertEqual(second_trace, first_trace)


if __name__ == "__main__":
    unittest.main()
