"""Complete V1 software acceptance scenarios."""

import unittest
from collections.abc import Callable
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from pathlib import Path

from ai_logistics_robot.adapters.simulation import HeadlessRenderer
from ai_logistics_robot.app import (
    SimulationApplication,
    build_simulation_application,
)
from ai_logistics_robot.app.settings import Settings, load_settings
from ai_logistics_robot.domain.enums import (
    BrainState,
    FailureReason,
    MissionStatus,
    PathPhase,
)
from ai_logistics_robot.domain.errors import InvalidStateTransitionError
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.status import SystemStatus


@dataclass(frozen=True, slots=True)
class AcceptanceTrace:
    """Retain one comparable deterministic mission result."""

    final_status: SystemStatus
    outbound_poses: tuple[RobotPose, ...]
    return_poses: tuple[RobotPose, ...]
    return_plan_positions: tuple[Position, ...]
    events: tuple[MissionEvent, ...]
    elapsed_time_seconds: float


class V1SoftwareAcceptanceTests(unittest.TestCase):
    """Verify AC-01 through AC-12 through the application boundary."""

    def setUp(self) -> None:
        self.project_root = Path(__file__).resolve().parents[3]
        loaded: Settings = load_settings(
            self.project_root
            / "configs"
            / "simulation.yaml"
        )
        self.settings = replace(
            loaded,
            renderer=replace(
                loaded.renderer,
                enabled=False,
            ),
        )
        self.epoch = datetime(
            2026,
            8,
            16,
            14,
            0,
            tzinfo=UTC,
        )

    def build_application(self) -> SimulationApplication:
        """Build one complete headless reference application."""

        return build_simulation_application(
            settings=self.settings,
            epoch=self.epoch,
        )

    def cycle(
        self,
        application: SimulationApplication,
    ) -> SystemStatus:
        """Execute exactly one complete application cycle."""

        return application.runner.start(
            maximum_cycles=1
        )

    def advance_until(
        self,
        application: SimulationApplication,
        predicate: Callable[[SystemStatus], bool],
        *,
        maximum_cycles: int = 500,
    ) -> SystemStatus:
        """Advance deterministically until one status predicate holds."""

        status = application.runner.get_status()

        if predicate(status):
            return status

        for _ in range(maximum_cycles):
            status = self.cycle(application)

            if predicate(status):
                return status

        self.fail(
            "the acceptance scenario did not reach its "
            "required state within the cycle bound"
        )

    def advance_to_state(
        self,
        application: SimulationApplication,
        state: BrainState,
    ) -> SystemStatus:
        """Advance until the requested Brain state is visible."""

        return self.advance_until(
            application,
            lambda status: status.brain_state is state,
        )

    def activate_mission(
        self,
        application: SimulationApplication,
    ) -> str:
        """Create one inactive-to-active target edge."""

        application.perception.set_target_active(False)
        self.advance_to_state(
            application,
            BrainState.WAITING_FOR_MISSION,
        )

        inactive_status = self.cycle(application)

        self.assertIs(
            inactive_status.brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertIsNone(
            inactive_status.mission_id
        )

        application.perception.set_target_active(True)
        status = self.cycle(application)

        self.assertIsNotNone(status.mission_id)
        assert status.mission_id is not None
        self.assertIs(
            status.mission_status,
            MissionStatus.ACTIVE,
        )

        return status.mission_id

    @staticmethod
    def next_distinct_position(
        status: SystemStatus,
        *,
        excluded: frozenset[Position] = frozenset(),
    ) -> Position:
        """Return the next distinct non-excluded planned position."""

        plan = status.active_plan

        if plan is None:
            raise AssertionError(
                "an active plan is required"
            )

        current_position = status.robot_pose.position

        for position in plan.positions[1:]:
            if (
                position != current_position
                and position not in excluded
            ):
                return position

        raise AssertionError(
            "the active plan contains no usable next position"
        )

    def complete_active_mission(
        self,
        application: SimulationApplication,
    ) -> AcceptanceTrace:
        """Complete one already activated mission and retain its trace."""

        collection_status = self.advance_to_state(
            application,
            BrainState.COLLECTION,
        )
        arrival_pose = collection_status.robot_pose

        self.assertIn(
            arrival_pose.position,
            application.settings.grid_map
            .authorized_arrival_positions,
        )
        self.assertNotEqual(
            arrival_pose.position,
            application.settings.grid_map.target_position,
        )

        collection_started_at = (
            application.clock.monotonic()
        )

        after_collection = self.cycle(application)

        self.assertIs(
            after_collection.brain_state,
            BrainState.RETURN_PREPARATION,
        )
        self.assertEqual(
            after_collection.robot_pose,
            arrival_pose,
        )
        self.assertEqual(
            application.clock.monotonic(),
            (
                collection_started_at
                + application.settings
                .mission.collection_duration_s
            ),
        )

        return_status = self.advance_to_state(
            application,
            BrainState.RETURN_NAVIGATION,
        )
        return_plan = return_status.active_plan

        self.assertIsNotNone(return_plan)
        assert return_plan is not None
        self.assertIs(
            return_plan.phase,
            PathPhase.RETURN,
        )

        return_record = (
            application.memory.build_return_path()
        )

        self.assertEqual(
            return_plan.positions,
            return_record.confirmed_positions,
        )
        self.assertEqual(
            return_plan.goal,
            application.settings.grid_map.base_position,
        )

        final_status = self.advance_until(
            application,
            lambda status: (
                status.brain_state
                is BrainState.WAITING_FOR_MISSION
                and status.mission_status
                is MissionStatus.SUCCESS
            ),
        )

        completed = application.memory.completed_mission

        self.assertIsNotNone(completed)
        assert completed is not None
        self.assertIs(
            completed.status,
            MissionStatus.SUCCESS,
        )
        self.assertTrue(completed.collection_completed)
        self.assertTrue(completed.base_arrival_confirmed)
        self.assertIsNone(completed.terminal_reason)
        self.assertEqual(
            final_status.robot_pose.position,
            application.settings.grid_map.base_position,
        )
        self.assertEqual(
            final_status.robot_pose,
            application.settings.robot.initial_pose,
        )
        self.assertFalse(
            final_status.safety_status.latched
        )

        mission_id = final_status.mission_id

        self.assertIsNotNone(mission_id)
        assert mission_id is not None

        events = application.monitoring.events_for(
            mission_id
        )

        self.assertEqual(
            application.memory.events,
            events,
        )

        renderer = application.renderer

        self.assertIsInstance(
            renderer,
            HeadlessRenderer,
        )
        assert isinstance(renderer, HeadlessRenderer)
        self.assertEqual(
            renderer.displayed_events[-len(events):],
            events,
        )

        configured_world = application.settings.grid_map

        self.assertTrue(
            all(
                configured_world.is_traversable(
                    pose.position
                )
                for pose in (
                    *application.memory.outbound_poses,
                    *application.memory.return_poses,
                )
            )
        )
        self.assertTrue(
            all(
                pose.position
                != configured_world.target_position
                for pose in (
                    *application.memory.outbound_poses,
                    *application.memory.return_poses,
                )
            )
        )

        return AcceptanceTrace(
            final_status=final_status,
            outbound_poses=(
                application.memory.outbound_poses
            ),
            return_poses=application.memory.return_poses,
            return_plan_positions=return_plan.positions,
            events=events,
            elapsed_time_seconds=(
                application.clock.monotonic()
            ),
        )

    def test_seq_01_nominal_covers_core_acceptance(
        self,
    ) -> None:
        """Verify AC-01, 02, 03, 05, 06, 07, 09, and 12."""

        application = self.build_application()
        initial_pose = application.runner.get_status().robot_pose

        application.perception.set_target_active(False)

        for _ in range(4):
            status = self.cycle(application)

        self.assertIs(
            status.brain_state,
            BrainState.WAITING_FOR_MISSION,
        )
        self.assertIsNone(status.mission_id)
        self.assertEqual(
            status.robot_pose,
            initial_pose,
        )
        self.assertEqual(
            application.clock.monotonic(),
            0.0,
        )
        self.assertEqual(
            application.memory.events,
            (),
        )

        mission_id = self.activate_mission(
            application
        )
        trace = self.complete_active_mission(
            application
        )
        event_names = tuple(
            event.name
            for event in trace.events
        )

        self.assertEqual(
            trace.final_status.mission_id,
            mission_id,
        )
        self.assertEqual(
            event_names.count("mission_started"),
            1,
        )
        self.assertEqual(
            event_names[0],
            "mission_started",
        )
        self.assertEqual(
            event_names[-1],
            "mission_completed",
        )

        required_events = {
            "outbound_plan_created",
            "arrival_confirmed",
            "collection_completed",
            "return_path_prepared",
            "base_arrival_confirmed",
            "mission_completed",
        }

        self.assertTrue(
            required_events.issubset(event_names)
        )
        self.assertEqual(
            tuple(
                event.sequence_number
                for event in trace.events
            ),
            tuple(
                range(1, len(trace.events) + 1)
            ),
        )
        self.assertEqual(
            trace.elapsed_time_seconds,
            self.settings.mission.collection_duration_s,
        )

    def test_seq_02_outbound_block_replans_and_completes(
        self,
    ) -> None:
        """Verify AC-04 with real rejection and detour planning."""

        application = self.build_application()
        mission_id = self.activate_mission(
            application
        )
        navigation = self.advance_to_state(
            application,
            BrainState.OUTBOUND_NAVIGATION,
        )
        hidden_obstacle = self.next_distinct_position(
            navigation
        )

        application.simulation.set_transient_obstacles(
            frozenset({hidden_obstacle})
        )

        for _ in range(20):
            before_status = application.runner.get_status()
            poses_before = len(
                application.memory.outbound_poses
            )
            status = self.cycle(application)

            if (
                status.brain_state
                is BrainState.OUTBOUND_REPLANNING
            ):
                self.assertEqual(
                    status.robot_pose,
                    before_status.robot_pose,
                )
                self.assertEqual(
                    len(application.memory.outbound_poses),
                    poses_before,
                )
                break
        else:
            self.fail(
                "the outbound transient obstacle "
                "did not reject movement"
            )

        self.assertIn(
            hidden_obstacle,
            application.simulation
            .read_world().obstacles,
        )

        replanned = self.cycle(application)
        detour = replanned.active_plan

        self.assertIs(
            replanned.brain_state,
            BrainState.OUTBOUND_NAVIGATION,
        )
        self.assertIsNotNone(detour)
        assert detour is not None
        self.assertIs(detour.phase, PathPhase.DETOUR)
        self.assertEqual(detour.version, 2)
        self.assertNotIn(
            hidden_obstacle,
            detour.positions,
        )

        trace = self.complete_active_mission(
            application
        )
        event_names = tuple(
            event.name
            for event in trace.events
        )

        self.assertEqual(
            trace.final_status.mission_id,
            mission_id,
        )
        self.assertIn(
            "outbound_step_blocked",
            event_names,
        )
        self.assertIn(
            "outbound_plan_recreated",
            event_names,
        )
        self.assertNotIn(
            hidden_obstacle,
            tuple(
                pose.position
                for pose in trace.outbound_poses
            ),
        )

    def test_seq_02_return_block_replans_safe_detour(
        self,
    ) -> None:
        """Verify AC-08 exact reversal and optional safe detour."""

        application = self.build_application()
        mission_id = self.activate_mission(
            application
        )

        self.advance_to_state(
            application,
            BrainState.COLLECTION,
        )
        self.cycle(application)

        return_status = self.advance_to_state(
            application,
            BrainState.RETURN_NAVIGATION,
        )
        original_plan = return_status.active_plan

        self.assertIsNotNone(original_plan)
        assert original_plan is not None
        self.assertIs(
            original_plan.phase,
            PathPhase.RETURN,
        )
        self.assertEqual(
            original_plan.positions,
            application.memory
            .build_return_path().confirmed_positions,
        )

        hidden_obstacle = self.next_distinct_position(
            return_status,
            excluded=frozenset(
                {
                    application.settings.grid_map
                    .base_position,
                }
            ),
        )

        application.simulation.set_transient_obstacles(
            frozenset({hidden_obstacle})
        )

        for _ in range(20):
            before_status = application.runner.get_status()
            poses_before = len(
                application.memory.return_poses
            )
            status = self.cycle(application)

            if (
                status.brain_state
                is BrainState.RETURN_REPLANNING
            ):
                self.assertEqual(
                    status.robot_pose,
                    before_status.robot_pose,
                )
                self.assertEqual(
                    len(application.memory.return_poses),
                    poses_before,
                )
                break
        else:
            self.fail(
                "the return transient obstacle "
                "did not reject movement"
            )

        replanned = self.cycle(application)
        detour = replanned.active_plan

        self.assertIs(
            replanned.brain_state,
            BrainState.RETURN_NAVIGATION,
        )
        self.assertIsNotNone(detour)
        assert detour is not None
        self.assertIs(detour.phase, PathPhase.DETOUR)
        self.assertEqual(detour.version, 2)
        self.assertEqual(
            detour.goal,
            application.settings.grid_map.base_position,
        )
        self.assertNotIn(
            hidden_obstacle,
            detour.positions,
        )

        final_status = self.advance_until(
            application,
            lambda current: (
                current.brain_state
                is BrainState.WAITING_FOR_MISSION
                and current.mission_status
                is MissionStatus.SUCCESS
            ),
        )
        events = application.monitoring.events_for(
            mission_id
        )
        event_names = tuple(
            event.name
            for event in events
        )

        self.assertEqual(
            final_status.robot_pose.position,
            application.settings.grid_map.base_position,
        )
        self.assertIn(
            "return_step_blocked",
            event_names,
        )
        self.assertIn(
            "return_plan_recreated",
            event_names,
        )
        self.assertNotIn(
            hidden_obstacle,
            tuple(
                pose.position
                for pose in application.memory.return_poses
            ),
        )

    def test_seq_03_hazard_latches_and_never_resumes(
        self,
    ) -> None:
        """Verify AC-10 priority stop and manual recovery rules."""

        application = self.build_application()
        mission_id = self.activate_mission(
            application
        )
        base_position = (
            application.settings.grid_map.base_position
        )

        moving_status = self.advance_until(
            application,
            lambda status: (
                status.brain_state
                is BrainState.OUTBOUND_NAVIGATION
                and status.robot_pose.position
                != base_position
            ),
        )

        self.assertNotEqual(
            moving_status.robot_pose.position,
            base_position,
        )

        application.perception.set_hazard_detected(
            True
        )
        stopped = self.cycle(application)

        self.assertIs(
            stopped.brain_state,
            BrainState.SAFETY_STOP,
        )
        self.assertIs(
            stopped.mission_status,
            MissionStatus.ABORTED,
        )
        self.assertIs(
            stopped.latest_error,
            FailureReason.EMERGENCY_STOP,
        )
        self.assertTrue(
            stopped.safety_status.latched
        )
        self.assertFalse(application.runner.running)

        completed = application.memory.completed_mission

        self.assertIsNotNone(completed)
        assert completed is not None
        self.assertIs(
            completed.status,
            MissionStatus.ABORTED,
        )
        self.assertIs(
            completed.terminal_reason,
            FailureReason.EMERGENCY_STOP,
        )
        self.assertEqual(
            application.monitoring.events_for(
                mission_id
            )[-1].name,
            "safety_stop",
        )

        with self.assertRaises(
            InvalidStateTransitionError
        ):
            application.runner.reset()

        rearmed = (
            application.runner.request_safety_rearm()
        )

        self.assertFalse(rearmed.latched)
        application.perception.set_hazard_detected(
            False
        )

        with self.assertRaises(
            InvalidStateTransitionError
        ):
            application.runner.start(
                maximum_cycles=1
            )

        with self.assertRaises(
            InvalidStateTransitionError
        ):
            application.runner.reset()

        after_rearm = application.runner.get_status()

        self.assertIs(
            after_rearm.brain_state,
            BrainState.SAFETY_STOP,
        )
        self.assertIs(
            application.memory.completed_mission,
            completed,
        )
        self.assertEqual(
            after_rearm.robot_pose,
            stopped.robot_pose,
        )

    def test_ac_11_two_missions_without_process_restart(
        self,
    ) -> None:
        """Verify guarded reset and a second activation edge."""

        application = self.build_application()
        first_id = self.activate_mission(
            application
        )
        first_trace = self.complete_active_mission(
            application
        )
        first_events = (
            application.monitoring.events_for(first_id)
        )

        application.perception.set_target_active(False)
        application.runner.reset()

        reset_status = application.runner.get_status()

        self.assertIs(
            reset_status.brain_state,
            BrainState.INITIALIZATION,
        )
        self.assertIsNone(reset_status.mission_id)
        self.assertEqual(
            application.clock.monotonic(),
            0.0,
        )
        self.assertEqual(
            application.memory.events,
            (),
        )
        self.assertEqual(
            application.monitoring.events_for(first_id),
            first_events,
        )

        second_id = self.activate_mission(
            application
        )
        second_trace = self.complete_active_mission(
            application
        )
        second_events = (
            application.monitoring.events_for(second_id)
        )

        self.assertNotEqual(second_id, first_id)
        self.assertTrue(first_id.endswith("-mission-1"))
        self.assertTrue(second_id.endswith("-mission-2"))
        self.assertEqual(
            first_trace.events,
            first_events,
        )
        self.assertEqual(
            second_trace.events,
            second_events,
        )
        self.assertEqual(
            tuple(
                event.sequence_number
                for event in second_events
            ),
            tuple(
                range(1, len(second_events) + 1)
            ),
        )

        renderer = application.renderer

        self.assertIsInstance(
            renderer,
            HeadlessRenderer,
        )
        assert isinstance(renderer, HeadlessRenderer)
        self.assertEqual(
            renderer.displayed_events,
            (*first_events, *second_events),
        )

    def test_nfr_10_identical_inputs_replay_identically(
        self,
    ) -> None:
        """Verify complete deterministic application replay."""

        first = self.build_application()
        first_id = self.activate_mission(first)
        first_trace = self.complete_active_mission(
            first
        )

        second = self.build_application()
        second_id = self.activate_mission(second)
        second_trace = self.complete_active_mission(
            second
        )

        self.assertEqual(second_id, first_id)
        self.assertEqual(second_trace, first_trace)


if __name__ == "__main__":
    unittest.main()
