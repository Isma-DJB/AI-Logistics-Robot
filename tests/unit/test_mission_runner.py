"""Unit tests for deterministic MissionRunner lifecycle."""

import unittest
from collections.abc import Callable
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path

from ai_logistics_robot.adapters.monitoring import InMemoryMonitoring
from ai_logistics_robot.adapters.simulation import (
    GridWorld,
    HeadlessRenderer,
)
from ai_logistics_robot.app import MissionRunner
from ai_logistics_robot.app.settings import Settings, load_settings
from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    BrainState,
    FailureReason,
    MissionStatus,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvalidStateTransitionError,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap


class RecordingSimulation:
    """Record reset operations while delegating to GridWorld."""

    def __init__(
        self,
        grid_world: GridWorld,
    ) -> None:
        self.grid_world = grid_world
        self.reset_count = 0

    @property
    def current_pose(self) -> RobotPose:
        """Return the delegated confirmed pose."""

        return self.grid_world.current_pose

    def reset(self) -> None:
        """Record and delegate one platform reset."""

        self.reset_count += 1
        self.grid_world.reset()

    def read_world(self) -> GridMap:
        """Return the delegated immutable world."""

        return self.grid_world.read_world()

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Delegate one command."""

        return self.grid_world.apply_command(command)

    def advance_time(self, seconds: float) -> None:
        """Delegate deterministic time advancement."""

        self.grid_world.advance_time(seconds)


class RecordingControl:
    """Provide controllable safety state for runner tests."""

    def __init__(
        self,
        *,
        robot_id: str,
        timestamp: datetime,
        operations: list[str],
    ) -> None:
        self.robot_id = robot_id
        self.timestamp = timestamp
        self.operations = operations
        self.stop_count = 0
        self.rearm_count = 0
        self.safety_status = SafetyStatus(
            robot_id=robot_id,
            updated_at=timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )

    def execute_step(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Reject unexpected direct execution in runner tests."""

        raise AssertionError(
            f"MissionRunner executed an unexpected command: {command}"
        )

    def stop(self) -> None:
        """Record one controlled stop request."""

        self.operations.append("control.stop")
        self.stop_count += 1

    def emergency_stop(
        self,
        reason: FailureReason,
    ) -> SafetyStatus:
        """Latch safety before recording the delegated stop."""

        self.operations.append("control.emergency_stop")
        self.safety_status = SafetyStatus(
            robot_id=self.robot_id,
            updated_at=self.timestamp,
            latched=True,
            severity=SafetySeverity.CRITICAL,
            reason=reason,
        )
        self.stop()
        return self.safety_status

    def get_safety_status(self) -> SafetyStatus:
        """Return the current controlled safety state."""

        return self.safety_status

    def reset_safety_latch(self) -> SafetyStatus:
        """Record explicit manual safety rearm."""

        self.operations.append("control.reset_safety_latch")
        self.rearm_count += 1
        self.safety_status = SafetyStatus(
            robot_id=self.robot_id,
            updated_at=self.timestamp,
            latched=False,
            severity=SafetySeverity.INFO,
        )
        return self.safety_status


class RecordingBrain:
    """Expose deterministic status and operation recording."""

    def __init__(
        self,
        *,
        initial_pose: RobotPose,
        control: RecordingControl,
        timestamp: datetime,
        operations: list[str],
    ) -> None:
        self.initial_pose = initial_pose
        self.control = control
        self.timestamp = timestamp
        self.operations = operations
        self.update_count = 0
        self.reset_count = 0
        self.on_update: Callable[[], None] | None = None
        self.status = self._status(
            brain_state=BrainState.INITIALIZATION,
        )

    def _status(
        self,
        *,
        brain_state: BrainState,
        pose: RobotPose | None = None,
        mission_id: str | None = None,
        mission_status: MissionStatus | None = None,
        latest_error: FailureReason | None = None,
    ) -> SystemStatus:
        """Build one internally consistent fake Brain status."""

        return SystemStatus(
            robot_id="robot_1",
            observed_at=self.timestamp,
            brain_state=brain_state,
            robot_pose=(
                self.initial_pose
                if pose is None
                else pose
            ),
            safety_status=self.control.get_safety_status(),
            mission_id=mission_id,
            mission_status=mission_status,
            latest_error=latest_error,
        )

    def set_active_mission(
        self,
        *,
        pose: RobotPose | None = None,
    ) -> None:
        """Expose one active mission status."""

        self.status = self._status(
            brain_state=BrainState.OUTBOUND_NAVIGATION,
            pose=pose,
            mission_id="mission_1",
            mission_status=MissionStatus.ACTIVE,
        )

    def update(self) -> None:
        """Record one cycle and observe priority safety."""

        self.operations.append("brain.update")
        self.update_count += 1

        safety_status = self.control.get_safety_status()

        if safety_status.latched:
            current = self.get_status()
            mission_id = current.mission_id

            self.status = self._status(
                brain_state=BrainState.SAFETY_STOP,
                pose=current.robot_pose,
                mission_id=mission_id,
                mission_status=(
                    MissionStatus.ABORTED
                    if mission_id is not None
                    else None
                ),
                latest_error=safety_status.reason,
            )

        if self.on_update is not None:
            self.on_update()

    def get_status(self) -> SystemStatus:
        """Return status with current Control safety."""

        return replace(
            self.status,
            safety_status=self.control.get_safety_status(),
        )

    def reset(self) -> None:
        """Record and restore temporary Brain state."""

        self.operations.append("brain.reset")
        self.reset_count += 1
        self.status = self._status(
            brain_state=BrainState.INITIALIZATION,
        )


class MissionRunnerTests(unittest.TestCase):
    """Verify configuration, loop, rendering, and safety lifecycle."""

    def setUp(self) -> None:
        self.project_root = Path(__file__).resolve().parents[2]
        self.settings: Settings = load_settings(
            self.project_root
            / "configs"
            / "simulation.yaml"
        )
        self.timestamp = datetime(
            2026,
            8,
            16,
            12,
            0,
            tzinfo=UTC,
        )
        self.operations: list[str] = []
        self.simulation = RecordingSimulation(
            GridWorld(
                world=self.settings.grid_map,
                robot_id=self.settings.robot.robot_id,
                initial_pose=self.settings.robot.initial_pose,
            )
        )
        self.control = RecordingControl(
            robot_id=self.settings.robot.robot_id,
            timestamp=self.timestamp,
            operations=self.operations,
        )
        self.brain = RecordingBrain(
            initial_pose=self.settings.robot.initial_pose,
            control=self.control,
            timestamp=self.timestamp,
            operations=self.operations,
        )
        self.monitoring = InMemoryMonitoring()
        self.renderer = HeadlessRenderer()
        self.runner = self.build_runner()
        self.runner.configure(self.settings)

    def build_runner(
        self,
        **overrides: object,
    ) -> MissionRunner:
        """Build one runner with optional invalid dependencies."""

        dependencies: dict[str, object] = {
            "brain": self.brain,
            "control": self.control,
            "simulation": self.simulation,
            "monitoring": self.monitoring,
            "renderer": self.renderer,
        }
        dependencies.update(overrides)

        return MissionRunner(
            **dependencies,  # type: ignore[arg-type]
        )

    def event(
        self,
        *,
        sequence_number: int,
        mission_id: str = "mission_1",
    ) -> MissionEvent:
        """Build one immutable event for forwarding tests."""

        return MissionEvent(
            event_id=(
                f"{mission_id}_event_{sequence_number}"
            ),
            sequence_number=sequence_number,
            mission_id=mission_id,
            robot_id=self.settings.robot.robot_id,
            occurred_at=self.timestamp,
            source="brain",
            name=f"event_{sequence_number}",
            brain_state=BrainState.OUTBOUND_NAVIGATION,
        )

    def test_constructor_rejects_invalid_dependencies(
        self,
    ) -> None:
        fields = (
            "brain",
            "control",
            "simulation",
            "monitoring",
            "renderer",
        )

        for field in fields:
            with self.subTest(field=field):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.build_runner(
                        **{field: None},
                    )

    def test_configuration_is_required_and_read_only(
        self,
    ) -> None:
        unconfigured = self.build_runner()

        self.assertFalse(unconfigured.configured)
        self.assertFalse(unconfigured.running)

        with self.assertRaises(
            InvalidStateTransitionError
        ):
            unconfigured.start(maximum_cycles=1)

        with self.assertRaises(DomainValidationError):
            unconfigured.configure(  # type: ignore[arg-type]
                None
            )

        self.assertFalse(unconfigured.configured)

        unconfigured.configure(self.settings)

        self.assertTrue(unconfigured.configured)
        self.assertFalse(unconfigured.running)

    def test_configuration_rejects_world_mismatch(
        self,
    ) -> None:
        different_world = GridMap(
            width=2,
            height=2,
            cell_size_cm=20,
            origin=Position(x=0, y=0),
            base_position=Position(x=0, y=0),
            target_position=Position(x=1, y=1),
            obstacles=frozenset(),
        )
        different_simulation = RecordingSimulation(
            GridWorld(
                world=different_world,
                robot_id=self.settings.robot.robot_id,
                initial_pose=RobotPose(
                    position=Position(x=0, y=0),
                    heading=self.settings.robot.initial_pose.heading,
                ),
            )
        )
        runner = self.build_runner(
            simulation=different_simulation
        )

        with self.assertRaises(DomainValidationError):
            runner.configure(self.settings)

        self.assertFalse(runner.configured)

    def test_bounded_start_runs_exact_cycle_count(
        self,
    ) -> None:
        status = self.runner.start(
            maximum_cycles=3
        )

        self.assertEqual(
            status,
            self.brain.get_status(),
        )
        self.assertEqual(
            self.brain.update_count,
            3,
        )
        self.assertEqual(
            len(self.renderer.rendered_frames),
            3,
        )
        self.assertTrue(
            all(
                world is self.settings.grid_map
                for world, _ in self.renderer.rendered_frames
            )
        )
        self.assertFalse(self.runner.running)
        self.assertEqual(
            self.control.stop_count,
            0,
        )

    def test_cycle_bound_must_be_positive_integer_or_none(
        self,
    ) -> None:
        for maximum_cycles in (
            0,
            -1,
            True,
            1.5,
            "1",
        ):
            with self.subTest(
                maximum_cycles=maximum_cycles
            ):
                with self.assertRaises(
                    DomainValidationError
                ):
                    self.runner.start(
                        maximum_cycles=(  # type: ignore[arg-type]
                            maximum_cycles
                        )
                    )

        self.assertEqual(
            self.brain.update_count,
            0,
        )
        self.assertEqual(
            self.renderer.rendered_frames,
            (),
        )

    def test_unbounded_start_stops_on_explicit_request(
        self,
    ) -> None:
        def stop_after_third_update() -> None:
            if self.brain.update_count == 3:
                self.runner.stop()

        self.brain.on_update = stop_after_third_update

        self.runner.start()

        self.assertEqual(
            self.brain.update_count,
            3,
        )
        self.assertEqual(
            len(self.renderer.rendered_frames),
            3,
        )
        self.assertEqual(
            self.control.stop_count,
            1,
        )
        self.assertFalse(self.runner.running)

    def test_get_status_has_no_execution_effect(
        self,
    ) -> None:
        expected = self.brain.get_status()

        first = self.runner.get_status()
        second = self.runner.get_status()

        self.assertEqual(first, expected)
        self.assertEqual(second, expected)
        self.assertEqual(
            self.brain.update_count,
            0,
        )
        self.assertEqual(
            self.operations,
            [],
        )
        self.assertEqual(
            self.renderer.rendered_frames,
            (),
        )

    def test_new_events_are_displayed_once_in_order(
        self,
    ) -> None:
        self.brain.set_active_mission()
        first = self.event(sequence_number=1)
        second = self.event(sequence_number=2)

        self.monitoring.publish(first)

        self.runner.start(maximum_cycles=1)
        self.runner.start(maximum_cycles=1)

        self.monitoring.publish(second)
        self.runner.start(maximum_cycles=1)

        self.assertEqual(
            self.renderer.displayed_events,
            (first, second),
        )
        self.assertIs(
            self.renderer.displayed_events[0],
            first,
        )

    def test_emergency_stop_precedes_brain_transition(
        self,
    ) -> None:
        self.brain.set_active_mission()

        status = self.runner.request_emergency_stop(
            FailureReason.EMERGENCY_STOP
        )

        self.assertEqual(
            self.operations[:3],
            [
                "control.emergency_stop",
                "control.stop",
                "brain.update",
            ],
        )
        self.assertIs(
            status.brain_state,
            BrainState.SAFETY_STOP,
        )
        self.assertIs(
            status.mission_status,
            MissionStatus.ABORTED,
        )
        self.assertIs(
            status.latest_error,
            FailureReason.EMERGENCY_STOP,
        )
        self.assertTrue(status.safety_status.latched)
        self.assertFalse(self.runner.running)
        self.assertEqual(
            len(self.renderer.rendered_frames),
            1,
        )

    def test_invalid_emergency_reason_has_no_effect(
        self,
    ) -> None:
        with self.assertRaises(DomainValidationError):
            self.runner.request_emergency_stop(  # type: ignore[arg-type]
                None
            )

        self.assertEqual(self.operations, [])
        self.assertFalse(
            self.control.get_safety_status().latched
        )

    def test_rearm_is_manual_and_separate_from_reset(
        self,
    ) -> None:
        self.brain.set_active_mission()

        self.runner.request_emergency_stop(
            FailureReason.EMERGENCY_STOP
        )

        with self.assertRaises(
            InvalidStateTransitionError
        ):
            self.runner.reset()

        rearmed = self.runner.request_safety_rearm()

        self.assertFalse(rearmed.latched)
        self.assertEqual(
            self.control.rearm_count,
            1,
        )
        self.assertEqual(
            self.brain.reset_count,
            0,
        )
        self.assertIs(
            self.runner.get_status().brain_state,
            BrainState.SAFETY_STOP,
        )

        with self.assertRaises(
            InvalidStateTransitionError
        ):
            self.runner.start(maximum_cycles=1)

        self.runner.reset()

        reset_status = self.runner.get_status()

        self.assertEqual(
            self.brain.reset_count,
            1,
        )
        self.assertEqual(
            self.simulation.reset_count,
            1,
        )
        self.assertIs(
            reset_status.brain_state,
            BrainState.INITIALIZATION,
        )
        self.assertIsNone(reset_status.mission_id)
        self.assertFalse(
            reset_status.safety_status.latched
        )

    def test_rearm_requires_latched_safety_stop(
        self,
    ) -> None:
        with self.assertRaises(
            InvalidStateTransitionError
        ):
            self.runner.request_safety_rearm()

        self.assertEqual(
            self.control.rearm_count,
            0,
        )

    def test_reset_rejects_unconfirmed_initial_pose(
        self,
    ) -> None:
        initial_position = (
            self.settings.robot.initial_pose.position
        )
        outside_initial_pose = RobotPose(
            position=Position(
                x=initial_position.x + 1,
                y=initial_position.y,
            ),
            heading=self.settings.robot.initial_pose.heading,
        )
        self.brain.status = self.brain._status(
            brain_state=BrainState.MISSION_FAILED,
            pose=outside_initial_pose,
            mission_id="mission_1",
            mission_status=MissionStatus.FAILED,
            latest_error=FailureReason.BLOCKED,
        )

        with self.assertRaises(
            InvalidStateTransitionError
        ):
            self.runner.reset()

        self.assertEqual(
            self.brain.reset_count,
            0,
        )
        self.assertEqual(
            self.simulation.reset_count,
            0,
        )

    def test_reset_is_rejected_while_runner_is_active(
        self,
    ) -> None:
        reset_rejected = False

        def attempt_reset_and_stop() -> None:
            nonlocal reset_rejected

            try:
                self.runner.reset()
            except InvalidStateTransitionError:
                reset_rejected = True

            self.runner.stop()

        self.brain.on_update = attempt_reset_and_stop

        self.runner.start()

        self.assertTrue(reset_rejected)
        self.assertEqual(
            self.brain.reset_count,
            0,
        )
        self.assertEqual(
            self.simulation.reset_count,
            0,
        )


if __name__ == "__main__":
    unittest.main()
