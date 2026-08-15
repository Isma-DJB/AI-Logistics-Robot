"""Deterministic platform-independent mission orchestration."""

from math import isfinite

from ai_logistics_robot.domain.enums import (
    BrainState,
    FailureReason,
    MissionStatus,
    PathPhase,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports.clock_port import ClockPort
from ai_logistics_robot.ports.control_port import ControlPort
from ai_logistics_robot.ports.memory_port import MemoryPort
from ai_logistics_robot.ports.monitoring_port import MonitoringPort
from ai_logistics_robot.ports.perception_port import PerceptionPort
from ai_logistics_robot.ports.planning_port import PlanningPort


def _validate_identifier(name: str, value: object) -> None:
    """Require a non-empty string identifier."""

    if not isinstance(value, str) or not value.strip():
        raise DomainValidationError(
            f"{name} must be a non-empty string."
        )


def _validate_non_negative_number(
    name: str,
    value: object,
    *,
    allow_none: bool,
) -> float | None:
    """Return a finite non-negative number or allowed None."""

    if value is None and allow_none:
        return None

    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
    ):
        raise DomainValidationError(
            f"{name} must be a finite non-negative number."
        )

    normalized = float(value)

    if not isfinite(normalized) or normalized < 0:
        raise DomainValidationError(
            f"{name} must be a finite non-negative number."
        )

    return normalized


def _validate_optional_positive_integer(
    name: str,
    value: object,
) -> int | None:
    """Return a positive integer or None."""

    if value is None:
        return None

    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < 1
    ):
        raise DomainValidationError(
            f"{name} must be a positive integer or None."
        )

    return value


class DeterministicBrain:
    """Advance one deterministic mission state action per update."""

    __slots__ = (
        "_active_plan",
        "_clock",
        "_collection_duration_s",
        "_control",
        "_current_pose",
        "_event_sequence",
        "_initial_pose",
        "_latest_error",
        "_maximum_replans",
        "_memory",
        "_mission",
        "_mission_counter",
        "_monitoring",
        "_perception",
        "_planning",
        "_robot_id",
        "_scenario_id",
        "_state",
        "_target_id",
        "_target_was_active",
        "_timeout_s",
        "_world",
    )

    def __init__(
        self,
        *,
        scenario_id: str,
        robot_id: str,
        target_id: str,
        world: GridMap,
        initial_pose: RobotPose,
        collection_duration_s: float,
        maximum_replans: int | None,
        timeout_s: float | None,
        perception: PerceptionPort,
        planning: PlanningPort,
        control: ControlPort,
        memory: MemoryPort,
        monitoring: MonitoringPort,
        clock: ClockPort,
    ) -> None:
        """Validate configuration and initialize deterministic state."""

        _validate_identifier("scenario_id", scenario_id)
        _validate_identifier("robot_id", robot_id)
        _validate_identifier("target_id", target_id)

        if not isinstance(world, GridMap):
            raise DomainValidationError(
                "world must be a GridMap instance."
            )

        if not isinstance(initial_pose, RobotPose):
            raise DomainValidationError(
                "initial_pose must be a RobotPose instance."
            )

        if not world.is_traversable(initial_pose.position):
            raise DomainValidationError(
                "initial_pose must be traversable in the world."
            )

        normalized_collection_duration = (
            _validate_non_negative_number(
                "collection_duration_s",
                collection_duration_s,
                allow_none=False,
            )
        )
        normalized_maximum_replans = (
            _validate_optional_positive_integer(
                "maximum_replans",
                maximum_replans,
            )
        )
        normalized_timeout = _validate_non_negative_number(
            "timeout_s",
            timeout_s,
            allow_none=True,
        )

        if not isinstance(perception, PerceptionPort):
            raise DomainValidationError(
                "perception must satisfy PerceptionPort."
            )

        if not isinstance(planning, PlanningPort):
            raise DomainValidationError(
                "planning must satisfy PlanningPort."
            )

        if not isinstance(control, ControlPort):
            raise DomainValidationError(
                "control must satisfy ControlPort."
            )

        if not isinstance(memory, MemoryPort):
            raise DomainValidationError(
                "memory must satisfy MemoryPort."
            )

        if not isinstance(monitoring, MonitoringPort):
            raise DomainValidationError(
                "monitoring must satisfy MonitoringPort."
            )

        if not isinstance(clock, ClockPort):
            raise DomainValidationError(
                "clock must satisfy ClockPort."
            )

        if normalized_collection_duration is None:
            raise InvariantViolationError(
                "collection duration normalization failed."
            )

        self._scenario_id = scenario_id
        self._robot_id = robot_id
        self._target_id = target_id
        self._world = world
        self._initial_pose = initial_pose
        self._collection_duration_s = (
            normalized_collection_duration
        )
        self._maximum_replans = normalized_maximum_replans
        self._timeout_s = normalized_timeout
        self._perception = perception
        self._planning = planning
        self._control = control
        self._memory = memory
        self._monitoring = monitoring
        self._clock = clock
        self._mission_counter = 0

        self._restore_temporary_state()

    def update(self) -> None:
        """Perform one deterministic orchestration cycle."""

        safety_status = self._control.get_safety_status()

        if safety_status.latched:
            self._state = BrainState.SAFETY_STOP
            return

        if self._state is BrainState.INITIALIZATION:
            self._initialize()
            return

        if self._state is BrainState.WAITING_FOR_MISSION:
            self._wait_for_mission()
            return

    def get_status(self) -> SystemStatus:
        """Return an immutable status without executing control."""

        mission_id = (
            None
            if self._mission is None
            else self._mission.mission_id
        )
        mission_status = (
            None
            if self._mission is None
            else self._mission.status
        )

        return SystemStatus(
            robot_id=self._robot_id,
            observed_at=self._clock.now(),
            brain_state=self._state,
            robot_pose=self._current_pose,
            safety_status=self._control.get_safety_status(),
            mission_id=mission_id,
            mission_status=mission_status,
            active_plan=self._active_plan,
            latest_error=self._latest_error,
        )

    def reset(self) -> None:
        """Clear temporary Brain and Memory state without rearming."""

        self._memory.reset()
        self._restore_temporary_state()

    def _restore_temporary_state(self) -> None:
        """Restore replayable state while preserving mission identity."""

        self._state = BrainState.INITIALIZATION
        self._current_pose = self._initial_pose
        self._mission: Mission | None = None
        self._active_plan: PathPlan | None = None
        self._latest_error: FailureReason | None = None
        self._target_was_active: bool | None = None
        self._event_sequence = 0

    def _initialize(self) -> None:
        """Confirm stationary behavior before entering waiting."""

        self._control.stop()
        self._state = BrainState.WAITING_FOR_MISSION

    def _wait_for_mission(self) -> None:
        """Remain stationary and detect one valid activation edge."""

        self._control.stop()
        snapshot = self._perception.observe()
        self._accept_snapshot(snapshot)

        if snapshot.hazard_detected:
            self._control.emergency_stop(
                FailureReason.EMERGENCY_STOP
            )
            self._state = BrainState.SAFETY_STOP
            return

        previous_target_state = self._target_was_active
        self._target_was_active = snapshot.target_active

        if (
            previous_target_state is False
            and snapshot.target_active
        ):
            self._start_mission()

    def _accept_snapshot(
        self,
        snapshot: object,
    ) -> None:
        """Accept a valid identity-matched perception snapshot."""

        if not isinstance(snapshot, PerceptionSnapshot):
            raise DomainValidationError(
                "perception must return a PerceptionSnapshot."
            )

        if snapshot.robot_id != self._robot_id:
            raise InvariantViolationError(
                "perception snapshot must reference "
                "the configured robot."
            )

        self._current_pose = snapshot.robot_pose

    def _start_mission(self) -> None:
        """Create and record one deterministic active mission."""

        self._mission_counter += 1
        mission_id = (
            f"{self._scenario_id}-mission-"
            f"{self._mission_counter}"
        )
        mission = Mission(
            mission_id=mission_id,
            robot_id=self._robot_id,
            target_id=self._target_id,
            target_position=self._world.target_position,
            base_position=self._world.base_position,
            status=MissionStatus.ACTIVE,
        )

        self._memory.start(mission)
        self._memory.record_pose(
            PathPhase.OUTBOUND,
            self._current_pose,
        )

        self._mission = mission
        self._active_plan = None
        self._latest_error = None
        self._event_sequence = 0
        self._state = BrainState.OUTBOUND_PLANNING
        self._record_mission_event("mission_started")

    def _record_mission_event(
        self,
        name: str,
    ) -> MissionEvent:
        """Create, store, and publish one ordered event."""

        if self._mission is None:
            raise InvariantViolationError(
                "a mission event requires an active mission."
            )

        self._event_sequence += 1
        event = MissionEvent(
            event_id=(
                f"{self._mission.mission_id}-event-"
                f"{self._event_sequence}"
            ),
            sequence_number=self._event_sequence,
            mission_id=self._mission.mission_id,
            robot_id=self._robot_id,
            occurred_at=self._clock.now(),
            source="brain",
            name=name,
            brain_state=self._state,
        )

        self._memory.record_event(event)
        self._monitoring.publish(event)
        return event
