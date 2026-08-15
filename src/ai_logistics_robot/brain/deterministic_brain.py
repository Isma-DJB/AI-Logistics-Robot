"""Deterministic platform-independent mission orchestration."""

from dataclasses import replace
from math import isfinite

from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    BrainState,
    CommandStatus,
    CommandType,
    FailureReason,
    Heading,
    MissionStatus,
    PathPhase,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.domain.paths import (
    PathPlan,
    PathRecord,
)
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports.clock_port import ClockPort
from ai_logistics_robot.ports.control_port import ControlPort
from ai_logistics_robot.ports.memory_port import MemoryPort
from ai_logistics_robot.ports.monitoring_port import MonitoringPort
from ai_logistics_robot.ports.perception_port import PerceptionPort
from ai_logistics_robot.ports.planning_port import PlanningPort

_CARDINAL_HEADINGS = (
    Heading.NORTH,
    Heading.EAST,
    Heading.SOUTH,
    Heading.WEST,
)


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
        "_configured_world",
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
        "_navigation_index",
        "_perception",
        "_plan_version",
        "_planning",
        "_replan_count",
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
        self._configured_world = world
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

        if self._state is BrainState.OUTBOUND_PLANNING:
            self._create_outbound_plan()
            return

        if self._state is BrainState.OUTBOUND_NAVIGATION:
            self._navigate_outbound()
            return

        if self._state is BrainState.OUTBOUND_REPLANNING:
            self._replan_outbound()
            return

        if self._state is BrainState.COLLECTION:
            self._perform_collection()
            return

        if self._state is BrainState.RETURN_PREPARATION:
            self._prepare_return()
            return

        if self._state is BrainState.RETURN_NAVIGATION:
            self._navigate_return()
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
        self._world = self._configured_world
        self._current_pose = self._initial_pose
        self._mission: Mission | None = None
        self._active_plan: PathPlan | None = None
        self._latest_error: FailureReason | None = None
        self._target_was_active: bool | None = None
        self._event_sequence = 0
        self._navigation_index = 0
        self._plan_version = 1
        self._replan_count = 0

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
        self._navigation_index = 0
        self._plan_version = 1
        self._replan_count = 0
        self._state = BrainState.OUTBOUND_PLANNING
        self._record_mission_event("mission_started")

    def _create_outbound_plan(self) -> None:
        """Create the initial path to an authorized arrival cell."""

        self._create_navigation_plan(
            phase=PathPhase.OUTBOUND,
            event_name="outbound_plan_created",
        )

    def _replan_outbound(self) -> None:
        """Create a detour using the revised immutable map."""

        self._replan_count += 1
        self._plan_version += 1

        self._create_navigation_plan(
            phase=PathPhase.DETOUR,
            event_name="outbound_plan_recreated",
        )

    def _create_navigation_plan(
        self,
        *,
        phase: PathPhase,
        event_name: str,
    ) -> None:
        """Create and accept one validated outbound plan."""

        mission = self._require_active_mission()
        authorized_goals = (
            self._world.authorized_arrival_positions
        )
        plan = self._planning.create_plan(
            mission_id=mission.mission_id,
            robot_id=self._robot_id,
            start_pose=self._current_pose,
            authorized_goals=authorized_goals,
            world=self._world,
            phase=phase,
            version=self._plan_version,
        )

        self._validate_navigation_plan(
            plan,
            phase=phase,
            authorized_goals=authorized_goals,
        )

        self._active_plan = plan
        self._navigation_index = 1
        self._state = BrainState.OUTBOUND_NAVIGATION
        self._record_mission_event(event_name)

    def _validate_navigation_plan(
        self,
        plan: object,
        *,
        phase: PathPhase,
        authorized_goals: tuple[Position, ...],
    ) -> None:
        """Require an identity-matched plan from the current pose."""

        if not isinstance(plan, PathPlan):
            raise DomainValidationError(
                "planning must return a PathPlan."
            )

        mission = self._require_active_mission()

        if plan.mission_id != mission.mission_id:
            raise InvariantViolationError(
                "plan mission_id must match the active mission."
            )

        if plan.robot_id != self._robot_id:
            raise InvariantViolationError(
                "plan robot_id must match the configured robot."
            )

        if plan.phase is not phase:
            raise InvariantViolationError(
                "plan phase must match the requested phase."
            )

        if plan.version != self._plan_version:
            raise InvariantViolationError(
                "plan version must match the requested version."
            )

        if plan.positions[0] != self._current_pose.position:
            raise InvariantViolationError(
                "the first planned position must equal "
                "the current confirmed position."
            )

        if plan.goal not in authorized_goals:
            raise InvariantViolationError(
                "the plan goal must be an authorized "
                "arrival position."
            )

    def _navigate_outbound(self) -> None:
        """Observe and execute at most one outbound command."""

        snapshot = self._perception.observe()
        self._accept_snapshot(snapshot)

        if snapshot.hazard_detected:
            self._control.emergency_stop(
                FailureReason.EMERGENCY_STOP
            )
            self._state = BrainState.SAFETY_STOP
            return

        plan = self._require_active_plan()

        if self._current_pose.position == plan.goal:
            self._enter_collection()
            return

        while (
            self._navigation_index < len(plan.positions)
            and plan.positions[self._navigation_index]
            == self._current_pose.position
        ):
            self._navigation_index += 1

        if self._navigation_index >= len(plan.positions):
            raise InvariantViolationError(
                "the active plan ended before its goal "
                "was confirmed."
            )

        intended_position = plan.positions[
            self._navigation_index
        ]
        command = self._command_toward(intended_position)
        result = self._control.execute_step(command)

        self._accept_navigation_result(
            result,
            command=command,
            intended_position=intended_position,
        )

    def _command_toward(
        self,
        intended_position: Position,
    ) -> MotionCommand:
        """Return one deterministic command toward an adjacent cell."""

        current_position = self._current_pose.position
        delta_x = intended_position.x - current_position.x
        delta_y = intended_position.y - current_position.y

        if abs(delta_x) + abs(delta_y) != 1:
            raise InvariantViolationError(
                "successive planned positions must be "
                "cardinally adjacent."
            )

        if delta_x == 1:
            desired_heading = Heading.EAST
        elif delta_x == -1:
            desired_heading = Heading.WEST
        elif delta_y == 1:
            desired_heading = Heading.NORTH
        else:
            desired_heading = Heading.SOUTH

        current_index = _CARDINAL_HEADINGS.index(
            self._current_pose.heading
        )
        desired_index = _CARDINAL_HEADINGS.index(
            desired_heading
        )
        right_turns = (
            desired_index - current_index
        ) % len(_CARDINAL_HEADINGS)

        if right_turns == 0:
            command_type = CommandType.MOVE_FORWARD
        elif right_turns in (1, 2):
            command_type = CommandType.TURN_RIGHT
        else:
            command_type = CommandType.TURN_LEFT

        return MotionCommand(
            robot_id=self._robot_id,
            command_type=command_type,
        )

    def _accept_navigation_result(
        self,
        result: object,
        *,
        command: MotionCommand,
        intended_position: Position,
    ) -> None:
        """Accept one confirmed movement or begin replanning."""

        if not isinstance(result, CommandResult):
            raise DomainValidationError(
                "control must return a CommandResult."
            )

        if result.command is not command:
            raise InvariantViolationError(
                "control result must retain the supplied command."
            )

        if result.pose_before != self._current_pose:
            raise InvariantViolationError(
                "control result must begin at the current pose."
            )

        if result.status is CommandStatus.SUCCESS:
            self._accept_successful_navigation_result(
                result,
                command=command,
                intended_position=intended_position,
            )
            return

        if (
            result.status is CommandStatus.FAILED
            and result.failure_reason is FailureReason.BLOCKED
        ):
            self._accept_blocked_navigation_result(
                result,
                command=command,
                intended_position=intended_position,
            )
            return

        if (
            result.status is CommandStatus.ABORTED
            and result.failure_reason
            is FailureReason.SAFETY_LATCHED
        ):
            self._latest_error = FailureReason.SAFETY_LATCHED
            self._state = BrainState.SAFETY_STOP
            return

        raise InvariantViolationError(
            "outbound navigation received an unsupported "
            "command outcome."
        )

    def _accept_successful_navigation_result(
        self,
        result: CommandResult,
        *,
        command: MotionCommand,
        intended_position: Position,
    ) -> None:
        """Record one successful confirmed navigation pose."""

        if command.command_type is CommandType.MOVE_FORWARD:
            if result.pose_after.position != intended_position:
                raise InvariantViolationError(
                    "a successful forward command must reach "
                    "the intended position."
                )
        elif (
            result.pose_after.position
            != self._current_pose.position
        ):
            raise InvariantViolationError(
                "a successful turn must preserve position."
            )

        self._current_pose = result.pose_after
        self._memory.record_pose(
            PathPhase.OUTBOUND,
            self._current_pose,
        )
        self._record_mission_event(
            "outbound_step_confirmed"
        )

        if command.command_type is CommandType.MOVE_FORWARD:
            self._navigation_index += 1

        plan = self._require_active_plan()

        if self._current_pose.position == plan.goal:
            self._enter_collection()

    def _accept_blocked_navigation_result(
        self,
        result: CommandResult,
        *,
        command: MotionCommand,
        intended_position: Position,
    ) -> None:
        """Preserve pose and update the map before replanning."""

        if command.command_type is not CommandType.MOVE_FORWARD:
            raise InvariantViolationError(
                "only a forward movement may report BLOCKED."
            )

        if (
            result.pose_after != self._current_pose
            or result.pose_before != result.pose_after
        ):
            raise InvariantViolationError(
                "a blocked movement must preserve "
                "the confirmed pose."
            )

        revised_obstacles = (
            self._world.obstacles
            | frozenset({intended_position})
        )
        self._world = replace(
            self._world,
            obstacles=revised_obstacles,
        )
        self._active_plan = None
        self._navigation_index = 0
        self._latest_error = FailureReason.BLOCKED
        self._state = BrainState.OUTBOUND_REPLANNING
        self._record_mission_event(
            "outbound_step_blocked"
        )

    def _enter_collection(self) -> None:
        """Confirm safe arrival without waiting in this cycle."""

        self._active_plan = None
        self._navigation_index = 0
        self._state = BrainState.COLLECTION
        self._record_mission_event("arrival_confirmed")

    def _perform_collection(self) -> None:
        """Remain stationary for the configured collection duration."""

        mission = self._require_active_mission()
        self._control.stop()

        deadline = (
            self._clock.monotonic()
            + self._collection_duration_s
        )

        if not isfinite(deadline):
            raise InvariantViolationError(
                "the collection deadline must be finite."
            )

        self._clock.wait_until(deadline)
        self._mission = replace(
            mission,
            collection_completed=True,
        )
        self._state = BrainState.RETURN_PREPARATION
        self._record_mission_event(
            "collection_completed"
        )

    def _prepare_return(self) -> None:
        """Build the exact reversed confirmed outbound route."""

        mission = self._require_active_mission()
        path_record = self._memory.build_return_path()

        if not isinstance(path_record, PathRecord):
            raise DomainValidationError(
                "memory must return a PathRecord."
            )

        if path_record.mission_id != mission.mission_id:
            raise InvariantViolationError(
                "return path mission_id must match "
                "the active mission."
            )

        if path_record.robot_id != self._robot_id:
            raise InvariantViolationError(
                "return path robot_id must match "
                "the configured robot."
            )

        if path_record.phase is not PathPhase.RETURN:
            raise InvariantViolationError(
                "return path phase must be RETURN."
            )

        positions = path_record.confirmed_positions

        if not positions:
            raise InvariantViolationError(
                "return preparation requires confirmed "
                "outbound history."
            )

        if positions[0] != self._current_pose.position:
            raise InvariantViolationError(
                "the reversed outbound record must begin "
                "at the current confirmed position."
            )

        if positions[-1] != mission.base_position:
            raise InvariantViolationError(
                "the reversed outbound record must end "
                "at the mission base."
            )

        self._plan_version = 1
        self._active_plan = PathPlan(
            mission_id=mission.mission_id,
            robot_id=self._robot_id,
            phase=PathPhase.RETURN,
            version=self._plan_version,
            positions=positions,
            goal=mission.base_position,
        )
        self._navigation_index = 1
        self._state = BrainState.RETURN_NAVIGATION
        self._record_mission_event(
            "return_path_prepared"
        )

    def _navigate_return(self) -> None:
        """Observe and execute at most one return command."""

        snapshot = self._perception.observe()
        self._accept_snapshot(snapshot)

        if snapshot.hazard_detected:
            self._control.emergency_stop(
                FailureReason.EMERGENCY_STOP
            )
            self._state = BrainState.SAFETY_STOP
            return

        plan = self._require_active_plan()

        if plan.phase is not PathPhase.RETURN:
            raise InvariantViolationError(
                "return navigation requires a RETURN plan."
            )

        if self._current_pose.position == plan.goal:
            self._enter_mission_completed()
            return

        while (
            self._navigation_index < len(plan.positions)
            and plan.positions[self._navigation_index]
            == self._current_pose.position
        ):
            self._navigation_index += 1

        if self._navigation_index >= len(plan.positions):
            raise InvariantViolationError(
                "the return plan ended before base arrival "
                "was confirmed."
            )

        intended_position = plan.positions[
            self._navigation_index
        ]
        command = self._command_toward(intended_position)
        result = self._control.execute_step(command)

        self._accept_return_navigation_result(
            result,
            command=command,
            intended_position=intended_position,
        )

    def _accept_return_navigation_result(
        self,
        result: object,
        *,
        command: MotionCommand,
        intended_position: Position,
    ) -> None:
        """Accept and record one successful return command."""

        if not isinstance(result, CommandResult):
            raise DomainValidationError(
                "control must return a CommandResult."
            )

        if result.command is not command:
            raise InvariantViolationError(
                "control result must retain the supplied command."
            )

        if result.pose_before != self._current_pose:
            raise InvariantViolationError(
                "control result must begin at the current pose."
            )

        if result.status is not CommandStatus.SUCCESS:
            raise InvariantViolationError(
                "nominal return navigation requires "
                "a successful command result."
            )

        if command.command_type is CommandType.MOVE_FORWARD:
            if result.pose_after.position != intended_position:
                raise InvariantViolationError(
                    "a successful return movement must reach "
                    "the intended position."
                )
        elif (
            result.pose_after.position
            != self._current_pose.position
        ):
            raise InvariantViolationError(
                "a successful return turn must preserve position."
            )

        self._current_pose = result.pose_after
        self._memory.record_pose(
            PathPhase.RETURN,
            self._current_pose,
        )
        self._record_mission_event(
            "return_step_confirmed"
        )

        if command.command_type is CommandType.MOVE_FORWARD:
            self._navigation_index += 1

        plan = self._require_active_plan()

        if self._current_pose.position == plan.goal:
            self._enter_mission_completed()

    def _enter_mission_completed(self) -> None:
        """Confirm base arrival before terminal completion."""

        mission = self._require_active_mission()

        if not mission.collection_completed:
            raise InvariantViolationError(
                "base arrival cannot precede collection."
            )

        self._mission = replace(
            mission,
            base_arrival_confirmed=True,
        )
        self._active_plan = None
        self._navigation_index = 0
        self._state = BrainState.MISSION_COMPLETED
        self._record_mission_event(
            "base_arrival_confirmed"
        )
    def _require_active_mission(self) -> Mission:
        """Return the active mission or reject invalid state."""

        if self._mission is None:
            raise InvariantViolationError(
                "the current state requires an active mission."
            )

        return self._mission

    def _require_active_plan(self) -> PathPlan:
        """Return the active plan or reject invalid state."""

        if self._active_plan is None:
            raise InvariantViolationError(
                "navigation requires an active plan."
            )

        return self._active_plan

    def _record_mission_event(
        self,
        name: str,
    ) -> MissionEvent:
        """Create, store, and publish one ordered event."""

        mission = self._require_active_mission()

        self._event_sequence += 1
        event = MissionEvent(
            event_id=(
                f"{mission.mission_id}-event-"
                f"{self._event_sequence}"
            ),
            sequence_number=self._event_sequence,
            mission_id=mission.mission_id,
            robot_id=self._robot_id,
            occurred_at=self._clock.now(),
            source="brain",
            name=name,
            brain_state=self._state,
        )

        self._memory.record_event(event)
        self._monitoring.publish(event)
        return event
