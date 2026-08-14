"""Deterministic in-memory recording for one mission at a time."""

from ai_logistics_robot.domain.enums import (
    MissionStatus,
    PathPhase,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvalidStateTransitionError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.mission import Mission
from ai_logistics_robot.domain.paths import PathRecord


class InMemoryMissionMemory:
    """Record one mission using immutable public snapshots."""

    def __init__(self) -> None:
        """Initialize an empty deterministic recording."""

        self._started_mission: Mission | None = None
        self._completed_mission: Mission | None = None
        self._outbound_poses: tuple[RobotPose, ...] = ()
        self._return_poses: tuple[RobotPose, ...] = ()
        self._events: tuple[MissionEvent, ...] = ()

    @property
    def active_mission(self) -> Mission | None:
        """Return the active mission while recording remains open."""

        if self._completed_mission is not None:
            return None

        return self._started_mission

    @property
    def completed_mission(self) -> Mission | None:
        """Return the stored terminal mission when completed."""

        return self._completed_mission

    @property
    def outbound_poses(self) -> tuple[RobotPose, ...]:
        """Return the immutable confirmed outbound history."""

        return self._outbound_poses

    @property
    def return_poses(self) -> tuple[RobotPose, ...]:
        """Return the immutable confirmed return history."""

        return self._return_poses

    @property
    def events(self) -> tuple[MissionEvent, ...]:
        """Return recorded events in accepted sequence order."""

        return self._events

    def start(self, mission: Mission) -> None:
        """Open recording for one created or active mission."""

        if not isinstance(mission, Mission):
            raise DomainValidationError(
                "mission must be a Mission instance."
            )

        if self._started_mission is not None:
            raise InvalidStateTransitionError(
                "memory must be reset before starting "
                "another mission."
            )

        if mission.status not in (
            MissionStatus.CREATED,
            MissionStatus.ACTIVE,
        ):
            raise InvalidStateTransitionError(
                "recording can start only for a created "
                "or active mission."
            )

        self._started_mission = mission

    def record_pose(
        self,
        phase: PathPhase,
        pose: RobotPose,
    ) -> None:
        """Record one confirmed pose in its navigation phase."""

        if not isinstance(phase, PathPhase):
            raise DomainValidationError(
                "phase must be a PathPhase instance."
            )

        if phase not in (
            PathPhase.OUTBOUND,
            PathPhase.RETURN,
        ):
            raise DomainValidationError(
                "recorded pose phase must be "
                "OUTBOUND or RETURN."
            )

        if not isinstance(pose, RobotPose):
            raise DomainValidationError(
                "pose must be a RobotPose instance."
            )

        self._require_open_mission()

        if phase is PathPhase.OUTBOUND:
            self._outbound_poses += (pose,)
        else:
            self._return_poses += (pose,)

    def record_event(self, event: MissionEvent) -> None:
        """Record one identity-matched event in sequence order."""

        if not isinstance(event, MissionEvent):
            raise DomainValidationError(
                "event must be a MissionEvent instance."
            )

        mission = self._require_open_mission()

        if (
            event.mission_id != mission.mission_id
            or event.robot_id != mission.robot_id
        ):
            raise DomainValidationError(
                "event identity must match the active mission."
            )

        if any(
            recorded.event_id == event.event_id
            for recorded in self._events
        ):
            raise InvariantViolationError(
                "event identifiers must be unique."
            )

        if (
            self._events
            and event.sequence_number
            <= self._events[-1].sequence_number
        ):
            raise InvariantViolationError(
                "event sequence numbers must increase."
            )

        self._events += (event,)

    def build_return_path(self) -> PathRecord:
        """Reverse the exact confirmed outbound pose history."""

        mission = self._require_started_mission()

        return PathRecord(
            mission_id=mission.mission_id,
            robot_id=mission.robot_id,
            phase=PathPhase.RETURN,
            confirmed_poses=tuple(
                reversed(self._outbound_poses)
            ),
        )

    def complete(self, mission: Mission) -> None:
        """Close recording with an identity-matched terminal mission."""

        if not isinstance(mission, Mission):
            raise DomainValidationError(
                "mission must be a Mission instance."
            )

        started_mission = self._require_open_mission()

        if mission.status not in (
            MissionStatus.SUCCESS,
            MissionStatus.FAILED,
            MissionStatus.ABORTED,
        ):
            raise InvalidStateTransitionError(
                "completed mission must have a terminal status."
            )

        if (
            mission.mission_id
            != started_mission.mission_id
            or mission.robot_id
            != started_mission.robot_id
            or mission.target_id
            != started_mission.target_id
            or mission.target_position
            != started_mission.target_position
            or mission.base_position
            != started_mission.base_position
        ):
            raise DomainValidationError(
                "completed mission identity and fixed geometry "
                "must match the started mission."
            )

        self._completed_mission = mission

    def reset(self) -> None:
        """Clear every mission, path, event, and result value."""

        self._started_mission = None
        self._completed_mission = None
        self._outbound_poses = ()
        self._return_poses = ()
        self._events = ()

    def _require_started_mission(self) -> Mission:
        """Return the started mission or reject missing state."""

        if self._started_mission is None:
            raise InvalidStateTransitionError(
                "a mission must be started before this operation."
            )

        return self._started_mission

    def _require_open_mission(self) -> Mission:
        """Return the active mission or reject closed recording."""

        mission = self._require_started_mission()

        if self._completed_mission is not None:
            raise InvalidStateTransitionError(
                "the mission recording is already completed."
            )

        return mission
