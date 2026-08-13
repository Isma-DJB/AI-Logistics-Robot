"""Deterministic headless GridWorld simulation adapter."""

from math import isfinite

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
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap

_CARDINAL_HEADINGS = (
    Heading.NORTH,
    Heading.EAST,
    Heading.SOUTH,
    Heading.WEST,
)


def _validate_duration(seconds: object) -> float:
    """Return a valid finite non-negative simulated duration."""

    if (
        isinstance(seconds, bool)
        or not isinstance(seconds, (int, float))
    ):
        raise DomainValidationError(
            "seconds must be a finite non-negative number."
        )

    duration = float(seconds)

    if not isfinite(duration) or duration < 0:
        raise DomainValidationError(
            "seconds must be a finite non-negative number."
        )

    return duration


def _rotate_heading(
    heading: Heading,
    *,
    quarter_turns: int,
) -> Heading:
    """Rotate one cardinal heading by signed quarter turns."""

    current_index = _CARDINAL_HEADINGS.index(heading)
    next_index = (
        current_index + quarter_turns
    ) % len(_CARDINAL_HEADINGS)

    return _CARDINAL_HEADINGS[next_index]


def _forward_position(pose: RobotPose) -> Position:
    """Return the cell located directly ahead of one pose."""

    if pose.heading is Heading.NORTH:
        displacement = (0, 1)
    elif pose.heading is Heading.EAST:
        displacement = (1, 0)
    elif pose.heading is Heading.SOUTH:
        displacement = (0, -1)
    elif pose.heading is Heading.WEST:
        displacement = (-1, 0)
    else:
        raise DomainValidationError(
            "pose heading is not supported by GridWorld."
        )

    return Position(
        x=pose.position.x + displacement[0],
        y=pose.position.y + displacement[1],
    )


class GridWorld:
    """Maintain deterministic headless simulation state for one robot."""

    __slots__ = (
        "_world",
        "_robot_id",
        "_initial_pose",
        "_current_pose",
        "_elapsed_time_seconds",
    )

    def __init__(
        self,
        *,
        world: GridMap,
        robot_id: str,
        initial_pose: RobotPose,
    ) -> None:
        """Validate and store the configured initial simulation state."""

        if not isinstance(world, GridMap):
            raise DomainValidationError(
                "world must be a GridMap instance."
            )

        if not isinstance(robot_id, str) or not robot_id.strip():
            raise DomainValidationError(
                "robot_id must be a non-empty string."
            )

        if not isinstance(initial_pose, RobotPose):
            raise DomainValidationError(
                "initial_pose must be a RobotPose instance."
            )

        if not world.is_traversable(initial_pose.position):
            raise DomainValidationError(
                "initial_pose must be traversable in the world."
            )

        self._world = world
        self._robot_id = robot_id
        self._initial_pose = initial_pose
        self._current_pose = initial_pose
        self._elapsed_time_seconds = 0.0

    @property
    def current_pose(self) -> RobotPose:
        """Return the confirmed current robot pose."""

        return self._current_pose

    @property
    def elapsed_time_seconds(self) -> float:
        """Return the confirmed elapsed simulated time."""

        return self._elapsed_time_seconds

    def reset(self) -> None:
        """Restore the configured initial pose and simulated time."""

        self._current_pose = self._initial_pose
        self._elapsed_time_seconds = 0.0

    def read_world(self) -> GridMap:
        """Return the configured immutable grid map."""

        return self._world

    def apply_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Apply one validated command to the confirmed robot pose."""

        if not isinstance(command, MotionCommand):
            raise DomainValidationError(
                "command must be a MotionCommand instance."
            )

        if command.robot_id != self._robot_id:
            raise DomainValidationError(
                "command robot_id must match the simulated robot."
            )

        if command.command_type is CommandType.STOP:
            return self._confirm_success(
                command=command,
                pose_after=self._current_pose,
            )

        if command.command_type is CommandType.TURN_LEFT:
            return self._confirm_success(
                command=command,
                pose_after=RobotPose(
                    position=self._current_pose.position,
                    heading=_rotate_heading(
                        self._current_pose.heading,
                        quarter_turns=-1,
                    ),
                ),
            )

        if command.command_type is CommandType.TURN_RIGHT:
            return self._confirm_success(
                command=command,
                pose_after=RobotPose(
                    position=self._current_pose.position,
                    heading=_rotate_heading(
                        self._current_pose.heading,
                        quarter_turns=1,
                    ),
                ),
            )

        if command.command_type is CommandType.MOVE_FORWARD:
            return self._move_forward(command)

        raise DomainValidationError(
            "command type is not supported by GridWorld."
        )

    def advance_time(self, seconds: float) -> None:
        """Advance simulated time by a finite non-negative duration."""

        duration = _validate_duration(seconds)
        elapsed_time = self._elapsed_time_seconds + duration

        if not isfinite(elapsed_time):
            raise DomainValidationError(
                "elapsed simulated time must remain finite."
            )

        self._elapsed_time_seconds = elapsed_time

    def _move_forward(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Move one cell or return a normalized failed result."""

        candidate = _forward_position(self._current_pose)

        if not self._world.contains(candidate):
            return self._reject_movement(
                command=command,
                failure_reason=FailureReason.OUT_OF_BOUNDS,
            )

        if not self._world.is_traversable(candidate):
            return self._reject_movement(
                command=command,
                failure_reason=FailureReason.BLOCKED,
            )

        return self._confirm_success(
            command=command,
            pose_after=RobotPose(
                position=candidate,
                heading=self._current_pose.heading,
            ),
        )

    def _confirm_success(
        self,
        *,
        command: MotionCommand,
        pose_after: RobotPose,
    ) -> CommandResult:
        """Build a successful result before confirming new state."""

        result = CommandResult(
            command=command,
            status=CommandStatus.SUCCESS,
            pose_before=self._current_pose,
            pose_after=pose_after,
        )

        self._current_pose = result.pose_after
        return result

    def _reject_movement(
        self,
        *,
        command: MotionCommand,
        failure_reason: FailureReason,
    ) -> CommandResult:
        """Return a failed result without changing confirmed state."""

        return CommandResult(
            command=command,
            status=CommandStatus.FAILED,
            pose_before=self._current_pose,
            pose_after=self._current_pose,
            failure_reason=failure_reason,
        )