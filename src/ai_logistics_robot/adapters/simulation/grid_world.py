"""Deterministic headless GridWorld simulation adapter."""

from math import isfinite

from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.world import GridMap


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

    def advance_time(self, seconds: float) -> None:
        """Advance simulated time by a finite non-negative duration."""

        duration = _validate_duration(seconds)
        elapsed_time = self._elapsed_time_seconds + duration

        if not isfinite(elapsed_time):
            raise DomainValidationError(
                "elapsed simulated time must remain finite."
            )

        self._elapsed_time_seconds = elapsed_time