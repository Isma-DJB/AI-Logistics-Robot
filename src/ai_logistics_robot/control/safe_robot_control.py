"""Platform-independent command execution with local safety."""

from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    FailureReason,
    SafetySeverity,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.ports.clock_port import ClockPort
from ai_logistics_robot.ports.simulation_port import SimulationPort


class SafeRobotControl:
    """Execute commands while owning confirmed pose and safety state."""

    __slots__ = (
        "_clock",
        "_confirmed_pose",
        "_robot_id",
        "_safety_status",
        "_simulation",
    )

    def __init__(
        self,
        *,
        robot_id: str,
        initial_pose: RobotPose,
        simulation: SimulationPort,
        clock: ClockPort,
    ) -> None:
        """Validate dependencies and initialize an unlatched control."""

        if not isinstance(robot_id, str) or not robot_id.strip():
            raise DomainValidationError(
                "robot_id must be a non-empty string."
            )

        if not isinstance(initial_pose, RobotPose):
            raise DomainValidationError(
                "initial_pose must be a RobotPose instance."
            )

        if not isinstance(simulation, SimulationPort):
            raise DomainValidationError(
                "simulation must satisfy SimulationPort."
            )

        if not isinstance(clock, ClockPort):
            raise DomainValidationError(
                "clock must satisfy ClockPort."
            )

        self._robot_id = robot_id
        self._confirmed_pose = initial_pose
        self._simulation = simulation
        self._clock = clock
        self._safety_status = SafetyStatus(
            robot_id=robot_id,
            updated_at=clock.now(),
            latched=False,
            severity=SafetySeverity.INFO,
        )

    def execute_step(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Execute one command unless the local safety latch is set."""

        self._validate_command(command)

        if self._safety_status.latched:
            return CommandResult(
                command=command,
                status=CommandStatus.ABORTED,
                pose_before=self._confirmed_pose,
                pose_after=self._confirmed_pose,
                failure_reason=FailureReason.SAFETY_LATCHED,
            )

        return self._apply_platform_command(command)

    def stop(self) -> None:
        """Send a normal STOP independently of the latch state."""

        command = MotionCommand(
            robot_id=self._robot_id,
            command_type=CommandType.STOP,
        )
        self._apply_platform_command(command)

    def emergency_stop(
        self,
        reason: FailureReason,
    ) -> SafetyStatus:
        """Latch local safety and send the priority STOP command."""

        if not isinstance(reason, FailureReason):
            raise DomainValidationError(
                "reason must be a FailureReason instance."
            )

        self._safety_status = SafetyStatus(
            robot_id=self._robot_id,
            updated_at=self._clock.now(),
            latched=True,
            severity=SafetySeverity.CRITICAL,
            reason=reason,
        )

        self.stop()
        return self._safety_status

    def get_safety_status(self) -> SafetyStatus:
        """Return the current status without causing control effects."""

        return self._safety_status

    def reset_safety_latch(self) -> SafetyStatus:
        """Perform explicit manual rearm without resetting the platform."""

        self._safety_status = SafetyStatus(
            robot_id=self._robot_id,
            updated_at=self._clock.now(),
            latched=False,
            severity=SafetySeverity.INFO,
        )
        return self._safety_status

    def _validate_command(
        self,
        command: object,
    ) -> None:
        """Validate command type and robot identity atomically."""

        if not isinstance(command, MotionCommand):
            raise DomainValidationError(
                "command must be a MotionCommand instance."
            )

        if command.robot_id != self._robot_id:
            raise DomainValidationError(
                "command robot_id must match Control."
            )

    def _apply_platform_command(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Execute and accept one internally consistent platform result."""

        result = self._simulation.apply_command(command)

        if not isinstance(result, CommandResult):
            raise DomainValidationError(
                "simulation must return a CommandResult."
            )

        if result.command is not command:
            raise InvariantViolationError(
                "command result must retain the supplied command."
            )

        if result.pose_before != self._confirmed_pose:
            raise InvariantViolationError(
                "command result must start at the latest "
                "confirmed pose."
            )

        self._confirmed_pose = result.pose_after
        return result
