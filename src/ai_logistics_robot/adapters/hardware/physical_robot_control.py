"""Execute supervised Arduino motion with camera pose confirmation."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoCommandOutcome,
    ArduinoCommandRejectedError,
    ArduinoControllerStatus,
    ArduinoSerialSessionError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoCompletionCode,
    ArduinoDeviceState,
    ArduinoErrorCode,
    ArduinoSafetyState,
    ArduinoSerialProtocolError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialTimeoutError,
    ArduinoSerialTransportError,
)
from ai_logistics_robot.control.physical_motion_confirmation import (
    pose_confirms_command,
)
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
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import RobotPose
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.ports.clock_port import ClockPort
from ai_logistics_robot.ports.perception_port import PerceptionPort


@runtime_checkable
class SupervisedArduinoControl(Protocol):
    """Expose only Arduino operations required by physical Control."""

    def execute_command(
        self,
        command_type: CommandType,
    ) -> ArduinoCommandOutcome:
        """Execute one command through matching ACK and DONE."""

        ...

    def emergency_stop(self) -> ArduinoControllerStatus:
        """Request and confirm the hardware emergency latch."""

        ...

    def rearm(self) -> ArduinoControllerStatus:
        """Request and confirm safe idle hardware state."""

        ...


class PhysicalRobotControl:
    """Confirm bounded hardware commands through physical perception."""

    __slots__ = (
        "_arduino",
        "_clock",
        "_confirmed_pose",
        "_perception",
        "_robot_id",
        "_safety_status",
    )

    def __init__(
        self,
        *,
        robot_id: str,
        initial_pose: RobotPose,
        arduino: SupervisedArduinoControl,
        perception: PerceptionPort,
        clock: ClockPort,
    ) -> None:
        """Validate dependencies and begin conservatively latched."""

        if not isinstance(robot_id, str) or not robot_id.strip():
            raise DomainValidationError(
                "robot_id must be a non-empty string."
            )

        if not isinstance(initial_pose, RobotPose):
            raise DomainValidationError(
                "initial_pose must be a RobotPose instance."
            )

        if not isinstance(arduino, SupervisedArduinoControl):
            raise DomainValidationError(
                "arduino must satisfy SupervisedArduinoControl."
            )

        if not isinstance(perception, PerceptionPort):
            raise DomainValidationError(
                "perception must satisfy PerceptionPort."
            )

        if not isinstance(clock, ClockPort):
            raise DomainValidationError(
                "clock must satisfy ClockPort."
            )

        self._robot_id = robot_id
        self._confirmed_pose = initial_pose
        self._arduino = arduino
        self._perception = perception
        self._clock = clock
        self._safety_status = SafetyStatus(
            robot_id=robot_id,
            updated_at=clock.now(),
            latched=True,
            severity=SafetySeverity.CRITICAL,
            reason=FailureReason.SAFETY_LATCHED,
        )

    def execute_step(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Execute one command and require physical confirmation."""

        self._validate_command(command)

        if self._safety_status.latched:
            return self._result(
                command=command,
                status=CommandStatus.ABORTED,
                pose_before=self._confirmed_pose,
                pose_after=self._confirmed_pose,
                failure_reason=FailureReason.SAFETY_LATCHED,
            )

        if command.command_type is CommandType.STOP:
            return self._execute_stop(command)

        snapshot_before = self._observe()
        pose_before = snapshot_before.robot_pose
        self._confirmed_pose = pose_before

        if snapshot_before.hazard_detected:
            self.emergency_stop(
                FailureReason.EMERGENCY_STOP
            )
            return self._result(
                command=command,
                status=CommandStatus.ABORTED,
                pose_before=pose_before,
                pose_after=pose_before,
                failure_reason=FailureReason.EMERGENCY_STOP,
            )

        outcome = self._request_command(
            command=command,
            pose_before=pose_before,
        )

        if isinstance(outcome, CommandResult):
            return outcome

        if not isinstance(outcome, ArduinoCommandOutcome):
            raise DomainValidationError(
                "arduino must return an ArduinoCommandOutcome."
            )

        if outcome.command_type is not command.command_type:
            raise DomainValidationError(
                "Arduino outcome command must match the request."
            )

        if outcome.completion_code is not ArduinoCompletionCode.OK:
            return self._result_from_completion(
                command=command,
                pose_before=pose_before,
                completion_code=outcome.completion_code,
            )

        snapshot_after = self._observe()
        pose_after = snapshot_after.robot_pose
        self._confirmed_pose = pose_after

        if (
            not snapshot_after.hazard_detected
            and pose_confirms_command(
                pose_before=pose_before,
                command_type=command.command_type,
                observed_pose=pose_after,
            )
        ):
            return self._result(
                command=command,
                status=CommandStatus.SUCCESS,
                pose_before=pose_before,
                pose_after=pose_after,
            )

        self.emergency_stop(FailureReason.EMERGENCY_STOP)

        return self._result(
            command=command,
            status=CommandStatus.ABORTED,
            pose_before=pose_before,
            pose_after=pose_after,
            failure_reason=FailureReason.EMERGENCY_STOP,
        )

    def stop(self) -> None:
        """Request normal hardware STOP in every safety state."""

        command = MotionCommand(
            robot_id=self._robot_id,
            command_type=CommandType.STOP,
        )
        self._execute_stop(command)

    def emergency_stop(
        self,
        reason: FailureReason,
    ) -> SafetyStatus:
        """Latch locally before requesting the hardware emergency stop."""

        if not isinstance(reason, FailureReason):
            raise DomainValidationError(
                "reason must be a FailureReason instance."
            )

        self._set_latched(reason)
        status = self._arduino.emergency_stop()

        if not isinstance(status, ArduinoControllerStatus):
            raise DomainValidationError(
                "arduino emergency_stop must return "
                "an ArduinoControllerStatus."
            )

        if (
            status.device_state
            is not ArduinoDeviceState.EMERGENCY_STOPPED
            or status.safety_state
            is not ArduinoSafetyState.LATCHED
        ):
            raise DomainValidationError(
                "arduino emergency_stop must confirm "
                "ESTOPPED/LATCHED."
            )

        return self._safety_status

    def get_safety_status(self) -> SafetyStatus:
        """Return the latest locally confirmed safety state."""

        return self._safety_status

    def reset_safety_latch(self) -> SafetyStatus:
        """Require confirmed Arduino rearm before becoming safe."""

        status = self._arduino.rearm()

        if not isinstance(status, ArduinoControllerStatus):
            raise DomainValidationError(
                "arduino rearm must return "
                "an ArduinoControllerStatus."
            )

        if (
            status.device_state is not ArduinoDeviceState.IDLE
            or status.safety_state
            is not ArduinoSafetyState.SAFE
        ):
            raise DomainValidationError(
                "arduino rearm must confirm IDLE/SAFE."
            )

        self._safety_status = SafetyStatus(
            robot_id=self._robot_id,
            updated_at=self._clock.now(),
            latched=False,
            severity=SafetySeverity.INFO,
        )
        return self._safety_status

    def _execute_stop(
        self,
        command: MotionCommand,
    ) -> CommandResult:
        """Execute STOP without requiring a new camera observation."""

        pose_before = self._confirmed_pose
        outcome = self._request_command(
            command=command,
            pose_before=pose_before,
        )

        if isinstance(outcome, CommandResult):
            return outcome

        if not isinstance(outcome, ArduinoCommandOutcome):
            raise DomainValidationError(
                "arduino must return an ArduinoCommandOutcome."
            )

        if outcome.command_type is not CommandType.STOP:
            raise DomainValidationError(
                "Arduino outcome command must match STOP."
            )

        if outcome.completion_code is ArduinoCompletionCode.OK:
            return self._result(
                command=command,
                status=CommandStatus.SUCCESS,
                pose_before=pose_before,
                pose_after=pose_before,
            )

        return self._result_from_completion(
            command=command,
            pose_before=pose_before,
            completion_code=outcome.completion_code,
        )

    def _request_command(
        self,
        *,
        command: MotionCommand,
        pose_before: RobotPose,
    ) -> ArduinoCommandOutcome | CommandResult:
        """Execute one request or normalize a serial failure."""

        try:
            return self._arduino.execute_command(
                command.command_type
            )
        except ArduinoCommandRejectedError as error:
            return self._result_from_rejection(
                command=command,
                pose_before=pose_before,
                error_code=error.error_code,
            )
        except ArduinoSerialTimeoutError:
            self._set_latched(FailureReason.TIMEOUT)
            return self._result(
                command=command,
                status=CommandStatus.TIMEOUT,
                pose_before=pose_before,
                pose_after=pose_before,
                failure_reason=FailureReason.TIMEOUT,
            )
        except (
            ArduinoSerialTransportError,
            ArduinoSerialSessionError,
            ArduinoSerialProtocolError,
        ):
            self._set_latched(
                FailureReason.COMMUNICATION_LOSS
            )
            return self._result(
                command=command,
                status=CommandStatus.ABORTED,
                pose_before=pose_before,
                pose_after=pose_before,
                failure_reason=(
                    FailureReason.COMMUNICATION_LOSS
                ),
            )

    def _result_from_rejection(
        self,
        *,
        command: MotionCommand,
        pose_before: RobotPose,
        error_code: ArduinoErrorCode,
    ) -> CommandResult:
        """Normalize one explicit Arduino command rejection."""

        if error_code is ArduinoErrorCode.SAFETY_LATCHED:
            status = CommandStatus.ABORTED
            reason = FailureReason.SAFETY_LATCHED
            self._set_latched(reason)
        else:
            status = CommandStatus.FAILED
            reason = FailureReason.INTERNAL_ERROR

        return self._result(
            command=command,
            status=status,
            pose_before=pose_before,
            pose_after=pose_before,
            failure_reason=reason,
        )

    def _result_from_completion(
        self,
        *,
        command: MotionCommand,
        pose_before: RobotPose,
        completion_code: ArduinoCompletionCode,
    ) -> CommandResult:
        """Normalize one non-success Arduino completion code."""

        if completion_code is ArduinoCompletionCode.BLOCKED:
            status = CommandStatus.FAILED
            reason = FailureReason.BLOCKED
        elif completion_code is ArduinoCompletionCode.TIMEOUT:
            status = CommandStatus.TIMEOUT
            reason = FailureReason.TIMEOUT
        elif (
            completion_code
            is ArduinoCompletionCode.COMMUNICATION_LOSS
        ):
            status = CommandStatus.ABORTED
            reason = FailureReason.COMMUNICATION_LOSS
            self._set_latched(reason)
        elif (
            completion_code
            is ArduinoCompletionCode.EMERGENCY_STOP
        ):
            status = CommandStatus.ABORTED
            reason = FailureReason.EMERGENCY_STOP
            self._set_latched(reason)
        else:
            raise DomainValidationError(
                "completion_code must be a non-success code."
            )

        return self._result(
            command=command,
            status=status,
            pose_before=pose_before,
            pose_after=pose_before,
            failure_reason=reason,
        )

    def _observe(self) -> PerceptionSnapshot:
        """Return one validated observation for this robot."""

        snapshot = self._perception.observe()

        if not isinstance(snapshot, PerceptionSnapshot):
            raise DomainValidationError(
                "perception must return a PerceptionSnapshot."
            )

        if snapshot.robot_id != self._robot_id:
            raise DomainValidationError(
                "perception robot_id must match Control."
            )

        return snapshot

    def _set_latched(
        self,
        reason: FailureReason,
    ) -> None:
        """Replace local safety state with one critical latch."""

        self._safety_status = SafetyStatus(
            robot_id=self._robot_id,
            updated_at=self._clock.now(),
            latched=True,
            severity=SafetySeverity.CRITICAL,
            reason=reason,
        )

    def _validate_command(
        self,
        command: object,
    ) -> None:
        """Validate command type and robot identity."""

        if not isinstance(command, MotionCommand):
            raise DomainValidationError(
                "command must be a MotionCommand instance."
            )

        if command.robot_id != self._robot_id:
            raise DomainValidationError(
                "command robot_id must match Control."
            )

    @staticmethod
    def _result(
        *,
        command: MotionCommand,
        status: CommandStatus,
        pose_before: RobotPose,
        pose_after: RobotPose,
        failure_reason: FailureReason | None = None,
    ) -> CommandResult:
        """Build one immutable normalized command result."""

        return CommandResult(
            command=command,
            status=status,
            pose_before=pose_before,
            pose_after=pose_after,
            failure_reason=failure_reason,
        )