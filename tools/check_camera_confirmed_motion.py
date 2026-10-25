"""Run one explicitly authorized camera-confirmed motion diagnostic."""

import sys
from argparse import ArgumentParser, Namespace
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from ai_logistics_robot.adapters.hardware.arduino_light_calibration import (
    MINIMUM_AMBIENT_SAMPLE_COUNT,
    ArduinoAmbientLightCalibration,
    calibrate_ambient_light,
)
from ai_logistics_robot.adapters.hardware.arduino_sensor_interpreter import (
    ArduinoSensorInterpreter,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoControllerStatus,
    ArduinoSerialClient,
    ArduinoSerialSessionError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoDeviceState,
    ArduinoSafetyState,
    ArduinoSerialProtocolError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialTransportError,
)
from ai_logistics_robot.adapters.hardware.aruco_robot_detector import (
    ArucoRobotDetectionError,
    ArucoRobotDetector,
)
from ai_logistics_robot.adapters.hardware.calibrated_grid_camera import (
    CalibratedGridCamera,
    CalibratedGridCameraError,
)
from ai_logistics_robot.adapters.hardware.esp32_camera import (
    Esp32CameraClient,
    Esp32CameraError,
)
from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibrationFileError,
    load_grid_calibration,
)
from ai_logistics_robot.adapters.hardware.physical_grid_perception import (
    PhysicalGridPerception,
)
from ai_logistics_robot.adapters.hardware.physical_robot_control import (
    PhysicalRobotControl,
)
from ai_logistics_robot.adapters.hardware.pyserial_connection import (
    open_arduino_serial_transport,
)
from ai_logistics_robot.adapters.hardware.sensor_fused_grid_perception import (
    SensorFusedGridPerception,
)
from ai_logistics_robot.adapters.hardware.system_clock import (
    SystemClock,
)
from ai_logistics_robot.domain.commands import (
    CommandResult,
    MotionCommand,
)
from ai_logistics_robot.domain.enums import (
    CommandStatus,
    CommandType,
    FailureReason,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.perception import PerceptionSnapshot
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.ports.control_port import ControlPort

DEFAULT_CALIBRATION = Path(
    "configs/hardware/grid_calibration_7x7.yaml"
)
DEFAULT_AMBIENT_CALIBRATION_SAMPLES = 10
DEFAULT_MAXIMUM_AMBIENT_LIGHT_RANGE = 10
DEFAULT_FRONTAL_OBSTACLE_DISTANCE_MM = 200
DEFAULT_TARGET_ACTIVATION_DELTA = 50
DEFAULT_TARGET_RELEASE_DELTA = 30
DEFAULT_REQUIRED_STABLE_READINGS = 5
DEFAULT_BAUD_RATE = 115200
DEFAULT_SERIAL_TIMEOUT_SECONDS = 5.0
DEFAULT_CAMERA_TIMEOUT_SECONDS = 15.0

_ALLOWED_COMMANDS = (
    CommandType.MOVE_FORWARD,
    CommandType.TURN_LEFT,
    CommandType.TURN_RIGHT,
)


class CameraConfirmedMotionDiagnosticError(RuntimeError):
    """Report one rejected or unsuccessful guarded diagnostic."""


@dataclass(frozen=True, slots=True)
class CameraConfirmedMotionCycleResult:
    """Retain one guarded control cycle and final safety state."""

    command_result: CommandResult
    final_safety_status: SafetyStatus


@dataclass(frozen=True, slots=True)
class CameraConfirmedMotionDiagnosticResult:
    """Retain one complete live camera-confirmed diagnostic."""

    ping_sequence: int
    initial_status: ArduinoControllerStatus
    ambient_calibration: (
        ArduinoAmbientLightCalibration | None
    )
    ambient_light_raw: int
    initial_snapshot: PerceptionSnapshot
    cycle: CameraConfirmedMotionCycleResult
    final_status: ArduinoControllerStatus


def build_parser() -> ArgumentParser:
    """Build the guarded physical-motion command-line parser."""

    parser = ArgumentParser(
        description=(
            "Execute one bounded Arduino command and require "
            "exact camera-confirmed grid pose."
        )
    )
    parser.add_argument(
        "--url",
        required=True,
        help="ESP32-CAM HTTP origin.",
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Arduino serial port, for example COM3.",
    )
    parser.add_argument(
        "--command",
        required=True,
        choices=tuple(
            command.value
            for command in _ALLOWED_COMMANDS
        ),
        help=(
            "Single bounded command: MOVE_FORWARD, "
            "TURN_LEFT, or TURN_RIGHT."
        ),
    )
    parser.add_argument(
        "--confirm-ground-test",
        action="store_true",
        help=(
            "Explicitly confirm that the robot is on the "
            "calibrated grid and the operator can intervene."
        ),
    )
    parser.add_argument(
        "--ambient-light-raw",
        type=int,
        default=None,
        help=(
            "Optional confirmed ambient baseline. When omitted, "
            "lamp-off calibration is automatic."
        ),
    )
    parser.add_argument(
        "--ambient-calibration-samples",
        type=int,
        default=DEFAULT_AMBIENT_CALIBRATION_SAMPLES,
        help=(
            "Lamp-off samples used for automatic calibration. "
            "Default: 10."
        ),
    )
    parser.add_argument(
        "--maximum-ambient-light-range",
        type=int,
        default=DEFAULT_MAXIMUM_AMBIENT_LIGHT_RANGE,
        help=(
            "Maximum accepted lamp-off sample range. "
            "Default: 10."
        ),
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help=(
            "Validated YAML grid calibration. "
            f"Default: {DEFAULT_CALIBRATION}."
        ),
    )
    parser.add_argument(
        "--robot-id",
        default="robot-1",
        help="Physical robot identifier. Default: robot-1.",
    )
    parser.add_argument(
        "--marker-id",
        type=int,
        default=0,
        help="Expected DICT_4X4_50 marker ID. Default: 0.",
    )
    parser.add_argument(
        "--frontal-obstacle-distance-mm",
        type=int,
        default=DEFAULT_FRONTAL_OBSTACLE_DISTANCE_MM,
        help=(
            "Inclusive frontal-obstacle threshold. "
            "Default: 200."
        ),
    )
    parser.add_argument(
        "--target-activation-delta",
        type=int,
        default=DEFAULT_TARGET_ACTIVATION_DELTA,
        help="Target activation delta. Default: 50.",
    )
    parser.add_argument(
        "--target-release-delta",
        type=int,
        default=DEFAULT_TARGET_RELEASE_DELTA,
        help="Target release delta. Default: 30.",
    )
    parser.add_argument(
        "--required-stable-readings",
        type=int,
        default=DEFAULT_REQUIRED_STABLE_READINGS,
        help="Stable light readings required. Default: 5.",
    )
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help="Arduino serial baud rate. Default: 115200.",
    )
    parser.add_argument(
        "--serial-timeout",
        type=float,
        default=DEFAULT_SERIAL_TIMEOUT_SECONDS,
        help="Serial timeout in seconds. Default: 5.",
    )
    parser.add_argument(
        "--camera-timeout",
        type=float,
        default=DEFAULT_CAMERA_TIMEOUT_SECONDS,
        help="Camera timeout in seconds. Default: 15.",
    )
    return parser


def _require_explicit_authorization(
    confirmed: object,
) -> None:
    """Require the exact opt-in flag before hardware access."""

    if confirmed is not True:
        raise CameraConfirmedMotionDiagnosticError(
            "ground-test authorization is required."
        )


def _require_motion_command(
    value: object,
) -> CommandType:
    """Return one explicitly supported physical command."""

    if not isinstance(value, str):
        raise DomainValidationError(
            "command must be MOVE_FORWARD, "
            "TURN_LEFT, or TURN_RIGHT."
        )

    try:
        command = CommandType(value)
    except ValueError as error:
        raise DomainValidationError(
            "command must be MOVE_FORWARD, "
            "TURN_LEFT, or TURN_RIGHT."
        ) from error

    if command not in _ALLOWED_COMMANDS:
        raise DomainValidationError(
            "command must be MOVE_FORWARD, "
            "TURN_LEFT, or TURN_RIGHT."
        )

    return command


def _require_ambient_calibration_samples(
    samples: object,
) -> int:
    """Return an accepted automatic-calibration sample count."""

    if (
        isinstance(samples, bool)
        or not isinstance(samples, int)
        or samples < MINIMUM_AMBIENT_SAMPLE_COUNT
    ):
        raise DomainValidationError(
            "ambient calibration samples must be an "
            f"integer of at least {MINIMUM_AMBIENT_SAMPLE_COUNT}."
        )

    return samples


def _require_latched_status(
    status: ArduinoControllerStatus,
    *,
    stage: str,
) -> None:
    """Require one confirmed ESTOPPED/LATCHED status."""

    if not isinstance(status, ArduinoControllerStatus):
        raise DomainValidationError(
            "controller status must be an "
            "ArduinoControllerStatus."
        )

    if (
        status.device_state
        is not ArduinoDeviceState.EMERGENCY_STOPPED
        or status.safety_state
        is not ArduinoSafetyState.LATCHED
    ):
        raise CameraConfirmedMotionDiagnosticError(
            f"controller must be ESTOPPED/LATCHED {stage}."
        )


def _build_fused_perception(
    arguments: Namespace,
    *,
    client: ArduinoSerialClient,
    ambient_light_raw: int,
) -> SensorFusedGridPerception:
    """Assemble live visual and sensor-fused perception."""

    calibration = load_grid_calibration(
        arguments.calibration
    )
    camera = Esp32CameraClient(
        base_url=arguments.url,
        timeout_seconds=arguments.camera_timeout,
    )
    calibrated_camera = CalibratedGridCamera(
        camera=camera,
        calibration=calibration,
    )
    detector = ArucoRobotDetector(
        calibration=calibration,
        marker_id=arguments.marker_id,
    )
    visual_perception = PhysicalGridPerception(
        robot_id=arguments.robot_id,
        camera=calibrated_camera,
        detector=detector,
        clock=SystemClock(),
    )
    interpreter = ArduinoSensorInterpreter(
        ambient_light_raw=ambient_light_raw,
        frontal_obstacle_distance_mm=(
            arguments.frontal_obstacle_distance_mm
        ),
        target_activation_delta=(
            arguments.target_activation_delta
        ),
        target_release_delta=(
            arguments.target_release_delta
        ),
        required_stable_readings=(
            arguments.required_stable_readings
        ),
    )

    return SensorFusedGridPerception(
        perception=visual_perception,
        sensor_reader=client.request_sensors,
        interpreter=interpreter,
    )


def _execute_control_cycle(
    *,
    control: ControlPort,
    command: MotionCommand,
) -> CameraConfirmedMotionCycleResult:
    """Execute one command and always request final ESTOP."""

    if not isinstance(control, ControlPort):
        raise DomainValidationError(
            "control must satisfy ControlPort."
        )

    if not isinstance(command, MotionCommand):
        raise DomainValidationError(
            "command must be a MotionCommand instance."
        )

    if command.command_type not in _ALLOWED_COMMANDS:
        raise DomainValidationError(
            "command must be MOVE_FORWARD, "
            "TURN_LEFT, or TURN_RIGHT."
        )

    command_result: CommandResult | None = None
    final_safety_status: SafetyStatus | None = None

    try:
        rearmed_status = control.reset_safety_latch()

        if not isinstance(rearmed_status, SafetyStatus):
            raise DomainValidationError(
                "control rearm must return a SafetyStatus."
            )

        if rearmed_status.latched:
            raise CameraConfirmedMotionDiagnosticError(
                "control did not confirm a safe rearm."
            )

        command_result = control.execute_step(command)

        if not isinstance(command_result, CommandResult):
            raise DomainValidationError(
                "control must return a CommandResult."
            )
    finally:
        final_safety_status = control.emergency_stop(
            FailureReason.EMERGENCY_STOP
        )

    if not isinstance(final_safety_status, SafetyStatus):
        raise DomainValidationError(
            "control ESTOP must return a SafetyStatus."
        )

    if not final_safety_status.latched:
        raise CameraConfirmedMotionDiagnosticError(
            "final ESTOP did not confirm a safety latch."
        )

    assert command_result is not None

    if command_result.status is not CommandStatus.SUCCESS:
        failure_reason = command_result.failure_reason
        reason_text = (
            "NONE"
            if failure_reason is None
            else failure_reason.value
        )
        raise CameraConfirmedMotionDiagnosticError(
            "physical command was not confirmed: "
            f"status={command_result.status.value}, "
            f"reason={reason_text}"
        )

    return CameraConfirmedMotionCycleResult(
        command_result=command_result,
        final_safety_status=final_safety_status,
    )


def _run_diagnostic(
    arguments: Namespace,
) -> CameraConfirmedMotionDiagnosticResult:
    """Execute one guarded physical command and final ESTOP."""

    _require_explicit_authorization(
        arguments.confirm_ground_test
    )
    command_type = _require_motion_command(
        arguments.command
    )

    calibration_sample_count: int | None = None

    if arguments.ambient_light_raw is None:
        calibration_sample_count = (
            _require_ambient_calibration_samples(
                arguments.ambient_calibration_samples
            )
        )

    with open_arduino_serial_transport(
        port=arguments.port,
        baud_rate=arguments.baud_rate,
        read_timeout_seconds=arguments.serial_timeout,
        write_timeout_seconds=arguments.serial_timeout,
    ) as transport:
        client = ArduinoSerialClient(
            transport=transport,
        )
        ping_sequence = client.ping()
        initial_status = client.request_status()
        _require_latched_status(
            initial_status,
            stage="before",
        )

        ambient_calibration: (
            ArduinoAmbientLightCalibration | None
        ) = None
        ambient_light_raw = arguments.ambient_light_raw

        if ambient_light_raw is None:
            assert calibration_sample_count is not None

            ambient_snapshots = tuple(
                client.request_sensors()
                for _ in range(calibration_sample_count)
            )
            ambient_calibration = calibrate_ambient_light(
                ambient_snapshots,
                maximum_light_range=(
                    arguments.maximum_ambient_light_range
                ),
            )
            ambient_light_raw = (
                ambient_calibration.ambient_light_raw
            )

        fused_perception = _build_fused_perception(
            arguments,
            client=client,
            ambient_light_raw=ambient_light_raw,
        )

        initial_snapshot = fused_perception.observe()
        clock = SystemClock()
        control = PhysicalRobotControl(
            robot_id=arguments.robot_id,
            initial_pose=initial_snapshot.robot_pose,
            arduino=client,
            perception=fused_perception,
            clock=clock,
        )
        command = MotionCommand(
            robot_id=arguments.robot_id,
            command_type=command_type,
        )
        cycle = _execute_control_cycle(
            control=control,
            command=command,
        )

        final_status = client.request_status()
        _require_latched_status(
            final_status,
            stage="after",
        )

    return CameraConfirmedMotionDiagnosticResult(
        ping_sequence=ping_sequence,
        initial_status=initial_status,
        ambient_calibration=ambient_calibration,
        ambient_light_raw=ambient_light_raw,
        initial_snapshot=initial_snapshot,
        cycle=cycle,
        final_status=final_status,
    )


def main(
    argv: Sequence[str] | None = None,
) -> int:
    """Run one guarded camera-confirmed physical command."""

    arguments = build_parser().parse_args(argv)

    try:
        result = _run_diagnostic(arguments)
    except (
        ArduinoSerialProtocolError,
        ArduinoSerialSessionError,
        ArduinoSerialTransportError,
        ArucoRobotDetectionError,
        CalibratedGridCameraError,
        CameraConfirmedMotionDiagnosticError,
        DomainValidationError,
        Esp32CameraError,
        GridCalibrationFileError,
    ) as error:
        print(
            "Camera-confirmed motion diagnostic "
            f"failed: {error}",
            file=sys.stderr,
        )
        return 2

    pose_before = result.initial_snapshot.robot_pose
    command_result = result.cycle.command_result
    pose_after = command_result.pose_after

    print(
        "Camera-confirmed motion diagnostic succeeded."
    )
    print(f"Camera: {arguments.url}")
    print(f"Arduino port: {arguments.port}")
    print(f"Ping sequence: {result.ping_sequence}")
    print(f"Command: {command_result.command.command_type.value}")
    print(
        "Initial controller state: "
        f"{result.initial_status.device_state.value}/"
        f"{result.initial_status.safety_state.value}"
    )

    if result.ambient_calibration is None:
        print(
            "Ambient light baseline: "
            f"{result.ambient_light_raw} (provided)"
        )
    else:
        calibration = result.ambient_calibration
        print(
            "Ambient light calibration: "
            f"baseline={calibration.ambient_light_raw}, "
            f"samples={calibration.sample_count}, "
            f"range={calibration.light_range}"
        )

    print(
        "Pose before: "
        f"x={pose_before.position.x}, "
        f"y={pose_before.position.y}, "
        f"heading={pose_before.heading.value}"
    )
    print(
        "Pose after: "
        f"x={pose_after.position.x}, "
        f"y={pose_after.position.y}, "
        f"heading={pose_after.heading.value}"
    )
    print(
        "Command result: "
        f"{command_result.status.value}"
    )
    print(
        "Final controller state: "
        f"{result.final_status.device_state.value}/"
        f"{result.final_status.safety_state.value}"
    )
    print("Emergency-stop cleanup was confirmed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())