"""Run one strictly passive fused physical-perception diagnostic."""

import sys
from argparse import ArgumentParser, Namespace
from collections.abc import Sequence
from pathlib import Path

from ai_logistics_robot.adapters.hardware.arduino_light_calibration import (
    DEFAULT_MAXIMUM_AMBIENT_LIGHT_RANGE,
    MINIMUM_AMBIENT_SAMPLE_COUNT,
    ArduinoAmbientLightCalibration,
    calibrate_ambient_light,
)
from ai_logistics_robot.adapters.hardware.arduino_sensor_interpreter import (
    ArduinoSensorInterpretation,
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
from ai_logistics_robot.adapters.hardware.pyserial_connection import (
    DEFAULT_BAUD_RATE,
    open_arduino_serial_transport,
)
from ai_logistics_robot.adapters.hardware.sensor_fused_grid_perception import (
    SensorFusedGridPerception,
)
from ai_logistics_robot.adapters.hardware.system_clock import (
    SystemClock,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.perception import PerceptionSnapshot

DEFAULT_CALIBRATION = Path(
    "configs/hardware/grid_calibration_7x7.yaml"
)
DEFAULT_AMBIENT_CALIBRATION_SAMPLES = 10


class FusedPhysicalPerceptionDiagnosticError(RuntimeError):
    """Report one unsafe or incomplete fused diagnostic."""


def build_parser() -> ArgumentParser:
    """Build the strictly passive physical diagnostic command."""

    parser = ArgumentParser(
        description=(
            "Fuse live ESP32-CAM robot localization with "
            "read-only Arduino sensor telemetry."
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
        "--ambient-light-raw",
        type=int,
        help=(
            "Optional confirmed ambient baseline. When "
            "omitted, lamp-off calibration is automatic."
        ),
    )
    parser.add_argument(
        "--ambient-calibration-samples",
        default=DEFAULT_AMBIENT_CALIBRATION_SAMPLES,
        type=int,
        help=(
            "Lamp-off samples used for automatic ambient "
            "calibration. "
            f"Default: {DEFAULT_AMBIENT_CALIBRATION_SAMPLES}."
        ),
    )
    parser.add_argument(
        "--maximum-ambient-light-range",
        default=DEFAULT_MAXIMUM_AMBIENT_LIGHT_RANGE,
        type=int,
        help=(
            "Maximum accepted range across lamp-off "
            "calibration samples. "
            f"Default: {DEFAULT_MAXIMUM_AMBIENT_LIGHT_RANGE}."
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
        default=0,
        type=int,
        help="Expected DICT_4X4_50 marker ID. Default: 0.",
    )
    parser.add_argument(
        "--samples",
        default=5,
        type=int,
        help="Positive number of fused observations. Default: 5.",
    )
    parser.add_argument(
        "--frontal-obstacle-distance-mm",
        default=200,
        type=int,
        help=(
            "Inclusive frontal-obstacle threshold in "
            "millimeters. Default: 200."
        ),
    )
    parser.add_argument(
        "--target-activation-delta",
        default=50,
        type=int,
        help=(
            "Light increase required for target activation. "
            "Default: 50."
        ),
    )
    parser.add_argument(
        "--target-release-delta",
        default=30,
        type=int,
        help=(
            "Light delta allowing target release. "
            "Default: 30."
        ),
    )
    parser.add_argument(
        "--required-stable-readings",
        default=5,
        type=int,
        help=(
            "Consecutive readings required for light-state "
            "changes. Default: 5."
        ),
    )
    parser.add_argument(
        "--baud-rate",
        default=DEFAULT_BAUD_RATE,
        type=int,
        help=(
            "Positive Arduino serial baud rate. "
            f"Default: {DEFAULT_BAUD_RATE}."
        ),
    )
    parser.add_argument(
        "--serial-timeout",
        default=2.0,
        type=float,
        help="Positive serial timeout in seconds. Default: 2.",
    )
    parser.add_argument(
        "--camera-timeout",
        default=5.0,
        type=float,
        help="Positive camera timeout in seconds. Default: 5.",
    )
    return parser


def _require_positive_samples(samples: object) -> int:
    """Return one validated positive sample count."""

    if (
        isinstance(samples, bool)
        or not isinstance(samples, int)
        or samples <= 0
    ):
        raise DomainValidationError(
            "samples must be a positive integer."
        )

    return samples


def _require_ambient_calibration_samples(
    samples: object,
) -> int:
    """Return one safe automatic-calibration sample count."""

    if (
        isinstance(samples, bool)
        or not isinstance(samples, int)
        or samples < MINIMUM_AMBIENT_SAMPLE_COUNT
    ):
        raise DomainValidationError(
            "ambient_calibration_samples must be an "
            "integer of at least "
            f"{MINIMUM_AMBIENT_SAMPLE_COUNT}."
        )

    return samples


def _require_latched_status(
    status: ArduinoControllerStatus,
    *,
    stage: str,
) -> None:
    """Require the controller to remain safely stopped."""

    if (
        status.device_state
        is not ArduinoDeviceState.EMERGENCY_STOPPED
        or status.safety_state
        is not ArduinoSafetyState.LATCHED
    ):
        raise FusedPhysicalPerceptionDiagnosticError(
            f"Arduino must be ESTOPPED/LATCHED {stage} "
            "the diagnostic."
        )


def _build_fused_perception(
    arguments: Namespace,
    *,
    client: ArduinoSerialClient,
    ambient_light_raw: int,
) -> tuple[
    SensorFusedGridPerception,
    ArduinoSensorInterpreter,
]:
    """Assemble live visual and read-only sensor perception."""

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

    fused_perception = SensorFusedGridPerception(
        perception=visual_perception,
        sensor_reader=client.request_sensors,
        interpreter=interpreter,
    )
    return fused_perception, interpreter


def main(
    argv: Sequence[str] | None = None,
) -> int:
    """Run one live diagnostic without changing safety state."""

    arguments = build_parser().parse_args(argv)

    try:
        sample_count = _require_positive_samples(
            arguments.samples
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

            fused_perception, interpreter = _build_fused_perception(
                arguments,
                client=client,
                ambient_light_raw=ambient_light_raw,
            )
            results: list[
                tuple[
                    PerceptionSnapshot,
                    ArduinoSensorInterpretation,
                    int,
                ]
            ] = []

            for _ in range(sample_count):
                snapshot = fused_perception.observe()
                interpretation = (
                    fused_perception.last_interpretation
                )

                if interpretation is None:
                    raise (
                        FusedPhysicalPerceptionDiagnosticError(
                            "Fused perception did not retain "
                            "its sensor interpretation."
                        )
                    )

                results.append(
                    (
                        snapshot,
                        interpretation,
                        interpreter.ambient_light_raw,                    )
                )

            final_status = client.request_status()
            _require_latched_status(
                final_status,
                stage="after",
            )
    except (
        ArduinoSerialProtocolError,
        ArduinoSerialSessionError,
        ArduinoSerialTransportError,
        ArucoRobotDetectionError,
        CalibratedGridCameraError,
        DomainValidationError,
        Esp32CameraError,
        FusedPhysicalPerceptionDiagnosticError,
        GridCalibrationFileError,
    ) as error:
        print(
            "Fused physical-perception diagnostic "
            f"failed: {error}",
            file=sys.stderr,
        )
        return 2

    print(
        "Fused physical-perception diagnostic succeeded."
    )
    print(f"Camera: {arguments.url}")
    print(f"Arduino port: {arguments.port}")
    print(f"Ping sequence: {ping_sequence}")
    print(
        "Initial controller state: "
        f"{initial_status.device_state.value}/"
        f"{initial_status.safety_state.value}"
    )

    if ambient_calibration is None:
        print(
            "Ambient light baseline: "
            f"{ambient_light_raw} (provided)"
        )
    else:
        print(
            "Ambient light calibration: "
            f"baseline={ambient_calibration.ambient_light_raw}, "
            f"samples={ambient_calibration.sample_count}, "
            f"range={ambient_calibration.light_range}"
        )
        print(
            "Ambient light observations: "
            f"{ambient_calibration.minimum_light_raw} to "
            f"{ambient_calibration.maximum_light_raw}"
        )

    for sample_number, (
        snapshot,
        interpretation,
        tracked_ambient_light_raw,
    ) in enumerate(results, start=1):
        pose = snapshot.robot_pose
        distance = (
            "NONE"
            if interpretation.distance_mm is None
            else str(interpretation.distance_mm)
        )

        print(f"Sample {sample_number}:")
        print(
            "  Pose: "
            f"x={pose.position.x}, "
            f"y={pose.position.y}, "
            f"heading={pose.heading.value}"
        )
        print(
            "  Infrared obstacles: "
            f"left={interpretation.left_obstacle_detected}, "
            f"right={interpretation.right_obstacle_detected}"
        )
        print(
            "  Boundary: "
            f"{interpretation.boundary_detected}"
        )
        print(
            "  Ultrasonic: "
            f"distance_mm={distance}, "
            "frontal_obstacle="
            f"{interpretation.frontal_obstacle_detected}"
        )
        print(
            "  Light: "
            f"raw={interpretation.light_raw}, "
            f"delta={interpretation.light_delta}, "
            f"baseline_after={tracked_ambient_light_raw}, "
            f"target_active={snapshot.target_active}"
        )
        print(
            "  Critical hazard: "
            f"{snapshot.hazard_detected}"
        )
        print(
            "  Captured at: "
            f"{snapshot.captured_at.isoformat()}"
        )

    print(
        "Final controller state: "
        f"{final_status.device_state.value}/"
        f"{final_status.safety_state.value}"
    )
    print("No rearm or motion command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())