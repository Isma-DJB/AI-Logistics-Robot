"""Run one explicitly authorized bounded Arduino motion diagnostic."""

import sys
from argparse import ArgumentParser, Namespace
from collections.abc import Sequence
from dataclasses import dataclass

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoCommandOutcome,
    ArduinoControllerStatus,
    ArduinoSensorSnapshot,
    ArduinoSerialClient,
    ArduinoSerialSessionError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoCompletionCode,
    ArduinoDeviceState,
    ArduinoSafetyState,
    ArduinoSerialProtocolError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialTransportError,
)
from ai_logistics_robot.adapters.hardware.pyserial_connection import (
    DEFAULT_BAUD_RATE,
    open_arduino_serial_transport,
)
from ai_logistics_robot.domain.enums import CommandType
from ai_logistics_robot.domain.errors import DomainValidationError

DEFAULT_FRONTAL_OBSTACLE_DISTANCE_MM = 200


class BoundedArduinoMotionDiagnosticError(RuntimeError):
    """Report one unsafe or incomplete bounded-motion diagnostic."""


@dataclass(frozen=True, slots=True)
class BoundedArduinoMotionDiagnosticResult:
    """Retain every confirmed stage of one bounded-motion exchange."""

    ping_sequence: int
    initial_status: ArduinoControllerStatus
    sensors: ArduinoSensorSnapshot
    rearmed_status: ArduinoControllerStatus
    outcome: ArduinoCommandOutcome
    final_status: ArduinoControllerStatus


def build_parser() -> ArgumentParser:
    """Build the explicitly authorized motion diagnostic command."""

    parser = ArgumentParser(
        description=(
            "Execute one bounded MOVE_FORWARD command while the "
            "robot is secured with both wheels lifted."
        )
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Arduino serial port, for example COM3.",
    )
    parser.add_argument(
        "--confirm-wheels-lifted",
        action="store_true",
        help=(
            "Explicitly confirm that both wheels are lifted, "
            "free, and unable to move the chassis."
        ),
    )
    parser.add_argument(
        "--frontal-obstacle-distance-mm",
        default=DEFAULT_FRONTAL_OBSTACLE_DISTANCE_MM,
        type=int,
        help=(
            "Minimum exclusive clear distance required before "
            "movement. Default: 200."
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
        "--timeout",
        default=2.0,
        type=float,
        help=(
            "Positive serial read/write timeout in seconds. "
            "Default: 2."
        ),
    )
    return parser


def _require_explicit_authorization(
    confirmed: object,
) -> None:
    """Require an explicit wheels-lifted confirmation."""

    if confirmed is not True:
        raise BoundedArduinoMotionDiagnosticError(
            "bounded motion requires --confirm-wheels-lifted."
        )


def _require_positive_distance_threshold(
    distance_mm: object,
) -> int:
    """Return one validated positive distance threshold."""

    if (
        isinstance(distance_mm, bool)
        or not isinstance(distance_mm, int)
        or distance_mm <= 0
    ):
        raise DomainValidationError(
            "frontal_obstacle_distance_mm must be "
            "a positive integer."
        )

    return distance_mm


def _require_latched_status(
    status: ArduinoControllerStatus,
    *,
    stage: str,
) -> None:
    """Require confirmed ESTOPPED/LATCHED safety."""

    if not isinstance(status, ArduinoControllerStatus):
        raise DomainValidationError(
            "status must be an ArduinoControllerStatus."
        )

    if (
        status.device_state
        is not ArduinoDeviceState.EMERGENCY_STOPPED
        or status.safety_state
        is not ArduinoSafetyState.LATCHED
    ):
        raise BoundedArduinoMotionDiagnosticError(
            f"Arduino must be ESTOPPED/LATCHED {stage} "
            "bounded motion."
        )


def _require_safe_preflight(
    snapshot: ArduinoSensorSnapshot,
    *,
    frontal_obstacle_distance_mm: int,
) -> None:
    """Require clear local sensors before allowing rearm."""

    if not isinstance(snapshot, ArduinoSensorSnapshot):
        raise DomainValidationError(
            "snapshot must be an ArduinoSensorSnapshot."
        )

    distance_threshold = (
        _require_positive_distance_threshold(
            frontal_obstacle_distance_mm
        )
    )

    if (
        snapshot.left_ir_raw != 1
        or snapshot.right_ir_raw != 1
    ):
        raise BoundedArduinoMotionDiagnosticError(
            "both infrared sensors must report clear raw value 1."
        )

    if snapshot.line_raw != 0:
        raise BoundedArduinoMotionDiagnosticError(
            "the boundary sensor must report safe raw value 0."
        )

    if snapshot.distance_mm is None:
        raise BoundedArduinoMotionDiagnosticError(
            "ultrasonic distance must be available before motion."
        )

    if snapshot.distance_mm <= distance_threshold:
        raise BoundedArduinoMotionDiagnosticError(
            "ultrasonic distance must be greater than "
            f"{distance_threshold} mm before motion."
        )


def _require_successful_completion(
    outcome: ArduinoCommandOutcome,
) -> None:
    """Require one confirmed successful forward completion."""

    if not isinstance(outcome, ArduinoCommandOutcome):
        raise DomainValidationError(
            "outcome must be an ArduinoCommandOutcome."
        )

    if outcome.command_type is not CommandType.MOVE_FORWARD:
        raise BoundedArduinoMotionDiagnosticError(
            "bounded diagnostic outcome must reference MOVE_FORWARD."
        )

    if outcome.completion_code is not ArduinoCompletionCode.OK:
        raise BoundedArduinoMotionDiagnosticError(
            "bounded MOVE_FORWARD completed with "
            f"{outcome.completion_code.value}."
        )


def _run_diagnostic(
    arguments: Namespace,
) -> BoundedArduinoMotionDiagnosticResult:
    """Execute one guarded command and always request final ESTOP."""

    distance_threshold = (
        _require_positive_distance_threshold(
            arguments.frontal_obstacle_distance_mm
        )
    )

    with open_arduino_serial_transport(
        port=arguments.port,
        baud_rate=arguments.baud_rate,
        read_timeout_seconds=arguments.timeout,
        write_timeout_seconds=arguments.timeout,
    ) as transport:
        client = ArduinoSerialClient(
            transport=transport,
        )

        try:
            ping_sequence = client.ping()
            initial_status = client.request_status()
            _require_latched_status(
                initial_status,
                stage="before",
            )

            sensors = client.request_sensors()
            _require_safe_preflight(
                sensors,
                frontal_obstacle_distance_mm=(
                    distance_threshold
                ),
            )

            rearmed_status = client.rearm()
            outcome = client.execute_command(
                CommandType.MOVE_FORWARD
            )
            _require_successful_completion(outcome)
        finally:
            final_status = client.emergency_stop()

        _require_latched_status(
            final_status,
            stage="after",
        )

    return BoundedArduinoMotionDiagnosticResult(
        ping_sequence=ping_sequence,
        initial_status=initial_status,
        sensors=sensors,
        rearmed_status=rearmed_status,
        outcome=outcome,
        final_status=final_status,
    )


def main(
    argv: Sequence[str] | None = None,
) -> int:
    """Run one explicitly authorized bounded-motion diagnostic."""

    arguments = build_parser().parse_args(argv)

    try:
        _require_explicit_authorization(
            arguments.confirm_wheels_lifted
        )
        result = _run_diagnostic(arguments)
    except (
        ArduinoSerialProtocolError,
        ArduinoSerialSessionError,
        ArduinoSerialTransportError,
        BoundedArduinoMotionDiagnosticError,
        DomainValidationError,
    ) as error:
        print(
            f"Bounded Arduino motion diagnostic failed: {error}",
            file=sys.stderr,
        )
        return 2

    distance_mm = result.sensors.distance_mm
    assert distance_mm is not None

    print("Bounded Arduino motion diagnostic succeeded.")
    print(f"Arduino port: {arguments.port}")
    print(f"Ping sequence: {result.ping_sequence}")
    print(
        "Initial controller state: "
        f"{result.initial_status.device_state.value}/"
        f"{result.initial_status.safety_state.value}"
    )
    print(
        "Preflight sensors: "
        f"left={result.sensors.left_ir_raw}, "
        f"right={result.sensors.right_ir_raw}, "
        f"line={result.sensors.line_raw}, "
        f"distance_mm={distance_mm}"
    )
    print(
        "Rearmed controller state: "
        f"{result.rearmed_status.device_state.value}/"
        f"{result.rearmed_status.safety_state.value}"
    )
    print(
        "Motion completion: "
        f"sequence={result.outcome.sequence_id}, "
        f"command={result.outcome.command_type.value}, "
        f"result={result.outcome.completion_code.value}"
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