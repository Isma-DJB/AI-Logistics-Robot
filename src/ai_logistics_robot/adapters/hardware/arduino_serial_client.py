"""Sequenced client for supervised Arduino robot control."""

from dataclasses import dataclass

from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    MAX_SEQUENCE_ID,
    ArduinoCompletionCode,
    ArduinoDeviceState,
    ArduinoErrorCode,
    ArduinoResponseType,
    ArduinoSafetyState,
    ArduinoSerialResponse,
    encode_emergency_stop,
    encode_motion_command,
    encode_ping,
    encode_rearm,
    encode_sensor_request,
    encode_status_request,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialTransport,
)
from ai_logistics_robot.domain.enums import CommandType
from ai_logistics_robot.domain.errors import DomainValidationError

_MAX_ULTRASONIC_DISTANCE_MM = 10_000
_MAX_LIGHT_RAW = 1_023

class ArduinoSerialSessionError(RuntimeError):
    """Report one invalid supervised serial exchange."""


class ArduinoUnexpectedResponseError(
    ArduinoSerialSessionError
):
    """Report a response type or payload that was not expected."""


class ArduinoSequenceMismatchError(
    ArduinoSerialSessionError
):
    """Report a response belonging to another request."""


class ArduinoCommandRejectedError(
    ArduinoSerialSessionError
):
    """Report one explicit Arduino rejection."""

    def __init__(
        self,
        *,
        sequence_id: int,
        error_code: ArduinoErrorCode,
    ) -> None:
        """Retain the rejected sequence and error code."""

        self.sequence_id = sequence_id
        self.error_code = error_code
        super().__init__(
            "Arduino rejected sequence "
            f"{sequence_id}: {error_code.value}."
        )


@dataclass(frozen=True, slots=True)
class ArduinoControllerStatus:
    """Confirmed controller and safety states."""

    sequence_id: int
    device_state: ArduinoDeviceState
    safety_state: ArduinoSafetyState

@dataclass(frozen=True, slots=True)
class ArduinoSensorSnapshot:
    """One sequenced raw physical-sensor snapshot."""

    sequence_id: int
    left_ir_raw: int
    right_ir_raw: int
    line_raw: int
    distance_mm: int | None
    light_raw: int

@dataclass(frozen=True, slots=True)
class ArduinoCommandOutcome:
    """Confirmed terminal outcome of one motion command."""

    sequence_id: int
    command_type: CommandType
    completion_code: ArduinoCompletionCode


class ArduinoSerialClient:
    """Execute sequenced exchanges through one serial transport."""

    __slots__ = (
        "_next_sequence_id",
        "_transport",
    )

    def __init__(
        self,
        *,
        transport: ArduinoSerialTransport,
        initial_sequence_id: int = 1,
    ) -> None:
        """Validate dependencies and initialize sequence allocation."""

        if not isinstance(
            transport,
            ArduinoSerialTransport,
        ):
            raise DomainValidationError(
                "transport must be an ArduinoSerialTransport."
            )

        if not _is_valid_sequence_id(
            initial_sequence_id
        ):
            raise DomainValidationError(
                "initial_sequence_id must be an integer "
                f"from 1 through {MAX_SEQUENCE_ID}."
            )

        self._transport = transport
        self._next_sequence_id = initial_sequence_id

    @property
    def transport(self) -> ArduinoSerialTransport:
        """Return the configured serial transport."""

        return self._transport

    @property
    def next_sequence_id(self) -> int:
        """Return the identifier reserved for the next request."""

        return self._next_sequence_id

    def wait_until_ready(self) -> str:
        """Receive the firmware boot-ready announcement."""

        response = self._transport.receive()

        if response.response_type is not ArduinoResponseType.READY:
            raise ArduinoUnexpectedResponseError(
                "Expected READY response from Arduino."
            )

        return response.payload[0]

    def ping(self) -> int:
        """Verify liveness and return the confirmed sequence."""

        sequence_id = self._allocate_sequence_id()
        self._transport.send(
            encode_ping(
                sequence_id=sequence_id,
            )
        )
        self._receive_expected(
            sequence_id=sequence_id,
            response_type=ArduinoResponseType.PONG,
        )
        return sequence_id

    def request_status(self) -> ArduinoControllerStatus:
        """Request and decode current controller status."""

        sequence_id = self._allocate_sequence_id()
        self._transport.send(
            encode_status_request(
                sequence_id=sequence_id,
            )
        )
        response = self._receive_expected(
            sequence_id=sequence_id,
            response_type=ArduinoResponseType.STATUS,
        )
        return _decode_status(response)

    def request_sensors(self) -> ArduinoSensorSnapshot:
        """Request and decode one raw physical-sensor snapshot."""

        sequence_id = self._allocate_sequence_id()
        self._transport.send(
            encode_sensor_request(
                sequence_id=sequence_id,
            )
        )
        response = self._receive_expected(
            sequence_id=sequence_id,
            response_type=ArduinoResponseType.SENSORS,
        )
        return _decode_sensor_snapshot(response)

    def emergency_stop(self) -> ArduinoControllerStatus:
        """Request a latched emergency stop and decode status."""

        sequence_id = self._allocate_sequence_id()
        self._transport.send(
            encode_emergency_stop(
                sequence_id=sequence_id,
            )
        )
        response = self._receive_expected(
            sequence_id=sequence_id,
            response_type=ArduinoResponseType.STATUS,
        )
        status = _decode_status(response)

        if (
            status.device_state
            is not ArduinoDeviceState.EMERGENCY_STOPPED
            or status.safety_state
            is not ArduinoSafetyState.LATCHED
        ):
            raise ArduinoUnexpectedResponseError(
                "Emergency stop did not confirm ESTOPPED/LATCHED."
            )

        return status

    def rearm(self) -> ArduinoControllerStatus:
        """Request explicit rearm and decode safe idle status."""

        sequence_id = self._allocate_sequence_id()
        self._transport.send(
            encode_rearm(
                sequence_id=sequence_id,
            )
        )
        response = self._receive_expected(
            sequence_id=sequence_id,
            response_type=ArduinoResponseType.STATUS,
        )
        status = _decode_status(response)

        if (
            status.device_state
            is not ArduinoDeviceState.IDLE
            or status.safety_state
            is not ArduinoSafetyState.SAFE
        ):
            raise ArduinoUnexpectedResponseError(
                "Rearm did not confirm IDLE/SAFE."
            )

        return status

    def execute_command(
        self,
        command_type: CommandType,
    ) -> ArduinoCommandOutcome:
        """Require matching ACK and DONE for one motion command."""

        if not isinstance(command_type, CommandType):
            raise DomainValidationError(
                "command_type must be a CommandType instance."
            )

        sequence_id = self._allocate_sequence_id()
        self._transport.send(
            encode_motion_command(
                sequence_id=sequence_id,
                command_type=command_type,
            )
        )

        acknowledgement = self._receive_expected(
            sequence_id=sequence_id,
            response_type=(
                ArduinoResponseType.ACKNOWLEDGED
            ),
        )

        if acknowledgement.payload != (
            command_type.value,
        ):
            raise ArduinoUnexpectedResponseError(
                "Arduino ACK command does not match the request."
            )

        completion = self._receive_expected(
            sequence_id=sequence_id,
            response_type=ArduinoResponseType.COMPLETED,
        )

        return ArduinoCommandOutcome(
            sequence_id=sequence_id,
            command_type=command_type,
            completion_code=ArduinoCompletionCode(
                completion.payload[0]
            ),
        )

    def close(self) -> None:
        """Close the underlying serial transport."""

        self._transport.close()

    def _receive_expected(
        self,
        *,
        sequence_id: int,
        response_type: ArduinoResponseType,
    ) -> ArduinoSerialResponse:
        """Require one matching non-error response."""

        response = self._transport.receive()

        if response.sequence_id != sequence_id:
            raise ArduinoSequenceMismatchError(
                "Arduino response sequence does not match "
                f"request {sequence_id}."
            )

        if response.response_type is ArduinoResponseType.ERROR:
            raise ArduinoCommandRejectedError(
                sequence_id=sequence_id,
                error_code=ArduinoErrorCode(
                    response.payload[0]
                ),
            )

        if response.response_type is not response_type:
            raise ArduinoUnexpectedResponseError(
                "Expected Arduino response "
                f"{response_type.value}, received "
                f"{response.response_type.value}."
            )

        return response

    def _allocate_sequence_id(self) -> int:
        """Allocate one identifier and wrap after the maximum."""

        sequence_id = self._next_sequence_id

        if sequence_id == MAX_SEQUENCE_ID:
            self._next_sequence_id = 1
        else:
            self._next_sequence_id += 1

        return sequence_id


def _decode_status(
    response: ArduinoSerialResponse,
) -> ArduinoControllerStatus:
    """Translate one validated STATUS response."""

    if (
        response.sequence_id is None
        or response.response_type
        is not ArduinoResponseType.STATUS
    ):
        raise ArduinoUnexpectedResponseError(
            "Expected one sequenced STATUS response."
        )

    return ArduinoControllerStatus(
        sequence_id=response.sequence_id,
        device_state=ArduinoDeviceState(
            response.payload[0]
        ),
        safety_state=ArduinoSafetyState(
            response.payload[1]
        ),
    )

def _decode_sensor_snapshot(
    response: ArduinoSerialResponse,
) -> ArduinoSensorSnapshot:
    """Translate one validated SENSORS response."""

    if (
        response.sequence_id is None
        or response.response_type
        is not ArduinoResponseType.SENSORS
    ):
        raise ArduinoUnexpectedResponseError(
            "Expected one sequenced SENSORS response."
        )

    distance_token = response.payload[3]
    distance_mm = (
        None
        if distance_token == "NONE"
        else int(distance_token)
    )
    light_raw = int(response.payload[4])

    if (
        distance_mm is not None
        and distance_mm > _MAX_ULTRASONIC_DISTANCE_MM
    ):
        raise ArduinoUnexpectedResponseError(
            "Arduino SENSORS distance exceeds 10000 mm."
        )

    if light_raw > _MAX_LIGHT_RAW:
        raise ArduinoUnexpectedResponseError(
            "Arduino SENSORS light value exceeds 1023."
        )

    return ArduinoSensorSnapshot(
        sequence_id=response.sequence_id,
        left_ir_raw=int(response.payload[0]),
        right_ir_raw=int(response.payload[1]),
        line_raw=int(response.payload[2]),
        distance_mm=distance_mm,
        light_raw=light_raw,
    )


def _is_valid_sequence_id(value: object) -> bool:
    """Return whether a sequence identifier is accepted."""

    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 1 <= value <= MAX_SEQUENCE_ID
    )