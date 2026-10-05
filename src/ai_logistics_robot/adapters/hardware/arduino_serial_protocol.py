"""Versioned line protocol for supervised Arduino robot control."""

from dataclasses import dataclass
from enum import StrEnum, unique

from ai_logistics_robot.domain.enums import CommandType
from ai_logistics_robot.domain.errors import DomainValidationError

PROTOCOL_NAME = "ALR"
PROTOCOL_VERSION = "1"
MAX_SEQUENCE_ID = 2_147_483_647


class ArduinoSerialProtocolError(ValueError):
    """Report one malformed or unsupported Arduino protocol message."""


@unique
class ArduinoRequestType(StrEnum):
    """Requests accepted by the supervised Arduino firmware."""

    COMMAND = "CMD"
    PING = "PING"
    EMERGENCY_STOP = "ESTOP"
    REARM = "REARM"
    STATUS = "STATUS"


@unique
class ArduinoResponseType(StrEnum):
    """Responses produced by the supervised Arduino firmware."""

    READY = "READY"
    PONG = "PONG"
    ACKNOWLEDGED = "ACK"
    COMPLETED = "DONE"
    ERROR = "ERROR"
    STATUS = "STATUS"


@unique
class ArduinoCompletionCode(StrEnum):
    """Terminal outcomes for one accepted command."""

    OK = "OK"
    BLOCKED = "BLOCKED"
    TIMEOUT = "TIMEOUT"
    COMMUNICATION_LOSS = "COMMUNICATION_LOSS"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@unique
class ArduinoErrorCode(StrEnum):
    """Explicit protocol and execution rejections."""

    BAD_FORMAT = "BAD_FORMAT"
    BAD_VERSION = "BAD_VERSION"
    BAD_SEQUENCE = "BAD_SEQUENCE"
    BAD_COMMAND = "BAD_COMMAND"
    BUSY = "BUSY"
    SAFETY_LATCHED = "SAFETY_LATCHED"


@unique
class ArduinoDeviceState(StrEnum):
    """Observable execution states of the Arduino controller."""

    IDLE = "IDLE"
    EXECUTING = "EXECUTING"
    EMERGENCY_STOPPED = "ESTOPPED"


@unique
class ArduinoSafetyState(StrEnum):
    """Observable Arduino safety-latch states."""

    SAFE = "SAFE"
    LATCHED = "LATCHED"


@dataclass(frozen=True, slots=True)
class ArduinoSerialResponse:
    """One validated response decoded from the Arduino."""

    response_type: ArduinoResponseType
    sequence_id: int | None
    payload: tuple[str, ...]

    def __post_init__(self) -> None:
        """Validate response shape and vocabulary."""

        if not isinstance(
            self.response_type,
            ArduinoResponseType,
        ):
            raise ArduinoSerialProtocolError(
                "response_type must be an ArduinoResponseType."
            )

        _validate_payload_tokens(self.payload)

        if self.response_type is ArduinoResponseType.READY:
            if self.sequence_id is not None:
                raise ArduinoSerialProtocolError(
                    "READY must not contain a sequence identifier."
                )
            if len(self.payload) != 1:
                raise ArduinoSerialProtocolError(
                    "READY must contain one firmware-version token."
                )
            return

        if not _is_valid_sequence_id(self.sequence_id):
            raise ArduinoSerialProtocolError(
                "response sequence identifier is invalid."
            )

        if self.response_type is ArduinoResponseType.PONG:
            if self.payload:
                raise ArduinoSerialProtocolError(
                    "PONG must not contain a payload."
                )
            return

        if self.response_type is ArduinoResponseType.ACKNOWLEDGED:
            _require_single_enum_payload(
                self.payload,
                CommandType,
                "ACK",
            )
            return

        if self.response_type is ArduinoResponseType.COMPLETED:
            _require_single_enum_payload(
                self.payload,
                ArduinoCompletionCode,
                "DONE",
            )
            return

        if self.response_type is ArduinoResponseType.ERROR:
            _require_single_enum_payload(
                self.payload,
                ArduinoErrorCode,
                "ERROR",
            )
            return

        if self.response_type is ArduinoResponseType.STATUS:
            if len(self.payload) != 2:
                raise ArduinoSerialProtocolError(
                    "STATUS must contain device and safety states."
                )

            _require_enum_token(
                self.payload[0],
                ArduinoDeviceState,
                "STATUS device state",
            )
            _require_enum_token(
                self.payload[1],
                ArduinoSafetyState,
                "STATUS safety state",
            )


def encode_motion_command(
    *,
    sequence_id: int,
    command_type: CommandType,
) -> bytes:
    """Encode one platform-independent motion command."""

    if not isinstance(command_type, CommandType):
        raise DomainValidationError(
            "command_type must be a CommandType instance."
        )

    return _encode_request(
        ArduinoRequestType.COMMAND,
        sequence_id,
        command_type.value,
    )


def encode_ping(*, sequence_id: int) -> bytes:
    """Encode one connection-liveness request."""

    return _encode_request(
        ArduinoRequestType.PING,
        sequence_id,
    )


def encode_emergency_stop(*, sequence_id: int) -> bytes:
    """Encode one priority emergency-stop request."""

    return _encode_request(
        ArduinoRequestType.EMERGENCY_STOP,
        sequence_id,
    )


def encode_rearm(*, sequence_id: int) -> bytes:
    """Encode one explicit safety-latch rearm request."""

    return _encode_request(
        ArduinoRequestType.REARM,
        sequence_id,
    )


def encode_status_request(*, sequence_id: int) -> bytes:
    """Encode one controller-status request."""

    return _encode_request(
        ArduinoRequestType.STATUS,
        sequence_id,
    )


def parse_arduino_response(
    line: object,
) -> ArduinoSerialResponse:
    """Decode and validate one complete Arduino response line."""

    if isinstance(line, bytes):
        try:
            decoded_line = line.decode("ascii")
        except UnicodeDecodeError as error:
            raise ArduinoSerialProtocolError(
                "Arduino response must contain ASCII text."
            ) from error
    elif isinstance(line, str):
        decoded_line = line
    else:
        raise ArduinoSerialProtocolError(
            "Arduino response must be bytes or a string."
        )

    normalized_line = decoded_line.rstrip("\r\n")

    if (
        not normalized_line
        or "\r" in normalized_line
        or "\n" in normalized_line
    ):
        raise ArduinoSerialProtocolError(
            "Arduino response must contain exactly one non-empty line."
        )

    fields = tuple(normalized_line.split("|"))

    if len(fields) < 3 or fields[0] != PROTOCOL_NAME:
        raise ArduinoSerialProtocolError(
            "Arduino response has an invalid protocol name."
        )

    if fields[1] != PROTOCOL_VERSION:
        raise ArduinoSerialProtocolError(
            "Arduino response has an unsupported protocol version."
        )

    try:
        response_type = ArduinoResponseType(fields[2])
    except ValueError as error:
        raise ArduinoSerialProtocolError(
            "Arduino response type is unsupported."
        ) from error

    if response_type is ArduinoResponseType.READY:
        if len(fields) != 4:
            raise ArduinoSerialProtocolError(
                "READY response has an invalid field count."
            )

        return ArduinoSerialResponse(
            response_type=response_type,
            sequence_id=None,
            payload=(fields[3],),
        )

    if len(fields) < 4:
        raise ArduinoSerialProtocolError(
            "Arduino response is missing its sequence identifier."
        )

    sequence_id = _parse_sequence_id(fields[3])

    return ArduinoSerialResponse(
        response_type=response_type,
        sequence_id=sequence_id,
        payload=fields[4:],
    )


def _encode_request(
    request_type: ArduinoRequestType,
    sequence_id: int,
    *payload: str,
) -> bytes:
    """Encode one validated newline-terminated request."""

    _validate_outbound_sequence_id(sequence_id)
    _validate_payload_tokens(payload)

    fields = (
        PROTOCOL_NAME,
        PROTOCOL_VERSION,
        request_type.value,
        str(sequence_id),
        *payload,
    )
    return ("|".join(fields) + "\n").encode("ascii")


def _validate_outbound_sequence_id(
    sequence_id: object,
) -> None:
    """Validate a host-generated positive sequence identifier."""

    if not _is_valid_sequence_id(sequence_id):
        raise DomainValidationError(
            "sequence_id must be an integer from 1 "
            f"through {MAX_SEQUENCE_ID}."
        )


def _parse_sequence_id(token: str) -> int:
    """Parse one positive decimal response identifier."""

    if not token.isdecimal():
        raise ArduinoSerialProtocolError(
            "Arduino response sequence identifier is invalid."
        )

    sequence_id = int(token)

    if not _is_valid_sequence_id(sequence_id):
        raise ArduinoSerialProtocolError(
            "Arduino response sequence identifier is invalid."
        )

    return sequence_id


def _is_valid_sequence_id(value: object) -> bool:
    """Return whether a value is an accepted sequence identifier."""

    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 1 <= value <= MAX_SEQUENCE_ID
    )


def _validate_payload_tokens(
    payload: tuple[str, ...],
) -> None:
    """Reject empty, spaced, delimited, or non-ASCII tokens."""

    if not isinstance(payload, tuple):
        raise ArduinoSerialProtocolError(
            "response payload must be a tuple."
        )

    for token in payload:
        if (
            not isinstance(token, str)
            or not token
            or token != token.strip()
            or "|" in token
            or "\r" in token
            or "\n" in token
            or not token.isascii()
        ):
            raise ArduinoSerialProtocolError(
                "protocol payload contains an invalid token."
            )


def _require_single_enum_payload(
    payload: tuple[str, ...],
    enum_type: type[StrEnum],
    response_name: str,
) -> None:
    """Require one token belonging to a selected enumeration."""

    if len(payload) != 1:
        raise ArduinoSerialProtocolError(
            f"{response_name} must contain exactly one payload token."
        )

    _require_enum_token(
        payload[0],
        enum_type,
        f"{response_name} payload",
    )


def _require_enum_token(
    token: str,
    enum_type: type[StrEnum],
    field_name: str,
) -> None:
    """Require one token belonging to a selected string enumeration."""

    try:
        enum_type(token)
    except ValueError as error:
        raise ArduinoSerialProtocolError(
            f"{field_name} is unsupported."
        ) from error