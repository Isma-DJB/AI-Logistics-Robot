"""Safe byte transport for the versioned Arduino serial protocol."""

from types import TracebackType
from typing import Protocol, runtime_checkable

from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoSerialResponse,
    parse_arduino_response,
)
from ai_logistics_robot.domain.errors import DomainValidationError


class ArduinoSerialTransportError(RuntimeError):
    """Report one Arduino serial input/output failure."""


class ArduinoSerialConnectionError(
    ArduinoSerialTransportError
):
    """Report an unavailable or interrupted serial connection."""


class ArduinoSerialTimeoutError(
    ArduinoSerialTransportError
):
    """Report a serial read that returned no complete response."""


@runtime_checkable
class ArduinoSerialPort(Protocol):
    """Minimal serial-port surface required by the transport."""

    @property
    def is_open(self) -> bool:
        """Return whether the underlying port is open."""

        ...

    def write(self, data: bytes) -> int:
        """Write bytes and return the number accepted."""

        ...

    def flush(self) -> None:
        """Wait until pending output has been transmitted."""

        ...

    def readline(self) -> bytes:
        """Read one newline-terminated response or return empty bytes."""

        ...

    def reset_input_buffer(self) -> None:
        """Discard stale unread input."""

        ...

    def close(self) -> None:
        """Close the underlying serial port."""

        ...


class ArduinoSerialTransport:
    """Send requests and receive validated Arduino responses."""

    __slots__ = ("_serial_port",)

    def __init__(
        self,
        *,
        serial_port: ArduinoSerialPort,
    ) -> None:
        """Validate and retain one already configured serial port."""

        if not isinstance(serial_port, ArduinoSerialPort):
            raise DomainValidationError(
                "serial_port must satisfy ArduinoSerialPort."
            )

        self._serial_port = serial_port

    @property
    def is_open(self) -> bool:
        """Return whether the underlying connection is open."""

        return self._serial_port.is_open

    def send(self, request: bytes) -> None:
        """Write exactly one complete ASCII protocol request."""

        self._require_open_port()
        self._validate_request(request)

        try:
            written_bytes = self._serial_port.write(request)
            self._serial_port.flush()
        except OSError as error:
            raise ArduinoSerialConnectionError(
                "Arduino serial request could not be written."
            ) from error

        if (
            isinstance(written_bytes, bool)
            or not isinstance(written_bytes, int)
            or written_bytes != len(request)
        ):
            raise ArduinoSerialConnectionError(
                "Arduino serial request was only partially written."
            )

    def receive(self) -> ArduinoSerialResponse:
        """Read and validate one complete Arduino response."""

        self._require_open_port()

        try:
            response_line = self._serial_port.readline()
        except OSError as error:
            raise ArduinoSerialConnectionError(
                "Arduino serial response could not be read."
            ) from error

        if response_line == b"":
            raise ArduinoSerialTimeoutError(
                "Arduino serial response timed out."
            )

        if not isinstance(response_line, bytes):
            raise ArduinoSerialConnectionError(
                "Arduino serial port returned non-binary data."
            )

        return parse_arduino_response(response_line)

    def reset_input_buffer(self) -> None:
        """Discard stale responses before a new session."""

        self._require_open_port()

        try:
            self._serial_port.reset_input_buffer()
        except OSError as error:
            raise ArduinoSerialConnectionError(
                "Arduino serial input buffer could not be reset."
            ) from error

    def close(self) -> None:
        """Close the connection if it remains open."""

        if not self._serial_port.is_open:
            return

        try:
            self._serial_port.close()
        except OSError as error:
            raise ArduinoSerialConnectionError(
                "Arduino serial connection could not be closed."
            ) from error

    def __enter__(self) -> "ArduinoSerialTransport":
        """Return this open transport for context-managed use."""

        self._require_open_port()
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the transport when leaving its context."""

        del exception_type
        del exception
        del traceback
        self.close()

    def _require_open_port(self) -> None:
        """Reject every operation when the port is closed."""

        if not self._serial_port.is_open:
            raise ArduinoSerialConnectionError(
                "Arduino serial connection is closed."
            )

    @staticmethod
    def _validate_request(request: object) -> None:
        """Require exactly one newline-terminated ASCII request."""

        if not isinstance(request, bytes):
            raise DomainValidationError(
                "request must be bytes."
            )

        if (
            not request
            or not request.endswith(b"\n")
            or request.count(b"\n") != 1
            or b"\r" in request
        ):
            raise DomainValidationError(
                "request must contain exactly one LF-terminated line."
            )

        try:
            request.decode("ascii")
        except UnicodeDecodeError as error:
            raise DomainValidationError(
                "request must contain ASCII bytes."
            ) from error