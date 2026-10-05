"""Optional PySerial factory for Arduino serial transports."""

from collections.abc import Callable
from math import isfinite
from typing import cast

import serial  # type: ignore[import-untyped]

from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialConnectionError,
    ArduinoSerialPort,
    ArduinoSerialTransport,
)
from ai_logistics_robot.domain.errors import DomainValidationError

DEFAULT_BAUD_RATE = 115_200
DEFAULT_READ_TIMEOUT_SECONDS = 1.0
DEFAULT_WRITE_TIMEOUT_SECONDS = 1.0

SerialPortFactory = Callable[
    [
        str,
        int,
        float,
        float,
    ],
    ArduinoSerialPort,
]


def open_arduino_serial_transport(
    *,
    port: str,
    baud_rate: int = DEFAULT_BAUD_RATE,
    read_timeout_seconds: float = (
        DEFAULT_READ_TIMEOUT_SECONDS
    ),
    write_timeout_seconds: float = (
        DEFAULT_WRITE_TIMEOUT_SECONDS
    ),
    serial_factory: SerialPortFactory | None = None,
) -> ArduinoSerialTransport:
    """Open one validated PySerial-backed Arduino transport."""

    if not isinstance(port, str) or not port.strip():
        raise DomainValidationError(
            "port must be a non-empty string."
        )

    normalized_port = port.strip()
    _validate_baud_rate(baud_rate)

    normalized_read_timeout = _validate_timeout(
        read_timeout_seconds,
        field_name="read_timeout_seconds",
    )
    normalized_write_timeout = _validate_timeout(
        write_timeout_seconds,
        field_name="write_timeout_seconds",
    )

    if (
        serial_factory is not None
        and not callable(serial_factory)
    ):
        raise DomainValidationError(
            "serial_factory must be callable."
        )

    selected_factory = (
        serial_factory or _open_pyserial_port
    )

    try:
        serial_port = selected_factory(
            normalized_port,
            baud_rate,
            normalized_read_timeout,
            normalized_write_timeout,
        )
    except (OSError, ValueError) as error:
        raise ArduinoSerialConnectionError(
            f"Arduino serial port {normalized_port!r} "
            "could not be opened."
        ) from error

    try:
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )
    except DomainValidationError as error:
        raise ArduinoSerialConnectionError(
            "Serial factory returned an incompatible port."
        ) from error

    if not transport.is_open:
        raise ArduinoSerialConnectionError(
            f"Arduino serial port {normalized_port!r} "
            "did not open."
        )

    return transport


def _open_pyserial_port(
    port: str,
    baud_rate: int,
    read_timeout_seconds: float,
    write_timeout_seconds: float,
) -> ArduinoSerialPort:
    """Open one native or URL-based PySerial connection."""

    try:
        serial_port = serial.serial_for_url(
            port,
            baudrate=baud_rate,
            timeout=read_timeout_seconds,
            write_timeout=write_timeout_seconds,
        )
    except serial.SerialException as error:
        raise OSError(
            "PySerial could not open the requested port."
        ) from error

    return cast(
        ArduinoSerialPort,
        serial_port,
    )


def _validate_baud_rate(baud_rate: object) -> None:
    """Require one positive integer baud rate."""

    if (
        isinstance(baud_rate, bool)
        or not isinstance(baud_rate, int)
        or baud_rate <= 0
    ):
        raise DomainValidationError(
            "baud_rate must be a positive integer."
        )


def _validate_timeout(
    timeout_seconds: object,
    *,
    field_name: str,
) -> float:
    """Return one normalized finite positive timeout."""

    if (
        isinstance(timeout_seconds, bool)
        or not isinstance(
            timeout_seconds,
            (int, float),
        )
        or not isfinite(float(timeout_seconds))
        or float(timeout_seconds) <= 0.0
    ):
        raise DomainValidationError(
            f"{field_name} must be a finite positive number."
        )

    return float(timeout_seconds)