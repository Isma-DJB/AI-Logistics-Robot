"""Run one safe Arduino serial connection diagnostic."""

import sys
from argparse import ArgumentParser
from collections.abc import Sequence

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoSerialClient,
    ArduinoSerialSessionError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoSerialProtocolError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialTransportError,
)
from ai_logistics_robot.adapters.hardware.pyserial_connection import (
    DEFAULT_BAUD_RATE,
    open_arduino_serial_transport,
)
from ai_logistics_robot.domain.errors import DomainValidationError


def build_parser() -> ArgumentParser:
    """Build the safe Arduino diagnostic command line."""

    parser = ArgumentParser(
        description=(
            "Verify Arduino serial liveness and read controller "
            "status without rearming or sending motion commands."
        )
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Arduino serial port, for example COM3.",
    )
    parser.add_argument(
        "--baud-rate",
        default=DEFAULT_BAUD_RATE,
        type=int,
        help=(
            "Positive serial baud rate. "
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


def main(
    argv: Sequence[str] | None = None,
) -> int:
    """Run one non-moving Arduino connection diagnostic."""

    arguments = build_parser().parse_args(argv)

    try:
        with open_arduino_serial_transport(
            port=arguments.port,
            baud_rate=arguments.baud_rate,
            read_timeout_seconds=arguments.timeout,
            write_timeout_seconds=arguments.timeout,
        ) as transport:
            client = ArduinoSerialClient(
                transport=transport,
            )
            ping_sequence = client.ping()
            status = client.request_status()
    except (
        ArduinoSerialProtocolError,
        ArduinoSerialSessionError,
        ArduinoSerialTransportError,
        DomainValidationError,
    ) as error:
        print(
            f"Arduino serial diagnostic failed: {error}",
            file=sys.stderr,
        )
        return 2

    print("Arduino serial diagnostic succeeded.")
    print(f"Port: {arguments.port}")
    print(f"Baud rate: {arguments.baud_rate}")
    print(f"Ping sequence: {ping_sequence}")
    print(f"Status sequence: {status.sequence_id}")
    print(f"Device state: {status.device_state.value}")
    print(f"Safety state: {status.safety_state.value}")
    print("No rearm or motion command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())