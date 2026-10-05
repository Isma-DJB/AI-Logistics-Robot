"""Unit tests for optional PySerial Arduino connections."""

import unittest
from math import inf, nan

from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoResponseType,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialConnectionError,
    ArduinoSerialPort,
)
from ai_logistics_robot.adapters.hardware.pyserial_connection import (
    DEFAULT_BAUD_RATE,
    DEFAULT_READ_TIMEOUT_SECONDS,
    DEFAULT_WRITE_TIMEOUT_SECONDS,
    open_arduino_serial_transport,
)
from ai_logistics_robot.domain.errors import DomainValidationError


class FakeSerialPort:
    """Minimal configurable port satisfying ArduinoSerialPort."""

    def __init__(
        self,
        *,
        is_open: bool = True,
    ) -> None:
        """Initialize one fake port."""

        self._is_open = is_open
        self.writes: list[bytes] = []

    @property
    def is_open(self) -> bool:
        """Return the current fake-port state."""

        return self._is_open

    def write(self, data: bytes) -> int:
        """Record and accept all bytes."""

        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        """Complete one synthetic flush."""

    def readline(self) -> bytes:
        """Return a synthetic read timeout."""

        return b""

    def reset_input_buffer(self) -> None:
        """Complete one synthetic input reset."""

    def close(self) -> None:
        """Close the fake port."""

        self._is_open = False


class PySerialConnectionTests(unittest.TestCase):
    """Verify configuration, failure mapping, and virtual I/O."""

    def test_injected_factory_receives_default_settings(self) -> None:
        serial_port = FakeSerialPort()
        captured_calls: list[
            tuple[str, int, float, float]
        ] = []

        def factory(
            port: str,
            baud_rate: int,
            read_timeout_seconds: float,
            write_timeout_seconds: float,
        ) -> ArduinoSerialPort:
            captured_calls.append(
                (
                    port,
                    baud_rate,
                    read_timeout_seconds,
                    write_timeout_seconds,
                )
            )
            return serial_port

        transport = open_arduino_serial_transport(
            port="  COM3  ",
            serial_factory=factory,
        )

        self.assertTrue(transport.is_open)
        self.assertEqual(
            captured_calls,
            [
                (
                    "COM3",
                    DEFAULT_BAUD_RATE,
                    DEFAULT_READ_TIMEOUT_SECONDS,
                    DEFAULT_WRITE_TIMEOUT_SECONDS,
                )
            ],
        )

    def test_injected_factory_receives_custom_settings(self) -> None:
        captured_calls: list[
            tuple[str, int, float, float]
        ] = []

        def factory(
            port: str,
            baud_rate: int,
            read_timeout_seconds: float,
            write_timeout_seconds: float,
        ) -> ArduinoSerialPort:
            captured_calls.append(
                (
                    port,
                    baud_rate,
                    read_timeout_seconds,
                    write_timeout_seconds,
                )
            )
            return FakeSerialPort()

        open_arduino_serial_transport(
            port="loop://",
            baud_rate=57_600,
            read_timeout_seconds=2,
            write_timeout_seconds=3.5,
            serial_factory=factory,
        )

        self.assertEqual(
            captured_calls,
            [
                (
                    "loop://",
                    57_600,
                    2.0,
                    3.5,
                )
            ],
        )

    def test_rejects_invalid_port(self) -> None:
        invalid_ports = (
            "",
            "   ",
            None,
            3,
        )

        for invalid_port in invalid_ports:
            with self.subTest(
                invalid_port=invalid_port,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "port",
                ):
                    open_arduino_serial_transport(
                        port=invalid_port,  # type: ignore[arg-type]
                    )

    def test_rejects_invalid_baud_rate(self) -> None:
        invalid_baud_rates = (
            True,
            0,
            -1,
            115_200.0,
            "115200",
        )

        for invalid_baud_rate in invalid_baud_rates:
            with self.subTest(
                invalid_baud_rate=invalid_baud_rate,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "baud_rate",
                ):
                    open_arduino_serial_transport(
                        port="COM3",
                        baud_rate=invalid_baud_rate,  # type: ignore[arg-type]
                    )

    def test_rejects_invalid_read_timeout(self) -> None:
        invalid_timeouts = (
            True,
            0,
            -1,
            inf,
            nan,
            "1",
        )

        for invalid_timeout in invalid_timeouts:
            with self.subTest(
                invalid_timeout=invalid_timeout,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "read_timeout_seconds",
                ):
                    open_arduino_serial_transport(
                        port="COM3",
                        read_timeout_seconds=invalid_timeout,  # type: ignore[arg-type]
                    )

    def test_rejects_invalid_write_timeout(self) -> None:
        invalid_timeouts = (
            True,
            0,
            -1,
            inf,
            nan,
            "1",
        )

        for invalid_timeout in invalid_timeouts:
            with self.subTest(
                invalid_timeout=invalid_timeout,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "write_timeout_seconds",
                ):
                    open_arduino_serial_transport(
                        port="COM3",
                        write_timeout_seconds=invalid_timeout,  # type: ignore[arg-type]
                    )

    def test_rejects_invalid_serial_factory(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "serial_factory",
        ):
            open_arduino_serial_transport(
                port="COM3",
                serial_factory=object(),  # type: ignore[arg-type]
            )

    def test_wraps_factory_os_error(self) -> None:
        def failing_factory(
            port: str,
            baud_rate: int,
            read_timeout_seconds: float,
            write_timeout_seconds: float,
        ) -> ArduinoSerialPort:
            del port
            del baud_rate
            del read_timeout_seconds
            del write_timeout_seconds
            raise OSError("synthetic open failure")

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "COM3",
        ):
            open_arduino_serial_transport(
                port="COM3",
                serial_factory=failing_factory,
            )

    def test_wraps_factory_value_error(self) -> None:
        def failing_factory(
            port: str,
            baud_rate: int,
            read_timeout_seconds: float,
            write_timeout_seconds: float,
        ) -> ArduinoSerialPort:
            del port
            del baud_rate
            del read_timeout_seconds
            del write_timeout_seconds
            raise ValueError("synthetic configuration failure")

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "could not be opened",
        ):
            open_arduino_serial_transport(
                port="COM3",
                serial_factory=failing_factory,
            )

    def test_rejects_incompatible_factory_result(self) -> None:
        def invalid_factory(
            port: str,
            baud_rate: int,
            read_timeout_seconds: float,
            write_timeout_seconds: float,
        ) -> ArduinoSerialPort:
            del port
            del baud_rate
            del read_timeout_seconds
            del write_timeout_seconds
            return object()  # type: ignore[return-value]

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "incompatible",
        ):
            open_arduino_serial_transport(
                port="COM3",
                serial_factory=invalid_factory,
            )

    def test_rejects_factory_result_that_is_closed(self) -> None:
        def closed_factory(
            port: str,
            baud_rate: int,
            read_timeout_seconds: float,
            write_timeout_seconds: float,
        ) -> ArduinoSerialPort:
            del port
            del baud_rate
            del read_timeout_seconds
            del write_timeout_seconds
            return FakeSerialPort(
                is_open=False,
            )

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "did not open",
        ):
            open_arduino_serial_transport(
                port="COM3",
                serial_factory=closed_factory,
            )

    def test_real_pyserial_loopback_round_trip(self) -> None:
        with open_arduino_serial_transport(
            port="loop://",
            read_timeout_seconds=0.5,
            write_timeout_seconds=0.5,
        ) as transport:
            transport.send(
                b"ALR|1|PONG|77\n"
            )
            response = transport.receive()

            self.assertEqual(
                response.response_type,
                ArduinoResponseType.PONG,
            )
            self.assertEqual(
                response.sequence_id,
                77,
            )
            self.assertEqual(
                response.payload,
                (),
            )

        self.assertFalse(transport.is_open)


if __name__ == "__main__":
    unittest.main()