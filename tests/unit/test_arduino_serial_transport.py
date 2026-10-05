"""Unit tests for the safe Arduino serial byte transport."""

import unittest
from typing import cast

from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    ArduinoResponseType,
    ArduinoSerialProtocolError,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialConnectionError,
    ArduinoSerialTimeoutError,
    ArduinoSerialTransport,
)
from ai_logistics_robot.domain.errors import DomainValidationError


class FakeSerialPort:
    """Controllable in-memory substitute for one serial connection."""

    def __init__(
        self,
        *,
        responses: tuple[bytes, ...] = (),
        is_open: bool = True,
    ) -> None:
        """Initialize configurable serial behavior."""

        self._is_open = is_open
        self.responses = list(responses)
        self.writes: list[bytes] = []
        self.flush_calls = 0
        self.reset_calls = 0
        self.close_calls = 0
        self.write_result: int | None = None
        self.write_error: OSError | None = None
        self.read_error: OSError | None = None
        self.reset_error: OSError | None = None
        self.close_error: OSError | None = None

    @property
    def is_open(self) -> bool:
        """Return the configured port state."""

        return self._is_open

    def write(self, data: bytes) -> int:
        """Record or reject one byte write."""

        if self.write_error is not None:
            raise self.write_error

        self.writes.append(data)

        if self.write_result is not None:
            return self.write_result

        return len(data)

    def flush(self) -> None:
        """Record one output flush."""

        self.flush_calls += 1

    def readline(self) -> bytes:
        """Return one configured response or a timeout."""

        if self.read_error is not None:
            raise self.read_error

        if not self.responses:
            return b""

        return self.responses.pop(0)

    def reset_input_buffer(self) -> None:
        """Record or reject one input reset."""

        if self.reset_error is not None:
            raise self.reset_error

        self.reset_calls += 1
        self.responses.clear()

    def close(self) -> None:
        """Record or reject one close operation."""

        if self.close_error is not None:
            raise self.close_error

        self.close_calls += 1
        self._is_open = False


class NonBinaryResponseSerialPort(FakeSerialPort):
    """Return invalid text despite the declared serial contract."""

    def readline(self) -> bytes:
        """Return synthetic non-binary data."""

        return cast(bytes, "not-binary")


class ArduinoSerialTransportTests(unittest.TestCase):
    """Verify safe serial I/O without opening physical hardware."""

    def test_constructor_rejects_invalid_serial_port(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "serial_port",
        ):
            ArduinoSerialTransport(
                serial_port=object(),  # type: ignore[arg-type]
            )

    def test_send_writes_and_flushes_complete_request(self) -> None:
        serial_port = FakeSerialPort()
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )
        request = b"ALR|1|PING|1\n"

        transport.send(request)

        self.assertEqual(serial_port.writes, [request])
        self.assertEqual(serial_port.flush_calls, 1)

    def test_request_must_be_bytes(self) -> None:
        transport = ArduinoSerialTransport(
            serial_port=FakeSerialPort(),
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "must be bytes",
        ):
            transport.send(
                "ALR|1|PING|1\n",  # type: ignore[arg-type]
            )

    def test_request_must_be_one_lf_terminated_line(self) -> None:
        transport = ArduinoSerialTransport(
            serial_port=FakeSerialPort(),
        )
        invalid_requests = (
            b"",
            b"ALR|1|PING|1",
            b"ALR|1|PING|1\r\n",
            b"ALR|1|PING|1\nALR|1|PING|2\n",
        )

        for invalid_request in invalid_requests:
            with self.subTest(
                invalid_request=invalid_request,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "exactly one",
                ):
                    transport.send(invalid_request)

    def test_request_must_contain_ascii(self) -> None:
        transport = ArduinoSerialTransport(
            serial_port=FakeSerialPort(),
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "ASCII",
        ):
            transport.send(
                b"ALR|1|PING|1|\xff\n"
            )

    def test_closed_port_rejects_operations(self) -> None:
        transport = ArduinoSerialTransport(
            serial_port=FakeSerialPort(
                is_open=False,
            )
        )
        self.assertFalse(transport.is_open)

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "closed",
        ):
            transport.send(b"ALR|1|PING|1\n")

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "closed",
        ):
            transport.receive()

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "closed",
        ):
            transport.reset_input_buffer()

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "closed",
        ):
            transport.__enter__()

    def test_partial_or_invalid_write_is_rejected(self) -> None:
        invalid_results = (
            0,
            3,
            True,
        )

        for invalid_result in invalid_results:
            with self.subTest(
                invalid_result=invalid_result,
            ):
                serial_port = FakeSerialPort()
                serial_port.write_result = invalid_result
                transport = ArduinoSerialTransport(
                    serial_port=serial_port,
                )

                with self.assertRaisesRegex(
                    ArduinoSerialConnectionError,
                    "partially written",
                ):
                    transport.send(
                        b"ALR|1|PING|1\n"
                    )

    def test_write_failure_is_wrapped(self) -> None:
        serial_port = FakeSerialPort()
        serial_port.write_error = OSError(
            "synthetic write failure"
        )
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "could not be written",
        ):
            transport.send(b"ALR|1|PING|1\n")

    def test_receive_returns_validated_response(self) -> None:
        serial_port = FakeSerialPort(
            responses=(
                b"ALR|1|PONG|7\n",
            )
        )
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )

        response = transport.receive()

        self.assertEqual(
            response.response_type,
            ArduinoResponseType.PONG,
        )
        self.assertEqual(response.sequence_id, 7)
        self.assertEqual(response.payload, ())

    def test_empty_read_is_a_timeout(self) -> None:
        transport = ArduinoSerialTransport(
            serial_port=FakeSerialPort(),
        )

        with self.assertRaisesRegex(
            ArduinoSerialTimeoutError,
            "timed out",
        ):
            transport.receive()

    def test_read_failure_is_wrapped(self) -> None:
        serial_port = FakeSerialPort()
        serial_port.read_error = OSError(
            "synthetic read failure"
        )
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "could not be read",
        ):
            transport.receive()

    def test_non_binary_response_is_rejected(self) -> None:
        transport = ArduinoSerialTransport(
            serial_port=NonBinaryResponseSerialPort(),
        )

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "non-binary",
        ):
            transport.receive()

    def test_protocol_error_is_preserved(self) -> None:
        transport = ArduinoSerialTransport(
            serial_port=FakeSerialPort(
                responses=(
                    b"INVALID\n",
                )
            )
        )

        with self.assertRaises(
            ArduinoSerialProtocolError
        ):
            transport.receive()

    def test_reset_input_buffer_discards_stale_data(self) -> None:
        serial_port = FakeSerialPort(
            responses=(
                b"ALR|1|PONG|1\n",
            )
        )
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )

        transport.reset_input_buffer()

        self.assertEqual(serial_port.reset_calls, 1)
        self.assertEqual(serial_port.responses, [])

    def test_reset_failure_is_wrapped(self) -> None:
        serial_port = FakeSerialPort()
        serial_port.reset_error = OSError(
            "synthetic reset failure"
        )
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "could not be reset",
        ):
            transport.reset_input_buffer()

    def test_close_is_idempotent(self) -> None:
        serial_port = FakeSerialPort()
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )

        transport.close()
        transport.close()

        self.assertFalse(transport.is_open)
        self.assertEqual(serial_port.close_calls, 1)

    def test_close_failure_is_wrapped(self) -> None:
        serial_port = FakeSerialPort()
        serial_port.close_error = OSError(
            "synthetic close failure"
        )
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )

        with self.assertRaisesRegex(
            ArduinoSerialConnectionError,
            "could not be closed",
        ):
            transport.close()

    def test_context_manager_closes_transport(self) -> None:
        serial_port = FakeSerialPort()
        transport = ArduinoSerialTransport(
            serial_port=serial_port,
        )

        with transport as active_transport:
            self.assertIs(
                active_transport,
                transport,
            )
            self.assertTrue(active_transport.is_open)

        self.assertFalse(transport.is_open)
        self.assertEqual(serial_port.close_calls, 1)


if __name__ == "__main__":
    unittest.main()