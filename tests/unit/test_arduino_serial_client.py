"""Unit tests for the sequenced Arduino serial client."""

import unittest
from typing import cast

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoCommandOutcome,
    ArduinoCommandRejectedError,
    ArduinoControllerStatus,
    ArduinoSequenceMismatchError,
    ArduinoSerialClient,
    ArduinoUnexpectedResponseError,
)
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
    encode_status_request,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_transport import (
    ArduinoSerialTransport,
)
from ai_logistics_robot.domain.enums import CommandType
from ai_logistics_robot.domain.errors import DomainValidationError


class _ScriptedTransport(ArduinoSerialTransport):
    """Provide deterministic responses without opening a serial port."""

    __slots__ = (
        "_responses",
        "closed",
        "requests",
    )

    def __init__(
        self,
        *responses: ArduinoSerialResponse,
    ) -> None:
        """Retain scripted responses and transmitted requests."""

        self._responses: list[ArduinoSerialResponse] = list(
            responses
        )
        self.requests: list[bytes] = []
        self.closed = False

    @property
    def is_open(self) -> bool:
        """Return whether this scripted transport is open."""

        return not self.closed

    def send(self, request: bytes) -> None:
        """Record one outgoing request."""

        self.requests.append(request)

    def receive(self) -> ArduinoSerialResponse:
        """Return the next scripted response."""

        if not self._responses:
            raise AssertionError(
                "No scripted Arduino response remains."
            )

        return self._responses.pop(0)

    def close(self) -> None:
        """Record transport closure."""

        self.closed = True


def _ready_response(
    firmware_version: str = "i-1.0",
) -> ArduinoSerialResponse:
    """Create one valid READY response."""

    return ArduinoSerialResponse(
        response_type=ArduinoResponseType.READY,
        sequence_id=None,
        payload=(firmware_version,),
    )


def _sequenced_response(
    response_type: ArduinoResponseType,
    sequence_id: int,
    *payload: str,
) -> ArduinoSerialResponse:
    """Create one valid sequenced response."""

    return ArduinoSerialResponse(
        response_type=response_type,
        sequence_id=sequence_id,
        payload=payload,
    )


class ArduinoSerialClientTests(unittest.TestCase):
    """Verify supervised Arduino exchanges and sequence handling."""

    def test_constructor_exposes_valid_dependencies(self) -> None:
        transport = _ScriptedTransport()

        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=17,
        )

        self.assertIs(client.transport, transport)
        self.assertEqual(client.next_sequence_id, 17)

    def test_constructor_rejects_invalid_dependencies(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "transport must be",
        ):
            ArduinoSerialClient(
                transport=cast(
                    ArduinoSerialTransport,
                    object(),
                )
            )

        invalid_sequence_ids: tuple[object, ...] = (
            0,
            -1,
            MAX_SEQUENCE_ID + 1,
            True,
            1.5,
            "1",
        )

        for invalid_sequence_id in invalid_sequence_ids:
            with self.subTest(
                invalid_sequence_id=invalid_sequence_id,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "initial_sequence_id",
                ):
                    ArduinoSerialClient(
                        transport=_ScriptedTransport(),
                        initial_sequence_id=cast(
                            int,
                            invalid_sequence_id,
                        ),
                    )

    def test_wait_until_ready_returns_firmware_version(self) -> None:
        transport = _ScriptedTransport(
            _ready_response("i-1.0-test"),
        )
        client = ArduinoSerialClient(
            transport=transport,
        )

        firmware_version = client.wait_until_ready()

        self.assertEqual(firmware_version, "i-1.0-test")
        self.assertEqual(transport.requests, [])
        self.assertEqual(client.next_sequence_id, 1)

    def test_wait_until_ready_rejects_other_response(self) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.PONG,
                1,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
        )

        with self.assertRaisesRegex(
            ArduinoUnexpectedResponseError,
            "Expected READY",
        ):
            client.wait_until_ready()

    def test_ping_sends_request_and_confirms_sequence(self) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.PONG,
                7,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=7,
        )

        sequence_id = client.ping()

        self.assertEqual(sequence_id, 7)
        self.assertEqual(
            transport.requests,
            [encode_ping(sequence_id=7)],
        )
        self.assertEqual(client.next_sequence_id, 8)

    def test_sequence_mismatch_is_rejected(self) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.PONG,
                9,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=8,
        )

        with self.assertRaisesRegex(
            ArduinoSequenceMismatchError,
            "request 8",
        ):
            client.ping()

    def test_explicit_rejection_is_preserved(self) -> None:
        acknowledgement = _sequenced_response(
            ArduinoResponseType.ACKNOWLEDGED,
            11,
            CommandType.MOVE_FORWARD.value,
        )
        rejection = _sequenced_response(
            ArduinoResponseType.ERROR,
            11,
            ArduinoErrorCode.BUSY.value,
        )

        cases = (
            (
                "before acknowledgement",
                (rejection,),
            ),
            (
                "after acknowledgement",
                (
                    acknowledgement,
                    rejection,
                ),
            ),
        )

        for phase, responses in cases:
            with self.subTest(phase=phase):
                transport = _ScriptedTransport(
                    *responses
                )
                client = ArduinoSerialClient(
                    transport=transport,
                    initial_sequence_id=11,
                )

                with self.assertRaises(
                    ArduinoCommandRejectedError
                ) as caught:
                    client.execute_command(
                        CommandType.MOVE_FORWARD
                    )

                self.assertEqual(
                    caught.exception.sequence_id,
                    11,
                )
                self.assertIs(
                    caught.exception.error_code,
                    ArduinoErrorCode.BUSY,
                )

    def test_status_response_is_decoded(self) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.STATUS,
                21,
                ArduinoDeviceState.EXECUTING.value,
                ArduinoSafetyState.SAFE.value,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=21,
        )

        status = client.request_status()

        self.assertEqual(
            status,
            ArduinoControllerStatus(
                sequence_id=21,
                device_state=ArduinoDeviceState.EXECUTING,
                safety_state=ArduinoSafetyState.SAFE,
            ),
        )
        self.assertEqual(
            transport.requests,
            [encode_status_request(sequence_id=21)],
        )

    def test_emergency_stop_requires_latched_confirmation(
        self,
    ) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.STATUS,
                31,
                ArduinoDeviceState.EMERGENCY_STOPPED.value,
                ArduinoSafetyState.LATCHED.value,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=31,
        )

        status = client.emergency_stop()

        self.assertEqual(
            status,
            ArduinoControllerStatus(
                sequence_id=31,
                device_state=(
                    ArduinoDeviceState.EMERGENCY_STOPPED
                ),
                safety_state=ArduinoSafetyState.LATCHED,
            ),
        )
        self.assertEqual(
            transport.requests,
            [encode_emergency_stop(sequence_id=31)],
        )

    def test_emergency_stop_rejects_unsafe_confirmation(
        self,
    ) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.STATUS,
                32,
                ArduinoDeviceState.IDLE.value,
                ArduinoSafetyState.SAFE.value,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=32,
        )

        with self.assertRaisesRegex(
            ArduinoUnexpectedResponseError,
            "ESTOPPED/LATCHED",
        ):
            client.emergency_stop()

    def test_rearm_requires_safe_idle_confirmation(self) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.STATUS,
                41,
                ArduinoDeviceState.IDLE.value,
                ArduinoSafetyState.SAFE.value,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=41,
        )

        status = client.rearm()

        self.assertEqual(
            status,
            ArduinoControllerStatus(
                sequence_id=41,
                device_state=ArduinoDeviceState.IDLE,
                safety_state=ArduinoSafetyState.SAFE,
            ),
        )
        self.assertEqual(
            transport.requests,
            [encode_rearm(sequence_id=41)],
        )

    def test_rearm_rejects_latched_confirmation(self) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.STATUS,
                42,
                ArduinoDeviceState.EMERGENCY_STOPPED.value,
                ArduinoSafetyState.LATCHED.value,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=42,
        )

        with self.assertRaisesRegex(
            ArduinoUnexpectedResponseError,
            "IDLE/SAFE",
        ):
            client.rearm()

    def test_executes_every_motion_command(self) -> None:
        for command_type in CommandType:
            with self.subTest(command_type=command_type):
                transport = _ScriptedTransport(
                    _sequenced_response(
                        ArduinoResponseType.ACKNOWLEDGED,
                        51,
                        command_type.value,
                    ),
                    _sequenced_response(
                        ArduinoResponseType.COMPLETED,
                        51,
                        ArduinoCompletionCode.OK.value,
                    ),
                )
                client = ArduinoSerialClient(
                    transport=transport,
                    initial_sequence_id=51,
                )

                outcome = client.execute_command(
                    command_type
                )

                self.assertEqual(
                    outcome,
                    ArduinoCommandOutcome(
                        sequence_id=51,
                        command_type=command_type,
                        completion_code=(
                            ArduinoCompletionCode.OK
                        ),
                    ),
                )
                self.assertEqual(
                    transport.requests,
                    [
                        encode_motion_command(
                            sequence_id=51,
                            command_type=command_type,
                        )
                    ],
                )
                self.assertEqual(
                    client.next_sequence_id,
                    52,
                )

    def test_preserves_every_completion_code(self) -> None:
        for completion_code in ArduinoCompletionCode:
            with self.subTest(
                completion_code=completion_code,
            ):
                transport = _ScriptedTransport(
                    _sequenced_response(
                        ArduinoResponseType.ACKNOWLEDGED,
                        61,
                        CommandType.MOVE_FORWARD.value,
                    ),
                    _sequenced_response(
                        ArduinoResponseType.COMPLETED,
                        61,
                        completion_code.value,
                    ),
                )
                client = ArduinoSerialClient(
                    transport=transport,
                    initial_sequence_id=61,
                )

                outcome = client.execute_command(
                    CommandType.MOVE_FORWARD
                )

                self.assertIs(
                    outcome.completion_code,
                    completion_code,
                )

    def test_invalid_command_is_rejected_before_sending(
        self,
    ) -> None:
        transport = _ScriptedTransport()
        client = ArduinoSerialClient(
            transport=transport,
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "command_type",
        ):
            client.execute_command(
                cast(
                    CommandType,
                    "MOVE_FORWARD",
                )
            )

        self.assertEqual(transport.requests, [])
        self.assertEqual(client.next_sequence_id, 1)

    def test_mismatched_acknowledgement_command_is_rejected(
        self,
    ) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.ACKNOWLEDGED,
                71,
                CommandType.TURN_LEFT.value,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=71,
        )

        with self.assertRaisesRegex(
            ArduinoUnexpectedResponseError,
            "ACK command",
        ):
            client.execute_command(
                CommandType.MOVE_FORWARD
            )

    def test_unexpected_acknowledgement_type_is_rejected(
        self,
    ) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.PONG,
                72,
            )
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=72,
        )

        with self.assertRaisesRegex(
            ArduinoUnexpectedResponseError,
            "Expected Arduino response ACK",
        ):
            client.execute_command(
                CommandType.MOVE_FORWARD
            )

    def test_unexpected_completion_type_is_rejected(
        self,
    ) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.ACKNOWLEDGED,
                73,
                CommandType.MOVE_FORWARD.value,
            ),
            _sequenced_response(
                ArduinoResponseType.PONG,
                73,
            ),
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=73,
        )

        with self.assertRaisesRegex(
            ArduinoUnexpectedResponseError,
            "Expected Arduino response DONE",
        ):
            client.execute_command(
                CommandType.MOVE_FORWARD
            )

    def test_sequence_wraps_after_maximum_identifier(self) -> None:
        transport = _ScriptedTransport(
            _sequenced_response(
                ArduinoResponseType.PONG,
                MAX_SEQUENCE_ID,
            ),
            _sequenced_response(
                ArduinoResponseType.PONG,
                1,
            ),
        )
        client = ArduinoSerialClient(
            transport=transport,
            initial_sequence_id=MAX_SEQUENCE_ID,
        )

        first_sequence = client.ping()
        second_sequence = client.ping()

        self.assertEqual(first_sequence, MAX_SEQUENCE_ID)
        self.assertEqual(second_sequence, 1)
        self.assertEqual(client.next_sequence_id, 2)
        self.assertEqual(
            transport.requests,
            [
                encode_ping(
                    sequence_id=MAX_SEQUENCE_ID,
                ),
                encode_ping(sequence_id=1),
            ],
        )

    def test_close_delegates_to_transport(self) -> None:
        transport = _ScriptedTransport()
        client = ArduinoSerialClient(
            transport=transport,
        )

        client.close()

        self.assertTrue(transport.closed)


if __name__ == "__main__":
    unittest.main()