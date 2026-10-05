"""Unit tests for the versioned Arduino serial protocol."""

import unittest

from ai_logistics_robot.adapters.hardware.arduino_serial_protocol import (
    MAX_SEQUENCE_ID,
    ArduinoCompletionCode,
    ArduinoDeviceState,
    ArduinoErrorCode,
    ArduinoResponseType,
    ArduinoSafetyState,
    ArduinoSerialProtocolError,
    ArduinoSerialResponse,
    encode_emergency_stop,
    encode_motion_command,
    encode_ping,
    encode_rearm,
    encode_status_request,
    parse_arduino_response,
)
from ai_logistics_robot.domain.enums import CommandType
from ai_logistics_robot.domain.errors import DomainValidationError


class ArduinoSerialProtocolTests(unittest.TestCase):
    """Verify deterministic requests and strict response parsing."""

    def test_encodes_every_motion_command(self) -> None:
        cases = (
            CommandType.MOVE_FORWARD,
            CommandType.TURN_LEFT,
            CommandType.TURN_RIGHT,
            CommandType.STOP,
        )

        for sequence_id, command_type in enumerate(
            cases,
            start=1,
        ):
            with self.subTest(command_type=command_type):
                encoded = encode_motion_command(
                    sequence_id=sequence_id,
                    command_type=command_type,
                )

                self.assertEqual(
                    encoded,
                    (
                        f"ALR|1|CMD|{sequence_id}|"
                        f"{command_type.value}\n"
                    ).encode("ascii"),
                )

    def test_encodes_all_control_requests(self) -> None:
        self.assertEqual(
            encode_ping(sequence_id=10),
            b"ALR|1|PING|10\n",
        )
        self.assertEqual(
            encode_emergency_stop(sequence_id=11),
            b"ALR|1|ESTOP|11\n",
        )
        self.assertEqual(
            encode_rearm(sequence_id=12),
            b"ALR|1|REARM|12\n",
        )
        self.assertEqual(
            encode_status_request(sequence_id=13),
            b"ALR|1|STATUS|13\n",
        )

    def test_accepts_maximum_sequence_identifier(self) -> None:
        self.assertEqual(
            encode_ping(sequence_id=MAX_SEQUENCE_ID),
            (
                f"ALR|1|PING|{MAX_SEQUENCE_ID}\n"
            ).encode("ascii"),
        )

    def test_rejects_invalid_outbound_sequence_identifiers(self) -> None:
        invalid_identifiers = (
            True,
            0,
            -1,
            MAX_SEQUENCE_ID + 1,
            1.5,
            "1",
        )

        for invalid_identifier in invalid_identifiers:
            with self.subTest(
                invalid_identifier=invalid_identifier,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "sequence_id",
                ):
                    encode_ping(
                        sequence_id=invalid_identifier,  # type: ignore[arg-type]
                    )

    def test_rejects_invalid_motion_command_type(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "command_type",
        ):
            encode_motion_command(
                sequence_id=1,
                command_type=object(),  # type: ignore[arg-type]
            )

    def test_parses_ready_response(self) -> None:
        response = parse_arduino_response(
            b"ALR|1|READY|I-1.0\n"
        )

        self.assertEqual(
            response,
            ArduinoSerialResponse(
                response_type=ArduinoResponseType.READY,
                sequence_id=None,
                payload=("I-1.0",),
            ),
        )

    def test_parses_pong_response_with_crlf(self) -> None:
        response = parse_arduino_response(
            "ALR|1|PONG|42\r\n"
        )

        self.assertEqual(
            response.response_type,
            ArduinoResponseType.PONG,
        )
        self.assertEqual(response.sequence_id, 42)
        self.assertEqual(response.payload, ())

    def test_parses_acknowledgements_for_all_commands(self) -> None:
        for command_type in CommandType:
            with self.subTest(command_type=command_type):
                response = parse_arduino_response(
                    "ALR|1|ACK|7|"
                    f"{command_type.value}\n"
                )

                self.assertEqual(
                    response.response_type,
                    ArduinoResponseType.ACKNOWLEDGED,
                )
                self.assertEqual(response.sequence_id, 7)
                self.assertEqual(
                    response.payload,
                    (command_type.value,),
                )

    def test_parses_all_completion_codes(self) -> None:
        for completion_code in ArduinoCompletionCode:
            with self.subTest(
                completion_code=completion_code,
            ):
                response = parse_arduino_response(
                    "ALR|1|DONE|8|"
                    f"{completion_code.value}\n"
                )

                self.assertEqual(
                    response.response_type,
                    ArduinoResponseType.COMPLETED,
                )
                self.assertEqual(response.sequence_id, 8)
                self.assertEqual(
                    response.payload,
                    (completion_code.value,),
                )

    def test_parses_all_error_codes(self) -> None:
        for error_code in ArduinoErrorCode:
            with self.subTest(error_code=error_code):
                response = parse_arduino_response(
                    "ALR|1|ERROR|9|"
                    f"{error_code.value}\n"
                )

                self.assertEqual(
                    response.response_type,
                    ArduinoResponseType.ERROR,
                )
                self.assertEqual(response.sequence_id, 9)
                self.assertEqual(
                    response.payload,
                    (error_code.value,),
                )

    def test_parses_status_response(self) -> None:
        response = parse_arduino_response(
            "ALR|1|STATUS|10|EXECUTING|SAFE\n"
        )

        self.assertEqual(
            response.response_type,
            ArduinoResponseType.STATUS,
        )
        self.assertEqual(response.sequence_id, 10)
        self.assertEqual(
            response.payload,
            (
                ArduinoDeviceState.EXECUTING.value,
                ArduinoSafetyState.SAFE.value,
            ),
        )

    def test_rejects_non_ascii_response(self) -> None:
        with self.assertRaisesRegex(
            ArduinoSerialProtocolError,
            "ASCII",
        ):
            parse_arduino_response(
                b"ALR|1|READY|\xff\n"
            )

    def test_rejects_non_text_response(self) -> None:
        with self.assertRaisesRegex(
            ArduinoSerialProtocolError,
            "bytes or a string",
        ):
            parse_arduino_response(123)

    def test_rejects_embedded_multiple_lines(self) -> None:
        with self.assertRaisesRegex(
            ArduinoSerialProtocolError,
            "exactly one",
        ):
            parse_arduino_response(
                "ALR|1|PONG|1\nALR|1|PONG|2\n"
            )

    def test_rejects_invalid_headers_and_identifiers(self) -> None:
        cases = (
            (
                "",
                "exactly one non-empty line",
            ),
            (
                "OTHER|1|PONG|1",
                "protocol name",
            ),
            (
                "ALR|2|PONG|1",
                "protocol version",
            ),
            (
                "ALR|1|UNKNOWN|1",
                "type is unsupported",
            ),
            (
                "ALR|1|PONG",
                "missing its sequence",
            ),
            (
                "ALR|1|PONG|abc",
                "sequence identifier",
            ),
            (
                "ALR|1|PONG|0",
                "sequence identifier",
            ),
            (
                f"ALR|1|PONG|{MAX_SEQUENCE_ID + 1}",
                "sequence identifier",
            ),
        )

        for line, expected_message in cases:
            with self.subTest(line=line):
                with self.assertRaisesRegex(
                    ArduinoSerialProtocolError,
                    expected_message,
                ):
                    parse_arduino_response(line)

    def test_rejects_invalid_response_payloads(self) -> None:
        cases = (
            (
                "ALR|1|READY|I-1.0|EXTRA",
                "field count",
            ),
            (
                "ALR|1|READY|",
                "invalid token",
            ),
            (
                "ALR|1|PONG|1|EXTRA",
                "must not contain",
            ),
            (
                "ALR|1|ACK|1",
                "exactly one",
            ),
            (
                "ALR|1|ACK|1|FLY",
                "unsupported",
            ),
            (
                "ALR|1|ACK|1| MOVE_FORWARD",
                "invalid token",
            ),
            (
                "ALR|1|DONE|1|UNKNOWN",
                "unsupported",
            ),
            (
                "ALR|1|ERROR|1|UNKNOWN",
                "unsupported",
            ),
            (
                "ALR|1|STATUS|1|IDLE",
                "device and safety",
            ),
            (
                "ALR|1|STATUS|1|UNKNOWN|SAFE",
                "device state",
            ),
            (
                "ALR|1|STATUS|1|IDLE|UNKNOWN",
                "safety state",
            ),
        )

        for line, expected_message in cases:
            with self.subTest(line=line):
                with self.assertRaisesRegex(
                    ArduinoSerialProtocolError,
                    expected_message,
                ):
                    parse_arduino_response(line)


if __name__ == "__main__":
    unittest.main()