"""Unit tests for the ESP32-CAM HTTP capture adapter."""

import unittest

from ai_logistics_robot.adapters.hardware.esp32_camera import (
    Esp32CameraClient,
    Esp32CameraError,
)
from ai_logistics_robot.domain.errors import DomainValidationError


class Esp32CameraClientTests(unittest.TestCase):
    """Verify validated and isolated ESP32-CAM captures."""

    def test_connection_settings_are_normalized(self) -> None:
        client = Esp32CameraClient(
            base_url="  http://192.168.1.109/  ",
            timeout_seconds=3,
        )

        self.assertEqual(
            client.base_url,
            "http://192.168.1.109",
        )
        self.assertEqual(
            client.capture_url,
            "http://192.168.1.109/capture",
        )
        self.assertEqual(
            client.timeout_seconds,
            3.0,
        )

    def test_invalid_camera_origins_are_rejected(self) -> None:
        invalid_origins = (
            "",
            "camera.local",
            "ftp://camera.local",
            "http://camera.local/capture",
            "http://camera.local?mode=still",
            "http://camera.local#capture",
        )

        for invalid_origin in invalid_origins:
            with self.subTest(
                invalid_origin=invalid_origin,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "base_url",
                ):
                    Esp32CameraClient(
                        base_url=invalid_origin,
                    )

    def test_invalid_timeouts_are_rejected(self) -> None:
        invalid_timeouts = (
            0,
            -1,
            float("inf"),
            float("nan"),
            True,
        )

        for invalid_timeout in invalid_timeouts:
            with self.subTest(
                invalid_timeout=invalid_timeout,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "timeout_seconds",
                ):
                    Esp32CameraClient(
                        base_url="http://camera.local",
                        timeout_seconds=invalid_timeout,
                    )

    def test_complete_jpeg_is_returned(self) -> None:
        calls: list[tuple[str, float]] = []
        expected_frame = b"\xff\xd8camera-frame\xff\xd9"

        def fetcher(
            url: str,
            timeout_seconds: float,
        ) -> bytes:
            calls.append(
                (
                    url,
                    timeout_seconds,
                )
            )
            return expected_frame

        client = Esp32CameraClient(
            base_url="http://camera.local",
            timeout_seconds=2.5,
            fetcher=fetcher,
        )

        frame = client.capture_jpeg()

        self.assertEqual(
            frame,
            expected_frame,
        )
        self.assertEqual(
            calls,
            [
                (
                    "http://camera.local/capture",
                    2.5,
                )
            ],
        )

    def test_incomplete_or_non_jpeg_responses_are_rejected(
        self,
    ) -> None:
        invalid_frames = (
            b"",
            b"not-a-jpeg",
            b"\xff\xd8missing-end",
            b"missing-start\xff\xd9",
        )

        for invalid_frame in invalid_frames:
            with self.subTest(
                invalid_frame=invalid_frame,
            ):

                def fetcher(
                    url: str,
                    timeout_seconds: float,
                    frame: bytes = invalid_frame,
                ) -> bytes:
                    del url
                    del timeout_seconds
                    return frame

                client = Esp32CameraClient(
                    base_url="http://camera.local",
                    fetcher=fetcher,
                )

                with self.assertRaisesRegex(
                    Esp32CameraError,
                    "complete JPEG",
                ):
                    client.capture_jpeg()

    def test_connection_failure_is_translated(self) -> None:
        def failing_fetcher(
            url: str,
            timeout_seconds: float,
        ) -> bytes:
            del url
            del timeout_seconds
            raise OSError("camera unavailable")

        client = Esp32CameraClient(
            base_url="http://camera.local",
            fetcher=failing_fetcher,
        )

        with self.assertRaisesRegex(
            Esp32CameraError,
            "capture request failed",
        ):
            client.capture_jpeg()


if __name__ == "__main__":
    unittest.main()