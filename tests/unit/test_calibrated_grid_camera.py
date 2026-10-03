"""Unit tests for calibrated ESP32-CAM grid capture."""

import unittest

import cv2
import numpy as np

from ai_logistics_robot.adapters.hardware.calibrated_grid_camera import (
    CalibratedGridCamera,
    CalibratedGridCameraError,
)
from ai_logistics_robot.adapters.hardware.esp32_camera import (
    Esp32CameraClient,
    Esp32CameraError,
)
from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
)
from ai_logistics_robot.domain.errors import DomainValidationError


class CalibratedGridCameraTests(unittest.TestCase):
    """Verify composed JPEG capture and perspective correction."""

    def test_constructor_exposes_valid_dependencies(self) -> None:
        camera = self._camera(self._jpeg())
        calibration = self._calibration()

        calibrated_camera = CalibratedGridCamera(
            camera=camera,
            calibration=calibration,
        )

        self.assertIs(calibrated_camera.camera, camera)
        self.assertIs(
            calibrated_camera.calibration,
            calibration,
        )

    def test_invalid_camera_dependency_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "camera must be",
        ):
            CalibratedGridCamera(
                camera=object(),
                calibration=self._calibration(),
            )

    def test_invalid_calibration_dependency_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "calibration must be",
        ):
            CalibratedGridCamera(
                camera=self._camera(self._jpeg()),
                calibration=object(),
            )

    def test_valid_jpeg_is_decoded_and_rectified(self) -> None:
        calibrated_camera = CalibratedGridCamera(
            camera=self._camera(self._jpeg()),
            calibration=self._calibration(),
        )

        rectified = calibrated_camera.capture_rectified()

        self.assertEqual(rectified.shape, (8, 8, 3))
        self.assertEqual(rectified.dtype, np.dtype(np.uint8))

    def test_invalid_jpeg_encoding_is_rejected(self) -> None:
        malformed_jpeg = (
            b"\xff\xd8not-a-real-jpeg\xff\xd9"
        )
        calibrated_camera = CalibratedGridCamera(
            camera=self._camera(malformed_jpeg),
            calibration=self._calibration(),
        )

        with self.assertRaisesRegex(
            CalibratedGridCameraError,
            "could not be decoded",
        ):
            calibrated_camera.capture_rectified()

    def test_frame_resolution_must_match_calibration(self) -> None:
        mismatched_jpeg = self._jpeg(
            width=10,
            height=8,
        )
        calibrated_camera = CalibratedGridCamera(
            camera=self._camera(mismatched_jpeg),
            calibration=self._calibration(),
        )

        with self.assertRaisesRegex(
            CalibratedGridCameraError,
            "does not match",
        ):
            calibrated_camera.capture_rectified()

    def test_camera_capture_error_is_preserved(self) -> None:
        def failing_fetcher(
            url: str,
            timeout_seconds: float,
        ) -> bytes:
            del url
            del timeout_seconds
            raise OSError("synthetic network failure")

        camera = Esp32CameraClient(
            base_url="http://camera.test",
            fetcher=failing_fetcher,
        )
        calibrated_camera = CalibratedGridCamera(
            camera=camera,
            calibration=self._calibration(),
        )

        with self.assertRaises(Esp32CameraError):
            calibrated_camera.capture_rectified()

    @staticmethod
    def _camera(payload: bytes) -> Esp32CameraClient:
        """Return one deterministic in-memory camera client."""

        def fetcher(
            url: str,
            timeout_seconds: float,
        ) -> bytes:
            del url
            del timeout_seconds
            return payload

        return Esp32CameraClient(
            base_url="http://camera.test",
            fetcher=fetcher,
        )

    @staticmethod
    def _calibration() -> GridCalibration:
        """Return one identity-style 8 by 8 calibration."""

        return GridCalibration(
            source_width=8,
            source_height=8,
            source_points=(
                (0.0, 0.0),
                (7.0, 0.0),
                (7.0, 7.0),
                (0.0, 7.0),
            ),
            columns=2,
            rows=2,
            pixels_per_cell=4,
        )

    @staticmethod
    def _jpeg(
        *,
        width: int = 8,
        height: int = 8,
    ) -> bytes:
        """Encode one deterministic synthetic colour frame."""

        frame = np.zeros(
            (height, width, 3),
            dtype=np.uint8,
        )
        frame[:, :] = (20, 80, 160)

        encoded_successfully, encoded_frame = cv2.imencode(
            ".jpg",
            frame,
        )

        if not encoded_successfully:
            raise AssertionError(
                "Synthetic JPEG encoding unexpectedly failed."
            )

        return encoded_frame.tobytes()


if __name__ == "__main__":
    unittest.main()