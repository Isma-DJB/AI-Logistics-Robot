"""Capture and rectify frames from one fixed overhead ESP32-CAM."""

import cv2
import numpy as np

from ai_logistics_robot.adapters.hardware.esp32_camera import (
    Esp32CameraClient,
)
from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
    ImageArray,
    rectify_frame,
)
from ai_logistics_robot.domain.errors import DomainValidationError


class CalibratedGridCameraError(RuntimeError):
    """Report one JPEG decoding or grid-rectification failure."""


class CalibratedGridCamera:
    """Compose one ESP32-CAM client with one grid calibration."""

    __slots__ = (
        "_calibration",
        "_camera",
    )

    def __init__(
        self,
        *,
        camera: Esp32CameraClient,
        calibration: GridCalibration,
    ) -> None:
        """Validate and retain capture and calibration dependencies."""

        if not isinstance(camera, Esp32CameraClient):
            raise DomainValidationError(
                "camera must be an Esp32CameraClient."
            )

        if not isinstance(calibration, GridCalibration):
            raise DomainValidationError(
                "calibration must be a GridCalibration."
            )

        self._camera = camera
        self._calibration = calibration

    @property
    def camera(self) -> Esp32CameraClient:
        """Return the configured raw-camera client."""

        return self._camera

    @property
    def calibration(self) -> GridCalibration:
        """Return the fixed perspective calibration."""

        return self._calibration

    def capture_rectified(self) -> ImageArray:
        """Capture, decode, validate, and rectify one camera frame."""

        jpeg_payload = self._camera.capture_jpeg()
        encoded_frame = np.frombuffer(
            jpeg_payload,
            dtype=np.uint8,
        )
        decoded_frame = cv2.imdecode(
            encoded_frame,
            cv2.IMREAD_COLOR,
        )

        if decoded_frame is None:
            raise CalibratedGridCameraError(
                "ESP32-CAM JPEG frame could not be decoded."
            )

        frame = np.asarray(
            decoded_frame,
            dtype=np.uint8,
        )

        try:
            return rectify_frame(
                frame,
                self._calibration,
            )
        except ValueError as error:
            raise CalibratedGridCameraError(
                "ESP32-CAM frame does not match the grid calibration."
            ) from error