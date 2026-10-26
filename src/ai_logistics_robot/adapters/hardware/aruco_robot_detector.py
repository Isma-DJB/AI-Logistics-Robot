"""Detect one robot ArUco marker in a rectified overhead-grid frame."""

from dataclasses import dataclass
from math import hypot, isfinite
from typing import TypeAlias

import cv2
import numpy as np
from numpy.typing import NDArray

from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
    rectified_pixel_to_cell,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose

ImageArray: TypeAlias = NDArray[np.uint8]
CornerArray: TypeAlias = NDArray[np.float64]


class ArucoRobotDetectionError(RuntimeError):
    """Report one missing, ambiguous, or invalid robot marker."""


@dataclass(frozen=True, slots=True)
class ArucoRobotDetection:
    """Describe one robot localization obtained from an ArUco marker."""

    marker_id: int
    center_x: float
    center_y: float
    pose: RobotPose


def _heading_from_marker_corners(
    marker_corners: CornerArray,
) -> Heading:
    """Convert the marker's canonical top direction into a heading."""

    center = marker_corners.mean(axis=0)
    top_midpoint = (
        marker_corners[0] + marker_corners[1]
    ) / 2.0

    delta_x = float(top_midpoint[0] - center[0])
    delta_y = float(top_midpoint[1] - center[1])

    if (
        not isfinite(delta_x)
        or not isfinite(delta_y)
        or hypot(delta_x, delta_y) < 1.0
    ):
        raise ArucoRobotDetectionError(
            "ArUco marker orientation could not be determined."
        )

    if abs(delta_x) > abs(delta_y):
        if delta_x > 0.0:
            return Heading.EAST

        return Heading.WEST

    if delta_y > 0.0:
        return Heading.SOUTH

    return Heading.NORTH


class ArucoRobotDetector:
    """Locate one configured robot marker on a rectified grid image."""

    __slots__ = (
        "_calibration",
        "_detector",
        "_marker_id",
    )

    def __init__(
        self,
        *,
        calibration: GridCalibration,
        marker_id: int = 0,
    ) -> None:
        """Validate settings and initialize the OpenCV detector."""

        if not isinstance(calibration, GridCalibration):
            raise DomainValidationError(
                "calibration must be a GridCalibration instance."
            )

        if (
            isinstance(marker_id, bool)
            or not isinstance(marker_id, int)
            or not 0 <= marker_id < 50
        ):
            raise DomainValidationError(
                "marker_id must be an integer between 0 and 49."
            )

        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self._calibration = calibration
        self._marker_id = marker_id
        self._detector = cv2.aruco.ArucoDetector(dictionary)

    @property
    def calibration(self) -> GridCalibration:
        """Return the grid calibration used for localization."""

        return self._calibration

    @property
    def marker_id(self) -> int:
        """Return the configured robot marker identifier."""

        return self._marker_id

    def detect(
        self,
        rectified_frame: ImageArray,
    ) -> ArucoRobotDetection:
        """Detect the configured marker and return its logical pose."""

        if not isinstance(rectified_frame, np.ndarray):
            raise DomainValidationError(
                "rectified_frame must be a NumPy array."
            )

        if rectified_frame.dtype != np.uint8:
            raise DomainValidationError(
                "rectified_frame must contain uint8 pixels."
            )

        if rectified_frame.ndim not in {2, 3}:
            raise DomainValidationError(
                "rectified_frame must be grayscale or BGR."
            )

        frame_height, frame_width = rectified_frame.shape[:2]

        if (
            frame_width != self._calibration.output_width
            or frame_height != self._calibration.output_height
        ):
            raise DomainValidationError(
                "rectified_frame dimensions must match the calibration."
            )

        grayscale: ImageArray

        if rectified_frame.ndim == 2:
            grayscale = rectified_frame
        elif rectified_frame.shape[2] == 3:
            grayscale = np.asarray(
                cv2.cvtColor(
                    rectified_frame,
                    cv2.COLOR_BGR2GRAY,
                ),
                dtype=np.uint8,
            )
        else:
            raise DomainValidationError(
                "rectified_frame must have exactly three BGR channels."
            )

        corners, identifiers, _ = self._detector.detectMarkers(
            grayscale
        )

        if identifiers is None:
            raise ArucoRobotDetectionError(
                "Configured ArUco robot marker was not detected."
            )

        identifier_values = np.asarray(
            identifiers,
            dtype=np.int32,
        ).reshape(-1)

        matching_indices = [
            index
            for index, identifier in enumerate(identifier_values)
            if int(identifier) == self._marker_id
        ]

        if not matching_indices:
            raise ArucoRobotDetectionError(
                "Configured ArUco robot marker was not detected."
            )

        if len(matching_indices) > 1:
            raise ArucoRobotDetectionError(
                "Configured ArUco robot marker was detected more than once."
            )

        marker_corners = np.asarray(
            corners[matching_indices[0]],
            dtype=np.float64,
        ).reshape(4, 2)

        center = marker_corners.mean(axis=0)
        center_x = float(center[0])
        center_y = float(center[1])

        if not isfinite(center_x) or not isfinite(center_y):
            raise ArucoRobotDetectionError(
                "ArUco marker center is not finite."
            )

        column, row = rectified_pixel_to_cell(
            int(center_x),
            int(center_y),
            self._calibration,
        )

        pose = RobotPose(
            position=Position(
                x=column,
                y=self._calibration.rows - 1 - row,
            ),
            heading=_heading_from_marker_corners(
                marker_corners
            ),
        )

        return ArucoRobotDetection(
            marker_id=self._marker_id,
            center_x=center_x,
            center_y=center_y,
            pose=pose,
        )