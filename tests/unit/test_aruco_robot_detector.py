"""Unit tests for rectified-grid ArUco robot localization."""

import unittest

import cv2
import numpy as np
from numpy.typing import NDArray

from ai_logistics_robot.adapters.hardware.aruco_robot_detector import (
    ArucoRobotDetectionError,
    ArucoRobotDetector,
)
from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position


def _calibration() -> GridCalibration:
    """Return one synthetic 7x7 calibration."""

    return GridCalibration(
        source_width=800,
        source_height=600,
        source_points=(
            (0.0, 0.0),
            (799.0, 0.0),
            (799.0, 599.0),
            (0.0, 599.0),
        ),
    )


def _generate_marker(
    *,
    marker_id: int = 0,
    rotation_code: int | None = None,
) -> NDArray[np.uint8]:
    """Generate one optionally rotated 4x4 ArUco marker."""

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    marker = np.asarray(
        cv2.aruco.generateImageMarker(
            dictionary,
            marker_id,
            120,
        ),
        dtype=np.uint8,
    )

    if rotation_code is not None:
        marker = np.asarray(
            cv2.rotate(marker, rotation_code),
            dtype=np.uint8,
        )

    return marker


def _frame_with_markers(
    *markers: tuple[
        NDArray[np.uint8],
        int,
        int,
    ],
) -> NDArray[np.uint8]:
    """Place markers at selected centers on a white rectified frame."""

    frame = np.full(
        (700, 700),
        255,
        dtype=np.uint8,
    )

    for marker, center_x, center_y in markers:
        half_size = marker.shape[0] // 2
        frame[
            center_y - half_size:center_y + half_size,
            center_x - half_size:center_x + half_size,
        ] = marker

    return frame


class ArucoRobotDetectorTests(unittest.TestCase):
    """Verify marker identity, cell mapping, and cardinal heading."""

    def setUp(self) -> None:
        """Create one detector for marker zero."""

        self.calibration = _calibration()
        self.detector = ArucoRobotDetector(
            calibration=self.calibration,
            marker_id=0,
        )

    def test_detects_all_cardinal_headings(self) -> None:
        """Translate canonical marker rotation into domain headings."""

        cases = (
            (None, Heading.NORTH),
            (cv2.ROTATE_90_CLOCKWISE, Heading.EAST),
            (cv2.ROTATE_180, Heading.SOUTH),
            (cv2.ROTATE_90_COUNTERCLOCKWISE, Heading.WEST),
        )

        for rotation_code, expected_heading in cases:
            with self.subTest(
                expected_heading=expected_heading,
            ):
                frame = _frame_with_markers(
                    (
                        _generate_marker(
                            rotation_code=rotation_code,
                        ),
                        250,
                        450,
                    )
                )

                detection = self.detector.detect(frame)

                self.assertEqual(
                    detection.marker_id,
                    0,
                )
                self.assertEqual(
                    detection.pose.position,
                    Position(x=2, y=2),
                )
                self.assertEqual(
                    detection.pose.heading,
                    expected_heading,
                )
                self.assertAlmostEqual(
                    detection.center_x,
                    249.5,
                    delta=1.0,
                )
                self.assertAlmostEqual(
                    detection.center_y,
                    449.5,
                    delta=1.0,
                )

    def test_inverts_image_rows_into_domain_y_coordinates(
        self,
    ) -> None:
        """Map image top to high domain y and image bottom to low y."""

        cases = (
            ((75, 75), Position(x=0, y=6)),
            ((625, 625), Position(x=6, y=0)),
        )

        for center, expected_position in cases:
            with self.subTest(
                expected_position=expected_position,
            ):
                frame = _frame_with_markers(
                    (
                        _generate_marker(),
                        center[0],
                        center[1],
                    )
                )

                detection = self.detector.detect(frame)

                self.assertEqual(
                    detection.pose.position,
                    expected_position,
                )

    def test_rejects_frame_without_marker(self) -> None:
        """Refuse to fabricate a pose when no marker is visible."""

        empty_frame = np.full(
            (700, 700),
            255,
            dtype=np.uint8,
        )

        with self.assertRaisesRegex(
            ArucoRobotDetectionError,
            "was not detected",
        ):
            self.detector.detect(empty_frame)

    def test_rejects_unconfigured_marker(self) -> None:
        """Ignore valid markers that do not identify this robot."""

        frame = _frame_with_markers(
            (
                _generate_marker(marker_id=1),
                350,
                350,
            )
        )

        with self.assertRaisesRegex(
            ArucoRobotDetectionError,
            "was not detected",
        ):
            self.detector.detect(frame)

    def test_rejects_duplicate_robot_marker(self) -> None:
        """Reject an ambiguous frame containing the robot ID twice."""

        marker = _generate_marker()
        frame = _frame_with_markers(
            (marker, 150, 150),
            (marker, 550, 550),
        )

        with self.assertRaisesRegex(
            ArucoRobotDetectionError,
            "more than once",
        ):
            self.detector.detect(frame)

    def test_validates_frame_geometry(self) -> None:
        """Require the exact rectified calibration dimensions."""

        invalid_frame = np.full(
            (699, 700),
            255,
            dtype=np.uint8,
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "dimensions",
        ):
            self.detector.detect(invalid_frame)

    def test_validates_marker_identifier(self) -> None:
        """Reject identifiers outside the selected dictionary."""

        for invalid_marker_id in (
            True,
            -1,
            50,
        ):
            with self.subTest(
                invalid_marker_id=invalid_marker_id,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "between 0 and 49",
                ):
                    ArucoRobotDetector(
                        calibration=self.calibration,
                        marker_id=invalid_marker_id,
                    )


if __name__ == "__main__":
    unittest.main()