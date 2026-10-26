"""Unit tests for normalized physical-grid perception."""

import unittest
from datetime import UTC, datetime

import cv2
import numpy as np

from ai_logistics_robot.adapters.hardware.aruco_robot_detector import (
    ArucoRobotDetectionError,
    ArucoRobotDetector,
)
from ai_logistics_robot.adapters.hardware.calibrated_grid_camera import (
    CalibratedGridCamera,
)
from ai_logistics_robot.adapters.hardware.esp32_camera import (
    Esp32CameraClient,
)
from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
)
from ai_logistics_robot.adapters.hardware.physical_grid_perception import (
    PhysicalGridPerception,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.perception import Observation
from ai_logistics_robot.ports.perception_port import PerceptionPort


class _FixedClock:
    """Return one deterministic timezone-aware instant."""

    __slots__ = ("_current_time",)

    def __init__(
        self,
        current_time: datetime,
    ) -> None:
        self._current_time = current_time

    def now(self) -> datetime:
        return self._current_time

    def monotonic(self) -> float:
        return 0.0

    def wait_until(
        self,
        deadline: float,
    ) -> None:
        del deadline


def _calibration(
    *,
    left_x: float = 0.0,
) -> GridCalibration:
    """Return one identity-like 7x7 calibration."""

    return GridCalibration(
        source_width=700,
        source_height=700,
        source_points=(
            (left_x, 0.0),
            (699.0, 0.0),
            (699.0, 699.0),
            (left_x, 699.0),
        ),
    )


def _jpeg(
    *,
    include_marker: bool = True,
) -> bytes:
    """Encode one synthetic calibrated-grid frame."""

    frame = np.full(
        (700, 700, 3),
        255,
        dtype=np.uint8,
    )

    if include_marker:
        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        marker = cv2.aruco.generateImageMarker(
            dictionary,
            0,
            120,
        )
        marker_bgr = cv2.cvtColor(
            marker,
            cv2.COLOR_GRAY2BGR,
        )
        frame[290:410, 190:310] = marker_bgr

    encoded_successfully, encoded_frame = cv2.imencode(
        ".jpg",
        frame,
        (cv2.IMWRITE_JPEG_QUALITY, 100),
    )

    if not encoded_successfully:
        raise RuntimeError("Synthetic JPEG encoding failed.")

    return encoded_frame.tobytes()


def _camera(
    *,
    calibration: GridCalibration,
    payload: bytes,
) -> CalibratedGridCamera:
    """Return one deterministic calibrated camera."""

    def fetcher(
        url: str,
        timeout_seconds: float,
    ) -> bytes:
        del url
        del timeout_seconds
        return payload

    return CalibratedGridCamera(
        camera=Esp32CameraClient(
            base_url="http://camera.test",
            fetcher=fetcher,
        ),
        calibration=calibration,
    )


class PhysicalGridPerceptionTests(unittest.TestCase):
    """Verify safe conversion from overhead frames to snapshots."""

    def setUp(self) -> None:
        """Create shared deterministic physical dependencies."""

        self.calibration = _calibration()
        self.clock = _FixedClock(
            datetime(
                2026,
                10,
                3,
                18,
                0,
                tzinfo=UTC,
            )
        )

    def _perception(
        self,
        *,
        payload: bytes | None = None,
    ) -> PhysicalGridPerception:
        """Return one assembled physical perception adapter."""

        return PhysicalGridPerception(
            robot_id="robot-physical-1",
            camera=_camera(
                calibration=self.calibration,
                payload=payload or _jpeg(),
            ),
            detector=ArucoRobotDetector(
                calibration=self.calibration,
                marker_id=0,
            ),
            clock=self.clock,
        )

    def test_observe_returns_normalized_snapshot(self) -> None:
        perception = self._perception()

        snapshot = perception.observe()

        self.assertIsInstance(perception, PerceptionPort)
        self.assertEqual(
            snapshot.robot_id,
            "robot-physical-1",
        )
        self.assertEqual(
            snapshot.captured_at,
            self.clock.now(),
        )
        self.assertEqual(
            snapshot.robot_pose,
            RobotPose(
                position=Position(x=2, y=3),
                heading=Heading.NORTH,
            ),
        )
        self.assertEqual(snapshot.observations, ())
        self.assertFalse(snapshot.target_active)
        self.assertFalse(snapshot.hazard_detected)

    def test_confirmed_external_inputs_are_preserved(self) -> None:
        perception = self._perception()
        observation = Observation(
            kind="obstacle",
            position=Position(x=4, y=1),
            confidence=0.9,
        )

        perception.set_target_active(True)
        perception.set_hazard_detected(True)
        perception.set_observations((observation,))

        snapshot = perception.observe()

        self.assertTrue(snapshot.target_active)
        self.assertTrue(snapshot.hazard_detected)
        self.assertEqual(
            snapshot.observations,
            (observation,),
        )

    def test_missing_marker_is_not_converted_to_a_pose(self) -> None:
        perception = self._perception(
            payload=_jpeg(include_marker=False),
        )

        with self.assertRaises(ArucoRobotDetectionError):
            perception.observe()

    def test_invalid_robot_identifier_is_rejected(self) -> None:
        camera = _camera(
            calibration=self.calibration,
            payload=_jpeg(),
        )
        detector = ArucoRobotDetector(
            calibration=self.calibration,
        )

        for invalid_identifier in ("", " ", object()):
            with self.subTest(
                invalid_identifier=invalid_identifier,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "robot_id",
                ):
                    PhysicalGridPerception(
                        robot_id=invalid_identifier,
                        camera=camera,
                        detector=detector,
                        clock=self.clock,
                    )

    def test_invalid_dependencies_are_rejected(self) -> None:
        camera = _camera(
            calibration=self.calibration,
            payload=_jpeg(),
        )
        detector = ArucoRobotDetector(
            calibration=self.calibration,
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "camera must be",
        ):
            PhysicalGridPerception(
                robot_id="robot-1",
                camera=object(),
                detector=detector,
                clock=self.clock,
            )

        with self.assertRaisesRegex(
            DomainValidationError,
            "detector must be",
        ):
            PhysicalGridPerception(
                robot_id="robot-1",
                camera=camera,
                detector=object(),
                clock=self.clock,
            )

        with self.assertRaisesRegex(
            DomainValidationError,
            "clock must satisfy",
        ):
            PhysicalGridPerception(
                robot_id="robot-1",
                camera=camera,
                detector=detector,
                clock=object(),
            )

    def test_different_calibrations_are_rejected(self) -> None:
        camera = _camera(
            calibration=self.calibration,
            payload=_jpeg(),
        )
        detector = ArucoRobotDetector(
            calibration=_calibration(left_x=1.0),
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "same grid calibration",
        ):
            PhysicalGridPerception(
                robot_id="robot-1",
                camera=camera,
                detector=detector,
                clock=self.clock,
            )

    def test_invalid_external_inputs_are_rejected(self) -> None:
        perception = self._perception()

        with self.assertRaisesRegex(
            DomainValidationError,
            "active must be",
        ):
            perception.set_target_active(1)

        with self.assertRaisesRegex(
            DomainValidationError,
            "detected must be",
        ):
            perception.set_hazard_detected("yes")

        with self.assertRaisesRegex(
            DomainValidationError,
            "immutable tuple",
        ):
            perception.set_observations([])

        with self.assertRaisesRegex(
            DomainValidationError,
            "every observation",
        ):
            perception.set_observations((object(),))