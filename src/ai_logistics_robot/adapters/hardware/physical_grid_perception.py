"""Produce normalized perception snapshots from overhead physical vision."""

from ai_logistics_robot.adapters.hardware.aruco_robot_detector import (
    ArucoRobotDetector,
)
from ai_logistics_robot.adapters.hardware.calibrated_grid_camera import (
    CalibratedGridCamera,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.perception import (
    Observation,
    PerceptionSnapshot,
)
from ai_logistics_robot.ports.clock_port import ClockPort


class PhysicalGridPerception:
    """Bridge calibrated overhead vision to the public perception contract."""

    __slots__ = (
        "_camera",
        "_clock",
        "_detector",
        "_hazard_detected",
        "_observations",
        "_robot_id",
        "_target_active",
    )

    def __init__(
        self,
        *,
        robot_id: str,
        camera: CalibratedGridCamera,
        detector: ArucoRobotDetector,
        clock: ClockPort,
    ) -> None:
        """Validate dependencies and initialize safe external inputs."""

        if not isinstance(robot_id, str) or not robot_id.strip():
            raise DomainValidationError(
                "robot_id must be a non-empty string."
            )

        if not isinstance(camera, CalibratedGridCamera):
            raise DomainValidationError(
                "camera must be a CalibratedGridCamera."
            )

        if not isinstance(detector, ArucoRobotDetector):
            raise DomainValidationError(
                "detector must be an ArucoRobotDetector."
            )

        if camera.calibration != detector.calibration:
            raise DomainValidationError(
                "camera and detector must use the same grid calibration."
            )

        if not isinstance(clock, ClockPort):
            raise DomainValidationError(
                "clock must satisfy ClockPort."
            )

        self._robot_id = robot_id
        self._camera = camera
        self._detector = detector
        self._clock = clock
        self._observations: tuple[Observation, ...] = ()
        self._target_active = False
        self._hazard_detected = False

    def set_target_active(
        self,
        active: bool,
    ) -> None:
        """Set the latest confirmed physical target state."""

        if not isinstance(active, bool):
            raise DomainValidationError(
                "active must be a boolean."
            )

        self._target_active = active

    def set_hazard_detected(
        self,
        detected: bool,
    ) -> None:
        """Set the latest confirmed physical hazard state."""

        if not isinstance(detected, bool):
            raise DomainValidationError(
                "detected must be a boolean."
            )

        self._hazard_detected = detected

    def set_observations(
        self,
        observations: tuple[Observation, ...],
    ) -> None:
        """Replace the latest immutable physical observation set."""

        if not isinstance(observations, tuple):
            raise DomainValidationError(
                "observations must be an immutable tuple."
            )

        if not all(
            isinstance(observation, Observation)
            for observation in observations
        ):
            raise DomainValidationError(
                "every observation must be an Observation instance."
            )

        self._observations = observations

    def observe(self) -> PerceptionSnapshot:
        """Capture and return one normalized physical snapshot."""

        rectified_frame = self._camera.capture_rectified()
        captured_at = self._clock.now()
        detection = self._detector.detect(rectified_frame)

        return PerceptionSnapshot(
            robot_id=self._robot_id,
            captured_at=captured_at,
            robot_pose=detection.pose,
            observations=self._observations,
            target_active=self._target_active,
            hazard_detected=self._hazard_detected,
        )