"""Detect and annotate one robot pose on a calibrated grid."""

import sys
from argparse import ArgumentParser, Namespace
from collections.abc import Sequence
from pathlib import Path

import cv2
import numpy as np

from ai_logistics_robot.adapters.hardware.aruco_robot_detector import (
    ArucoRobotDetection,
    ArucoRobotDetectionError,
    ArucoRobotDetector,
)
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
    GridCalibrationFileError,
    ImageArray,
    load_grid_calibration,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError

DEFAULT_CALIBRATION = Path(
    "configs/hardware/grid_calibration_7x7.yaml"
)


class RobotPoseDiagnosticError(RuntimeError):
    """Report one unreadable rectified diagnostic image."""


def build_parser() -> ArgumentParser:
    """Build the robot-pose diagnostic command line."""

    parser = ArgumentParser(
        description=(
            "Detect one ArUco robot marker on a calibrated "
            "overhead-grid image."
        )
    )

    source_group = parser.add_mutually_exclusive_group(
        required=True
    )
    source_group.add_argument(
        "--url",
        help="ESP32-CAM HTTP origin.",
    )
    source_group.add_argument(
        "--input",
        type=Path,
        help="Existing rectified grid image.",
    )

    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help=(
            "Validated YAML grid calibration. "
            f"Default: {DEFAULT_CALIBRATION}."
        ),
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Destination path for the annotated image.",
    )
    parser.add_argument(
        "--marker-id",
        default=0,
        type=int,
        help="Expected DICT_4X4_50 robot marker ID. Default: 0.",
    )
    parser.add_argument(
        "--timeout",
        default=5.0,
        type=float,
        help="Positive camera timeout in seconds. Default: 5.",
    )
    return parser


def _load_source(
    arguments: Namespace,
    calibration: GridCalibration,
) -> tuple[ImageArray, str]:
    """Load a rectified image or capture and rectify a live frame."""

    if arguments.url is not None:
        camera = Esp32CameraClient(
            base_url=arguments.url,
            timeout_seconds=arguments.timeout,
        )
        calibrated_camera = CalibratedGridCamera(
            camera=camera,
            calibration=calibration,
        )
        return (
            calibrated_camera.capture_rectified(),
            camera.capture_url,
        )

    input_path: Path = arguments.input
    loaded_frame = cv2.imread(
        str(input_path),
        cv2.IMREAD_UNCHANGED,
    )

    if loaded_frame is None:
        raise RobotPoseDiagnosticError(
            "Rectified input image could not be read."
        )

    return (
        np.asarray(
            loaded_frame,
            dtype=np.uint8,
        ),
        str(input_path.resolve()),
    )


def _annotate_detection(
    frame: ImageArray,
    calibration: GridCalibration,
    detection: ArucoRobotDetection,
) -> ImageArray:
    """Draw the detected cell, center, heading, and identity."""

    if frame.ndim == 2:
        annotated_frame = np.asarray(
            cv2.cvtColor(
                frame,
                cv2.COLOR_GRAY2BGR,
            ),
            dtype=np.uint8,
        )
    else:
        annotated_frame = frame.copy()

    position = detection.pose.position
    image_row = (
        calibration.rows - 1 - position.y
    )

    cell_left = (
        position.x * calibration.pixels_per_cell
    )
    cell_top = (
        image_row * calibration.pixels_per_cell
    )
    cell_right = (
        cell_left + calibration.pixels_per_cell - 1
    )
    cell_bottom = (
        cell_top + calibration.pixels_per_cell - 1
    )

    cv2.rectangle(
        annotated_frame,
        (cell_left, cell_top),
        (cell_right, cell_bottom),
        (0, 255, 0),
        3,
    )

    center = (
        int(round(detection.center_x)),
        int(round(detection.center_y)),
    )
    cv2.circle(
        annotated_frame,
        center,
        7,
        (0, 0, 255),
        -1,
    )

    direction_by_heading = {
        Heading.NORTH: (0, -1),
        Heading.EAST: (1, 0),
        Heading.SOUTH: (0, 1),
        Heading.WEST: (-1, 0),
    }
    direction_x, direction_y = direction_by_heading[
        detection.pose.heading
    ]
    arrow_length = max(
        30,
        calibration.pixels_per_cell // 3,
    )
    arrow_end = (
        center[0] + direction_x * arrow_length,
        center[1] + direction_y * arrow_length,
    )

    cv2.arrowedLine(
        annotated_frame,
        center,
        arrow_end,
        (255, 0, 0),
        4,
        tipLength=0.3,
    )

    label = (
        f"ID={detection.marker_id} "
        f"cell=({position.x},{position.y}) "
        f"heading={detection.pose.heading.value}"
    )

    cv2.putText(
        annotated_frame,
        label,
        (15, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        5,
        cv2.LINE_AA,
    )
    cv2.putText(
        annotated_frame,
        label,
        (15, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )

    return annotated_frame


def main(
    argv: Sequence[str] | None = None,
) -> int:
    """Acquire, localize, annotate, and save one robot pose."""

    arguments = build_parser().parse_args(argv)

    try:
        calibration = load_grid_calibration(
            arguments.calibration
        )
        rectified_frame, source = _load_source(
            arguments,
            calibration,
        )
        detector = ArucoRobotDetector(
            calibration=calibration,
            marker_id=arguments.marker_id,
        )
        detection = detector.detect(rectified_frame)
        annotated_frame = _annotate_detection(
            rectified_frame,
            calibration,
            detection,
        )
    except (
        DomainValidationError,
        Esp32CameraError,
        GridCalibrationFileError,
        CalibratedGridCameraError,
        ArucoRobotDetectionError,
        RobotPoseDiagnosticError,
        cv2.error,
    ) as error:
        print(
            f"Robot-pose diagnostic failed: {error}",
            file=sys.stderr,
        )
        return 2

    output_path: Path = arguments.output

    try:
        output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )
        image_written = cv2.imwrite(
            str(output_path),
            annotated_frame,
        )
    except (OSError, cv2.error) as error:
        print(
            f"Annotated robot-pose image could not be saved: {error}",
            file=sys.stderr,
        )
        return 2

    if not image_written:
        print(
            "Annotated robot-pose image could not be saved.",
            file=sys.stderr,
        )
        return 2

    position = detection.pose.position

    print("ArUco robot localization succeeded.")
    print(f"Source: {source}")
    print(f"Robot marker: {detection.marker_id}")
    print(
        f"Grid position: x={position.x}, y={position.y}"
    )
    print(
        f"Heading: {detection.pose.heading.value}"
    )
    print(
        "Center: "
        f"x={detection.center_x:.1f}, "
        f"y={detection.center_y:.1f}"
    )
    print(f"Annotated image: {output_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())