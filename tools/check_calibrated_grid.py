"""Capture and rectify one live ESP32-CAM grid frame."""

import sys
from argparse import ArgumentParser
from collections.abc import Sequence
from pathlib import Path

import cv2

from ai_logistics_robot.adapters.hardware.calibrated_grid_camera import (
    CalibratedGridCamera,
    CalibratedGridCameraError,
)
from ai_logistics_robot.adapters.hardware.esp32_camera import (
    Esp32CameraClient,
    Esp32CameraError,
)
from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibrationFileError,
    load_grid_calibration,
)
from ai_logistics_robot.domain.errors import DomainValidationError

DEFAULT_CALIBRATION = Path(
    "configs/hardware/grid_calibration_7x7.yaml"
)


def build_parser() -> ArgumentParser:
    """Build the explicit calibrated-camera diagnostic command."""

    parser = ArgumentParser(
        description=(
            "Capture one ESP32-CAM JPEG frame and rectify it "
            "through a validated grid calibration."
        )
    )
    parser.add_argument(
        "--url",
        required=True,
        help="Camera HTTP origin, for example http://192.168.1.109.",
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
        help="Destination path for the rectified image.",
    )
    parser.add_argument(
        "--timeout",
        default=5.0,
        type=float,
        help="Positive HTTP timeout in seconds. Default: 5.",
    )
    return parser


def main(
    argv: Sequence[str] | None = None,
) -> int:
    """Capture, rectify, and save one explicitly requested frame."""

    arguments = build_parser().parse_args(argv)

    try:
        calibration = load_grid_calibration(
            arguments.calibration
        )
        camera = Esp32CameraClient(
            base_url=arguments.url,
            timeout_seconds=arguments.timeout,
        )
        calibrated_camera = CalibratedGridCamera(
            camera=camera,
            calibration=calibration,
        )
        rectified_frame = (
            calibrated_camera.capture_rectified()
        )
    except (
        DomainValidationError,
        Esp32CameraError,
        GridCalibrationFileError,
        CalibratedGridCameraError,
    ) as error:
        print(
            f"Calibrated grid diagnostic failed: {error}",
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
            rectified_frame,
        )
    except (OSError, cv2.error) as error:
        print(
            f"Rectified grid image could not be saved: {error}",
            file=sys.stderr,
        )
        return 2

    if not image_written:
        print(
            "Rectified grid image could not be saved.",
            file=sys.stderr,
        )
        return 2

    print("Calibrated ESP32-CAM capture succeeded.")
    print(f"Endpoint: {camera.capture_url}")
    print(
        f"Calibration: {arguments.calibration.resolve()}"
    )
    print(
        "Rectified geometry: "
        f"{calibration.output_width}x"
        f"{calibration.output_height} pixels."
    )
    print(f"Saved to: {output_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())