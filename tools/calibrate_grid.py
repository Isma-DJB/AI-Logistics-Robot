"""Interactively calibrate the fixed overhead camera against the 7x7 grid."""

import argparse
import sys
from pathlib import Path
from typing import Literal, TypeAlias

import cv2
import numpy as np
import yaml

from ai_logistics_robot.adapters.hardware.grid_calibration import (
    CornerPoints,
    GridCalibration,
    rectify_frame,
)

PixelPoint: TypeAlias = tuple[int, int]
ReviewAction: TypeAlias = Literal["save", "redo", "cancel"]

DEFAULT_REFERENCE_IMAGE = Path(
    "docs/assets/i-1.0/images/grid-reference-7x7-svga.jpg"
)
DEFAULT_RECTIFIED_IMAGE = Path(
    "docs/assets/i-1.0/images/grid-reference-7x7-rectified.jpg"
)
DEFAULT_CALIBRATION_FILE = Path(
    "configs/hardware/grid_calibration_7x7.yaml"
)

CORNER_LABELS = (
    "TOP-LEFT",
    "TOP-RIGHT",
    "BOTTOM-RIGHT",
    "BOTTOM-LEFT",
)


def parse_arguments() -> argparse.Namespace:
    """Parse calibration command-line arguments."""

    parser = argparse.ArgumentParser(
        description=(
            "Select the four inner black-boundary corners and create a "
            "rectified 7x7 grid reference."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_REFERENCE_IMAGE,
        help="Reference ESP32-CAM image.",
    )
    parser.add_argument(
        "--config-output",
        type=Path,
        default=DEFAULT_CALIBRATION_FILE,
        help="Generated YAML calibration file.",
    )
    parser.add_argument(
        "--preview-output",
        type=Path,
        default=DEFAULT_RECTIFIED_IMAGE,
        help="Generated rectified reference image.",
    )
    parser.add_argument(
        "--pixels-per-cell",
        type=int,
        default=100,
        help="Rectified pixels assigned to each logical cell.",
    )
    return parser.parse_args()


def select_corners(image: np.ndarray) -> list[PixelPoint] | None:
    """Collect four ordered grid corners from an OpenCV window."""

    window_name = "I-1.0 grid calibration"
    selected_points: list[PixelPoint] = []

    def handle_mouse(
        event: int,
        x_coordinate: int,
        y_coordinate: int,
        flags: int,
        parameter: object,
    ) -> None:
        del flags
        del parameter

        if event == cv2.EVENT_LBUTTONDOWN and len(selected_points) < 4:
            selected_points.append((x_coordinate, y_coordinate))
        elif event == cv2.EVENT_RBUTTONDOWN:
            selected_points.clear()

    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, handle_mouse)

    try:
        while True:
            preview = draw_selected_corners(image, selected_points)
            cv2.imshow(window_name, preview)

            key = cv2.waitKey(20) & 0xFF

            if key == 27:
                return None

            if key in {ord("r"), ord("R")}:
                selected_points.clear()

            if key in {10, 13} and len(selected_points) == 4:
                return selected_points.copy()
    finally:
        cv2.destroyWindow(window_name)


def draw_selected_corners(
    image: np.ndarray,
    selected_points: list[PixelPoint],
) -> np.ndarray:
    """Draw selected points, their order, and calibration instructions."""

    preview = image.copy()

    for index, point in enumerate(selected_points):
        cv2.circle(preview, point, 7, (0, 255, 255), -1)
        cv2.putText(
            preview,
            f"{index + 1} {CORNER_LABELS[index]}",
            (point[0] + 10, point[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        if index > 0:
            cv2.line(
                preview,
                selected_points[index - 1],
                point,
                (0, 255, 255),
                2,
            )

    if len(selected_points) == 4:
        cv2.line(
            preview,
            selected_points[-1],
            selected_points[0],
            (0, 255, 255),
            2,
        )

    return preview


def review_rectification(
    rectified: np.ndarray,
    calibration: GridCalibration,
) -> ReviewAction:
    """Display a temporary grid overlay before saving raw output."""

    window_name = "I-1.0 rectified grid review"
    preview = rectified.copy()

    for column in range(1, calibration.columns):
        x_coordinate = column * calibration.pixels_per_cell
        cv2.line(
            preview,
            (x_coordinate, 0),
            (x_coordinate, calibration.output_height - 1),
            (0, 255, 255),
            1,
        )

    for row in range(1, calibration.rows):
        y_coordinate = row * calibration.pixels_per_cell
        cv2.line(
            preview,
            (0, y_coordinate),
            (calibration.output_width - 1, y_coordinate),
            (0, 255, 255),
            1,
        )

    cv2.rectangle(
        preview,
        (0, 0),
        (calibration.output_width - 1, calibration.output_height - 1),
        (0, 255, 255),
        2,
    )

    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(20) & 0xFF

            if key in {10, 13, ord("s"), ord("S")}:
                return "save"

            if key in {ord("r"), ord("R")}:
                return "redo"

            if key == 27:
                return "cancel"
    finally:
        cv2.destroyWindow(window_name)


def build_calibration_document(
    calibration: GridCalibration,
    reference_image: Path,
) -> dict[str, object]:
    """Create one portable YAML calibration document."""

    labels = (
        "top_left",
        "top_right",
        "bottom_right",
        "bottom_left",
    )
    source_points = {
        label: [float(point[0]), float(point[1])]
        for label, point in zip(
            labels,
            calibration.source_points,
            strict=True,
        )
    }

    return {
        "schema_version": 1,
        "reference_image": reference_image.as_posix(),
        "source_image": {
            "width": calibration.source_width,
            "height": calibration.source_height,
        },
        "grid": {
            "columns": calibration.columns,
            "rows": calibration.rows,
            "cell_size_cm": calibration.cell_size_cm,
            "physical_width_cm": (
                calibration.columns * calibration.cell_size_cm
            ),
            "physical_height_cm": (
                calibration.rows * calibration.cell_size_cm
            ),
        },
        "source_points": source_points,
        "rectified_image": {
            "width": calibration.output_width,
            "height": calibration.output_height,
            "pixels_per_cell": calibration.pixels_per_cell,
        },
    }


def save_results(
    calibration: GridCalibration,
    rectified: np.ndarray,
    reference_image: Path,
    config_output: Path,
    preview_output: Path,
) -> None:
    """Save the raw rectified image and its YAML calibration."""

    config_output.parent.mkdir(parents=True, exist_ok=True)
    preview_output.parent.mkdir(parents=True, exist_ok=True)

    if not cv2.imwrite(str(preview_output), rectified):
        raise OSError(f"Could not write rectified image: {preview_output}")

    document = build_calibration_document(
        calibration,
        reference_image,
    )
    config_output.write_text(
        yaml.safe_dump(
            document,
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )


def main() -> int:
    """Run interactive fixed-camera grid calibration."""

    arguments = parse_arguments()

    if arguments.pixels_per_cell <= 0:
        print(
            "Calibration failed: --pixels-per-cell must be positive.",
            file=sys.stderr,
        )
        return 2

    image = cv2.imread(str(arguments.input), cv2.IMREAD_COLOR)

    if image is None:
        print(
            f"Calibration failed: image could not be read: {arguments.input}",
            file=sys.stderr,
        )
        return 2

    source_height, source_width = image.shape[:2]

    print("Select the four INNER corners of the black 7x7 boundary.")
    print("Order: top-left, top-right, bottom-right, bottom-left.")
    print("Selection controls: Enter=validate, R=reset, Esc=cancel.")
    print("Review controls: Enter/S=save, R=redo, Esc=cancel.")

    try:
        while True:
            selected_points = select_corners(image)

            if selected_points is None:
                print("Calibration cancelled; no file was written.")
                return 1

            source_points: CornerPoints = (
                (
                    float(selected_points[0][0]),
                    float(selected_points[0][1]),
                ),
                (
                    float(selected_points[1][0]),
                    float(selected_points[1][1]),
                ),
                (
                    float(selected_points[2][0]),
                    float(selected_points[2][1]),
                ),
                (
                    float(selected_points[3][0]),
                    float(selected_points[3][1]),
                ),
            )
            calibration = GridCalibration(
                source_width=source_width,
                source_height=source_height,
                source_points=source_points,
                pixels_per_cell=arguments.pixels_per_cell,
            )
            rectified = rectify_frame(image, calibration)
            review_action = review_rectification(
                rectified,
                calibration,
            )

            if review_action == "redo":
                continue

            if review_action == "cancel":
                print("Calibration cancelled; no file was written.")
                return 1

            save_results(
                calibration=calibration,
                rectified=rectified,
                reference_image=arguments.input,
                config_output=arguments.config_output,
                preview_output=arguments.preview_output,
            )

            print("Grid calibration succeeded.")
            print(f"Configuration: {arguments.config_output.resolve()}")
            print(f"Rectified image: {arguments.preview_output.resolve()}")
            print(
                "Output geometry: "
                f"{calibration.output_width}x"
                f"{calibration.output_height} pixels."
            )
            return 0
    except (OSError, ValueError) as error:
        print(f"Calibration failed: {error}", file=sys.stderr)
        return 2
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())