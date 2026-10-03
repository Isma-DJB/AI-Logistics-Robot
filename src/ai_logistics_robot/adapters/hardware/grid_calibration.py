"""Perspective calibration for one fixed overhead grid camera."""

from dataclasses import dataclass
from math import isfinite
from typing import TypeAlias

import cv2
import numpy as np
from numpy.typing import NDArray

Point: TypeAlias = tuple[float, float]
CornerPoints: TypeAlias = tuple[Point, Point, Point, Point]
ImageArray: TypeAlias = NDArray[np.uint8]
PerspectiveMatrix: TypeAlias = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class GridCalibration:
    """Describe the mapping from one camera frame to a logical grid."""

    source_width: int
    source_height: int
    source_points: CornerPoints
    columns: int = 7
    rows: int = 7
    cell_size_cm: float = 20.0
    pixels_per_cell: int = 100

    def __post_init__(self) -> None:
        """Validate dimensions and ordered source corner coordinates."""

        if self.source_width <= 0 or self.source_height <= 0:
            raise ValueError("Source image dimensions must be positive.")

        if self.columns <= 0 or self.rows <= 0:
            raise ValueError("Grid dimensions must be positive.")

        if not isfinite(self.cell_size_cm) or self.cell_size_cm <= 0:
            raise ValueError("Cell size must be a positive finite value.")

        if self.pixels_per_cell <= 0:
            raise ValueError("Pixels per cell must be positive.")

        if len(set(self.source_points)) != 4:
            raise ValueError("The four source corner points must be unique.")

        for x_coordinate, y_coordinate in self.source_points:
            if not isfinite(x_coordinate) or not isfinite(y_coordinate):
                raise ValueError("Source corner coordinates must be finite.")

            if not 0 <= x_coordinate < self.source_width:
                raise ValueError("A source corner lies outside the image width.")

            if not 0 <= y_coordinate < self.source_height:
                raise ValueError("A source corner lies outside the image height.")

        if _polygon_area(self.source_points) < 1.0:
            raise ValueError("Source corner points must form a valid quadrilateral.")

    @property
    def output_width(self) -> int:
        """Return the rectified image width in pixels."""

        return self.columns * self.pixels_per_cell

    @property
    def output_height(self) -> int:
        """Return the rectified image height in pixels."""

        return self.rows * self.pixels_per_cell


def build_perspective_matrix(
    calibration: GridCalibration,
) -> PerspectiveMatrix:
    """Build the camera-to-grid perspective transformation matrix."""

    source = np.asarray(calibration.source_points, dtype=np.float32)
    destination = np.asarray(
        (
            (0.0, 0.0),
            (float(calibration.output_width - 1), 0.0),
            (
                float(calibration.output_width - 1),
                float(calibration.output_height - 1),
            ),
            (0.0, float(calibration.output_height - 1)),
        ),
        dtype=np.float32,
    )

    matrix = cv2.getPerspectiveTransform(source, destination)
    return np.asarray(matrix, dtype=np.float64)


def rectify_frame(
    frame: ImageArray,
    calibration: GridCalibration,
) -> ImageArray:
    """Transform one camera frame into a top-down logical-grid image."""

    if frame.dtype != np.uint8:
        raise ValueError("Camera frame must use unsigned 8-bit pixels.")

    if frame.ndim not in {2, 3}:
        raise ValueError("Camera frame must be a grayscale or colour image.")

    source_height, source_width = frame.shape[:2]

    if (
        source_width != calibration.source_width
        or source_height != calibration.source_height
    ):
        raise ValueError(
            "Camera frame dimensions do not match the calibration source."
        )

    matrix = build_perspective_matrix(calibration)
    rectified = cv2.warpPerspective(
        frame,
        matrix,
        (calibration.output_width, calibration.output_height),
    )

    return np.asarray(rectified, dtype=np.uint8)


def rectified_pixel_to_cell(
    x_coordinate: int,
    y_coordinate: int,
    calibration: GridCalibration,
) -> tuple[int, int]:
    """Convert a rectified pixel into a zero-based grid column and row."""

    if not 0 <= x_coordinate < calibration.output_width:
        raise ValueError("Rectified x coordinate lies outside the grid.")

    if not 0 <= y_coordinate < calibration.output_height:
        raise ValueError("Rectified y coordinate lies outside the grid.")

    column = x_coordinate // calibration.pixels_per_cell
    row = y_coordinate // calibration.pixels_per_cell
    return column, row


def _polygon_area(points: CornerPoints) -> float:
    """Return the absolute area of one ordered four-point polygon."""

    doubled_area = 0.0

    for index in range(len(points)):
        current_x, current_y = points[index]
        next_x, next_y = points[(index + 1) % len(points)]
        doubled_area += current_x * next_y - next_x * current_y

    return abs(doubled_area) / 2.0