"""Perspective calibration for one fixed overhead grid camera."""

from collections.abc import Mapping
from dataclasses import dataclass
from math import isclose, isfinite
from pathlib import Path
from typing import TypeAlias, cast

import cv2
import numpy as np
import yaml
from numpy.typing import NDArray

Point: TypeAlias = tuple[float, float]
CornerPoints: TypeAlias = tuple[Point, Point, Point, Point]
ImageArray: TypeAlias = NDArray[np.uint8]
PerspectiveMatrix: TypeAlias = NDArray[np.float64]

class GridCalibrationFileError(RuntimeError):
    """Report one unreadable or invalid grid calibration file."""


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

def load_grid_calibration(path: Path) -> GridCalibration:
    """Load and validate one versioned YAML grid calibration."""

    try:
        document: object = yaml.safe_load(
            path.read_text(encoding="utf-8")
        )
    except (OSError, UnicodeError) as error:
        raise GridCalibrationFileError(
            "Grid calibration file could not be read."
        ) from error
    except yaml.YAMLError as error:
        raise GridCalibrationFileError(
            "Grid calibration file is not valid YAML."
        ) from error

    root = _require_mapping(document, "calibration document")
    schema_version = _require_integer(
        root.get("schema_version"),
        "schema_version",
    )

    if schema_version != 1:
        raise GridCalibrationFileError(
            "Grid calibration schema_version must be 1."
        )

    source_image = _require_mapping(
        root.get("source_image"),
        "source_image",
    )
    grid = _require_mapping(
        root.get("grid"),
        "grid",
    )
    source_points_document = _require_mapping(
        root.get("source_points"),
        "source_points",
    )
    rectified_image = _require_mapping(
        root.get("rectified_image"),
        "rectified_image",
    )

    source_points: CornerPoints = (
        _require_point(
            source_points_document.get("top_left"),
            "source_points.top_left",
        ),
        _require_point(
            source_points_document.get("top_right"),
            "source_points.top_right",
        ),
        _require_point(
            source_points_document.get("bottom_right"),
            "source_points.bottom_right",
        ),
        _require_point(
            source_points_document.get("bottom_left"),
            "source_points.bottom_left",
        ),
    )

    try:
        calibration = GridCalibration(
            source_width=_require_positive_integer(
                source_image.get("width"),
                "source_image.width",
            ),
            source_height=_require_positive_integer(
                source_image.get("height"),
                "source_image.height",
            ),
            source_points=source_points,
            columns=_require_positive_integer(
                grid.get("columns"),
                "grid.columns",
            ),
            rows=_require_positive_integer(
                grid.get("rows"),
                "grid.rows",
            ),
            cell_size_cm=_require_positive_number(
                grid.get("cell_size_cm"),
                "grid.cell_size_cm",
            ),
            pixels_per_cell=_require_positive_integer(
                rectified_image.get("pixels_per_cell"),
                "rectified_image.pixels_per_cell",
            ),
        )
    except ValueError as error:
        raise GridCalibrationFileError(
            f"Grid calibration geometry is invalid: {error}"
        ) from error

    recorded_physical_width = _require_positive_number(
        grid.get("physical_width_cm"),
        "grid.physical_width_cm",
    )
    recorded_physical_height = _require_positive_number(
        grid.get("physical_height_cm"),
        "grid.physical_height_cm",
    )
    expected_physical_width = (
        calibration.columns * calibration.cell_size_cm
    )
    expected_physical_height = (
        calibration.rows * calibration.cell_size_cm
    )

    if not isclose(
        recorded_physical_width,
        expected_physical_width,
        rel_tol=0.0,
        abs_tol=1e-6,
    ):
        raise GridCalibrationFileError(
            "grid.physical_width_cm is inconsistent."
        )

    if not isclose(
        recorded_physical_height,
        expected_physical_height,
        rel_tol=0.0,
        abs_tol=1e-6,
    ):
        raise GridCalibrationFileError(
            "grid.physical_height_cm is inconsistent."
        )

    recorded_output_width = _require_positive_integer(
        rectified_image.get("width"),
        "rectified_image.width",
    )
    recorded_output_height = _require_positive_integer(
        rectified_image.get("height"),
        "rectified_image.height",
    )

    if recorded_output_width != calibration.output_width:
        raise GridCalibrationFileError(
            "rectified_image.width is inconsistent."
        )

    if recorded_output_height != calibration.output_height:
        raise GridCalibrationFileError(
            "rectified_image.height is inconsistent."
        )

    return calibration


def _require_mapping(
    value: object,
    field_name: str,
) -> Mapping[str, object]:
    """Return one required YAML mapping."""

    if not isinstance(value, Mapping):
        raise GridCalibrationFileError(
            f"{field_name} must be a mapping."
        )

    return cast(Mapping[str, object], value)


def _require_integer(
    value: object,
    field_name: str,
) -> int:
    """Return one required non-boolean integer."""

    if isinstance(value, bool) or not isinstance(value, int):
        raise GridCalibrationFileError(
            f"{field_name} must be an integer."
        )

    return value


def _require_positive_integer(
    value: object,
    field_name: str,
) -> int:
    """Return one required positive integer."""

    integer = _require_integer(value, field_name)

    if integer <= 0:
        raise GridCalibrationFileError(
            f"{field_name} must be positive."
        )

    return integer


def _require_number(
    value: object,
    field_name: str,
) -> float:
    """Return one required finite number."""

    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not isfinite(float(value))
    ):
        raise GridCalibrationFileError(
            f"{field_name} must be a finite number."
        )

    return float(value)


def _require_positive_number(
    value: object,
    field_name: str,
) -> float:
    """Return one required positive finite number."""

    number = _require_number(value, field_name)

    if number <= 0.0:
        raise GridCalibrationFileError(
            f"{field_name} must be positive."
        )

    return number


def _require_point(
    value: object,
    field_name: str,
) -> Point:
    """Return one required two-coordinate YAML point."""

    if not isinstance(value, list) or len(value) != 2:
        raise GridCalibrationFileError(
            f"{field_name} must contain two coordinates."
        )

    return (
        _require_number(value[0], f"{field_name}[0]"),
        _require_number(value[1], f"{field_name}[1]"),
    )