"""Camera proposals for changed physical grid cells."""

from __future__ import annotations

from math import isfinite

import cv2
import numpy as np

from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
    ImageArray,
)
from ai_logistics_robot.control.physical_obstacle_evidence import (
    ObstacleEvidence,
    ObstacleEvidenceSource,
    ObstacleEvidenceStatus,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position


class GridCellChangeDetector:
    """Propose changed cells without confirming obstacles."""

    __slots__ = (
        "_calibration",
        "_cell_margin_pixels",
        "_grid_origin",
        "_minimum_changed_fraction",
        "_pixel_difference_threshold",
    )

    def __init__(
        self,
        *,
        calibration: GridCalibration,
        pixel_difference_threshold: int = 30,
        minimum_changed_fraction: float = 0.12,
        cell_margin_pixels: int = 8,
        grid_origin: Position | None = None,
    ) -> None:
        """Validate and retain deterministic comparison settings."""

        if not isinstance(
            calibration,
            GridCalibration,
        ):
            raise DomainValidationError(
                "calibration must be a GridCalibration instance."
            )

        if (
            isinstance(pixel_difference_threshold, bool)
            or not isinstance(
                pixel_difference_threshold,
                int,
            )
            or not 1
            <= pixel_difference_threshold
            <= 255
        ):
            raise DomainValidationError(
                "pixel_difference_threshold must be "
                "an integer from 1 through 255."
            )

        if (
            isinstance(minimum_changed_fraction, bool)
            or not isinstance(
                minimum_changed_fraction,
                (int, float),
            )
        ):
            raise DomainValidationError(
                "minimum_changed_fraction must be numeric."
            )

        normalized_fraction = float(
            minimum_changed_fraction
        )
        if (
            not isfinite(normalized_fraction)
            or not 0.0
            < normalized_fraction
            <= 1.0
        ):
            raise DomainValidationError(
                "minimum_changed_fraction must be finite "
                "and greater than zero through one."
            )

        if (
            isinstance(cell_margin_pixels, bool)
            or not isinstance(cell_margin_pixels, int)
            or cell_margin_pixels < 0
            or cell_margin_pixels * 2
            >= calibration.pixels_per_cell
        ):
            raise DomainValidationError(
                "cell_margin_pixels must leave a positive "
                "cell interior."
            )

        if grid_origin is None:
            normalized_grid_origin = Position(
                x=0,
                y=0,
            )
        elif isinstance(grid_origin, Position):
            normalized_grid_origin = grid_origin
        else:
            raise DomainValidationError(
                "grid_origin must be a Position instance "
                "or None."
            )

        self._calibration = calibration
        self._pixel_difference_threshold = (
            pixel_difference_threshold
        )
        self._minimum_changed_fraction = (
            normalized_fraction
        )
        self._cell_margin_pixels = (
            cell_margin_pixels
        )
        self._grid_origin = normalized_grid_origin

    @property
    def calibration(self) -> GridCalibration:
        """Return the immutable calibration."""

        return self._calibration

    def detect(
        self,
        reference_frame: ImageArray,
        current_frame: ImageArray,
        *,
        excluded_positions: frozenset[Position] = (
            frozenset()
        ),
    ) -> tuple[ObstacleEvidence, ...]:
        """Return ordered unconfirmed camera proposals."""

        self._validate_frame(
            "reference_frame",
            reference_frame,
        )
        self._validate_frame(
            "current_frame",
            current_frame,
        )
        self._validate_exclusions(
            excluded_positions
        )

        reference_gray = self._to_gray(
            reference_frame
        )
        current_gray = self._to_gray(
            current_frame
        )

        reference_blurred = cv2.GaussianBlur(
            reference_gray,
            (5, 5),
            0,
        )
        current_blurred = cv2.GaussianBlur(
            current_gray,
            (5, 5),
            0,
        )
        difference = cv2.absdiff(
            reference_blurred,
            current_blurred,
        )

        proposals: list[ObstacleEvidence] = []

        for logical_y in range(
            self._calibration.rows
        ):
            for logical_x in range(
                self._calibration.columns
            ):
                position = Position(
                    x=self._grid_origin.x
                    + logical_x,
                    y=self._grid_origin.y
                    + logical_y,
                )

                if position in excluded_positions:
                    continue

                changed_fraction = (
                    self._changed_fraction(
                        difference,
                        logical_x=logical_x,
                        logical_y=logical_y,
                    )
                )

                if (
                    changed_fraction
                    < self._minimum_changed_fraction
                ):
                    continue

                proposals.append(
                    ObstacleEvidence(
                        position=position,
                        source=(
                            ObstacleEvidenceSource.CAMERA
                        ),
                        status=(
                            ObstacleEvidenceStatus.DETECTED
                        ),
                        confidence=changed_fraction,
                    )
                )

        return tuple(proposals)

    def _validate_frame(
        self,
        name: str,
        frame: object,
    ) -> None:
        """Require one matching uint8 grayscale or BGR frame."""

        if not isinstance(frame, np.ndarray):
            raise DomainValidationError(
                f"{name} must be a NumPy array."
            )

        if frame.dtype != np.uint8:
            raise DomainValidationError(
                f"{name} must use uint8 pixels."
            )

        valid_shape = (
            frame.ndim == 2
            or (
                frame.ndim == 3
                and frame.shape[2] == 3
            )
        )
        if not valid_shape:
            raise DomainValidationError(
                f"{name} must be grayscale or "
                "three-channel BGR."
            )

        if (
            frame.shape[0]
            != self._calibration.output_height
            or frame.shape[1]
            != self._calibration.output_width
        ):
            raise InvariantViolationError(
                f"{name} dimensions must match "
                "the rectified calibration."
            )

    @staticmethod
    def _validate_exclusions(
        excluded_positions: object,
    ) -> None:
        """Require immutable position exclusions."""

        if not isinstance(
            excluded_positions,
            frozenset,
        ):
            raise DomainValidationError(
                "excluded_positions must be "
                "an immutable frozenset."
            )

        if not all(
            isinstance(position, Position)
            for position in excluded_positions
        ):
            raise DomainValidationError(
                "every excluded position must be "
                "a Position instance."
            )

    @staticmethod
    def _to_gray(
        frame: ImageArray,
    ) -> ImageArray:
        """Return one independent grayscale image."""

        if frame.ndim == 2:
            return np.asarray(
                frame.copy(),
                dtype=np.uint8,
            )

        return np.asarray(
            cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY,
            ),
            dtype=np.uint8,
        )

    def _changed_fraction(
        self,
        difference: ImageArray,
        *,
        logical_x: int,
        logical_y: int,
    ) -> float:
        """Measure changed pixels inside one cell interior."""

        pixels = self._calibration.pixels_per_cell
        image_row = (
            self._calibration.rows
            - 1
            - logical_y
        )
        margin = self._cell_margin_pixels

        left = logical_x * pixels + margin
        right = (logical_x + 1) * pixels - margin
        top = image_row * pixels + margin
        bottom = (image_row + 1) * pixels - margin

        cell_difference = difference[
            top:bottom,
            left:right,
        ]
        changed_pixels = np.count_nonzero(
            cell_difference
            >= self._pixel_difference_threshold
        )
        return float(
            changed_pixels
            / cell_difference.size
        )
