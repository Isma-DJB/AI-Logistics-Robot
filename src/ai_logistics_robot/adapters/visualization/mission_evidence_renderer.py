"""OpenCV renderer for assisted physical-mission evidence."""

from __future__ import annotations

import cv2
import numpy as np

from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibration,
    ImageArray,
)
from ai_logistics_robot.control.mission_evidence import (
    MissionEvidence,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position

_PLANNED_COLOR = (255, 0, 0)
_CONFIRMED_COLOR = (0, 200, 0)
_NEXT_COLOR = (0, 255, 255)
_OBSTACLE_COLOR = (0, 0, 255)
_TARGET_COLOR = (255, 0, 255)
_ROBOT_COLOR = (255, 255, 255)
_GRID_COLOR = (180, 180, 180)
_TEXT_COLOR = (255, 255, 255)
_TEXT_OUTLINE_COLOR = (0, 0, 0)


class MissionEvidenceRenderer:
    """Render one immutable Batch E evidence state."""

    __slots__ = ("_calibration",)

    def __init__(
        self,
        *,
        calibration: GridCalibration,
    ) -> None:
        """Retain one validated rectified-grid calibration."""

        if not isinstance(
            calibration,
            GridCalibration,
        ):
            raise DomainValidationError(
                "calibration must be a GridCalibration instance."
            )

        self._calibration = calibration

    @property
    def calibration(self) -> GridCalibration:
        """Return the immutable grid calibration."""

        return self._calibration

    def render(
        self,
        frame: ImageArray,
        evidence: MissionEvidence,
    ) -> ImageArray:
        """Return an annotated copy of one rectified frame."""

        self._validate_frame(frame)

        if not isinstance(
            evidence,
            MissionEvidence,
        ):
            raise DomainValidationError(
                "evidence must be a MissionEvidence instance."
            )

        self._validate_geometry(evidence)

        annotated = self._to_bgr_copy(frame)
        self._draw_semantic_cells(
            annotated,
            evidence,
        )
        self._draw_grid(annotated)
        self._draw_paths(
            annotated,
            evidence,
        )
        self._draw_robot(
            annotated,
            evidence,
        )
        self._draw_labels(
            annotated,
            evidence,
        )
        return annotated

    def _validate_frame(
        self,
        frame: object,
    ) -> None:
        """Require one uint8 rectified grayscale or BGR frame."""

        if not isinstance(frame, np.ndarray):
            raise DomainValidationError(
                "frame must be a NumPy array."
            )

        if frame.dtype != np.uint8:
            raise DomainValidationError(
                "frame must use uint8 pixels."
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
                "frame must be grayscale or three-channel BGR."
            )

        if (
            frame.shape[0]
            != self._calibration.output_height
            or frame.shape[1]
            != self._calibration.output_width
        ):
            raise InvariantViolationError(
                "frame dimensions must match the rectified calibration."
            )

    def _validate_geometry(
        self,
        evidence: MissionEvidence,
    ) -> None:
        """Require matching logical and rectified grids."""

        world = evidence.world

        if (
            world.width
            != self._calibration.columns
            or world.height
            != self._calibration.rows
        ):
            raise InvariantViolationError(
                "world dimensions must match the calibration grid."
            )

        if (
            world.cell_size_cm
            != self._calibration.cell_size_cm
        ):
            raise InvariantViolationError(
                "world cell size must match the calibration."
            )

    @staticmethod
    def _to_bgr_copy(
        frame: ImageArray,
    ) -> ImageArray:
        """Copy one frame into a writable BGR image."""

        if frame.ndim == 2:
            return np.asarray(
                cv2.cvtColor(
                    frame,
                    cv2.COLOR_GRAY2BGR,
                ),
                dtype=np.uint8,
            )

        return np.asarray(
            frame.copy(),
            dtype=np.uint8,
        )

    def _draw_semantic_cells(
        self,
        frame: ImageArray,
        evidence: MissionEvidence,
    ) -> None:
        """Draw translucent target and obstacle cells."""

        overlay = frame.copy()

        for obstacle in evidence.world.obstacles:
            self._fill_cell(
                overlay,
                evidence,
                obstacle,
                _OBSTACLE_COLOR,
            )

        self._fill_cell(
            overlay,
            evidence,
            evidence.world.target_position,
            _TARGET_COLOR,
        )

        cv2.addWeighted(
            overlay,
            0.28,
            frame,
            0.72,
            0,
            dst=frame,
        )

    def _draw_grid(
        self,
        frame: ImageArray,
    ) -> None:
        """Draw logical cell boundaries."""

        pixels = self._calibration.pixels_per_cell

        for column in range(
            self._calibration.columns + 1
        ):
            x_coordinate = min(
                column * pixels,
                self._calibration.output_width - 1,
            )
            cv2.line(
                frame,
                (x_coordinate, 0),
                (
                    x_coordinate,
                    self._calibration.output_height - 1,
                ),
                _GRID_COLOR,
                1,
                cv2.LINE_AA,
            )

        for row in range(
            self._calibration.rows + 1
        ):
            y_coordinate = min(
                row * pixels,
                self._calibration.output_height - 1,
            )
            cv2.line(
                frame,
                (0, y_coordinate),
                (
                    self._calibration.output_width - 1,
                    y_coordinate,
                ),
                _GRID_COLOR,
                1,
                cv2.LINE_AA,
            )

    def _draw_paths(
        self,
        frame: ImageArray,
        evidence: MissionEvidence,
    ) -> None:
        """Draw planned, confirmed, and next movement."""

        if evidence.plan is not None:
            self._draw_position_sequence(
                frame,
                evidence,
                evidence.plan.positions,
                _PLANNED_COLOR,
                5,
            )

        self._draw_position_sequence(
            frame,
            evidence,
            evidence.confirmed_positions,
            _CONFIRMED_COLOR,
            7,
        )

        for confirmed in evidence.confirmed_positions:
            cv2.circle(
                frame,
                self._cell_center(
                    evidence,
                    confirmed,
                ),
                8,
                _CONFIRMED_COLOR,
                -1,
                cv2.LINE_AA,
            )

        if evidence.next_position is not None:
            left, top, right, bottom = (
                self._cell_bounds(
                    evidence,
                    evidence.next_position,
                )
            )
            cv2.rectangle(
                frame,
                (left + 3, top + 3),
                (right - 3, bottom - 3),
                _NEXT_COLOR,
                5,
                cv2.LINE_AA,
            )

    def _draw_robot(
        self,
        frame: ImageArray,
        evidence: MissionEvidence,
    ) -> None:
        """Draw confirmed robot cell and cardinal heading."""

        pose = evidence.perception.robot_pose
        left, top, right, bottom = self._cell_bounds(
            evidence,
            pose.position,
        )
        cv2.rectangle(
            frame,
            (left + 8, top + 8),
            (right - 8, bottom - 8),
            _ROBOT_COLOR,
            4,
            cv2.LINE_AA,
        )

        center = self._cell_center(
            evidence,
            pose.position,
        )
        direction_by_heading = {
            Heading.NORTH: (0, -1),
            Heading.EAST: (1, 0),
            Heading.SOUTH: (0, 1),
            Heading.WEST: (-1, 0),
        }
        direction_x, direction_y = (
            direction_by_heading[pose.heading]
        )
        arrow_length = max(
            25,
            self._calibration.pixels_per_cell // 3,
        )
        arrow_end = (
            center[0] + direction_x * arrow_length,
            center[1] + direction_y * arrow_length,
        )
        cv2.arrowedLine(
            frame,
            center,
            arrow_end,
            _ROBOT_COLOR,
            4,
            cv2.LINE_AA,
            tipLength=0.3,
        )

    def _draw_labels(
        self,
        frame: ImageArray,
        evidence: MissionEvidence,
    ) -> None:
        """Draw readable scenario and state labels."""

        pose = evidence.perception.robot_pose
        first_label = (
            f"{evidence.scenario_id} "
            f"step={evidence.step_number} "
            f"status={evidence.status.value}"
        )
        second_label = (
            f"stage={evidence.stage} "
            f"pose=({pose.position.x},{pose.position.y})/"
            f"{pose.heading.value} "
            f"target_active={evidence.perception.target_active}"
        )

        self._draw_text(
            frame,
            first_label,
            (15, 32),
        )
        self._draw_text(
            frame,
            second_label,
            (15, 63),
        )

    @staticmethod
    def _draw_text(
        frame: ImageArray,
        label: str,
        origin: tuple[int, int],
    ) -> None:
        """Draw outlined text over arbitrary backgrounds."""

        cv2.putText(
            frame,
            label,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            _TEXT_OUTLINE_COLOR,
            5,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            label,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            _TEXT_COLOR,
            2,
            cv2.LINE_AA,
        )

    def _fill_cell(
        self,
        frame: ImageArray,
        evidence: MissionEvidence,
        position: Position,
        color: tuple[int, int, int],
    ) -> None:
        """Fill one logical cell."""

        left, top, right, bottom = self._cell_bounds(
            evidence,
            position,
        )
        cv2.rectangle(
            frame,
            (left + 1, top + 1),
            (right - 1, bottom - 1),
            color,
            -1,
        )

    def _cell_bounds(
        self,
        evidence: MissionEvidence,
        position: Position,
    ) -> tuple[int, int, int, int]:
        """Convert one domain position into pixel bounds."""

        world = evidence.world
        column = position.x - world.origin.x
        logical_row = position.y - world.origin.y
        image_row = world.height - 1 - logical_row
        pixels = self._calibration.pixels_per_cell

        left = column * pixels
        top = image_row * pixels
        right = left + pixels - 1
        bottom = top + pixels - 1
        return left, top, right, bottom

    def _cell_center(
        self,
        evidence: MissionEvidence,
        position: Position,
    ) -> tuple[int, int]:
        """Return the center pixel of one logical cell."""

        left, top, right, bottom = self._cell_bounds(
            evidence,
            position,
        )
        return (
            (left + right) // 2,
            (top + bottom) // 2,
        )

    def _draw_position_sequence(
        self,
        frame: ImageArray,
        evidence: MissionEvidence,
        positions: tuple[Position, ...],
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        """Draw one ordered position sequence."""

        if not positions:
            return

        points = np.asarray(
            [
                self._cell_center(
                    evidence,
                    position,
                )
                for position in positions
            ],
            dtype=np.int32,
        ).reshape((-1, 1, 2))

        if len(positions) == 1:
            cv2.circle(
                frame,
                tuple(points[0, 0]),
                thickness,
                color,
                -1,
                cv2.LINE_AA,
            )
            return

        cv2.polylines(
            frame,
            [points],
            False,
            color,
            thickness,
            cv2.LINE_AA,
        )
