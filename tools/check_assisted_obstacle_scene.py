"""Review camera changes and render assisted physical evidence."""

from __future__ import annotations

import argparse
from argparse import ArgumentParser, Namespace
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

import cv2

from ai_logistics_robot.adapters.hardware.aruco_robot_detector import (
    ArucoRobotDetector,
)
from ai_logistics_robot.adapters.hardware.grid_calibration import (
    GridCalibrationFileError,
    ImageArray,
    load_grid_calibration,
)
from ai_logistics_robot.adapters.hardware.grid_cell_change_detector import (
    GridCellChangeDetector,
)
from ai_logistics_robot.adapters.visualization.mission_evidence_renderer import (
    MissionEvidenceRenderer,
)
from ai_logistics_robot.control.assisted_obstacle_review import (
    review_obstacle_proposals,
)
from ai_logistics_robot.control.mission_evidence import (
    MissionEvidence,
    MissionEvidenceStatus,
)
from ai_logistics_robot.control.physical_obstacle_evidence import (
    ObstacleEvidence,
    ObstacleEvidenceStatus,
    PhysicalObstacleMap,
)
from ai_logistics_robot.domain.enums import PathPhase
from ai_logistics_robot.domain.errors import DomainError
from ai_logistics_robot.domain.geometry import (
    Position,
    RobotPose,
)
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.perception import (
    Observation,
    PerceptionSnapshot,
)
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.planning.a_star_planner import AStarPlanner


class AssistedObstacleSceneError(RuntimeError):
    """Report one invalid assisted physical-scene diagnostic."""


@dataclass(frozen=True, slots=True)
class AssistedObstacleSceneResult:
    """Retain the complete accepted assisted-scene result."""

    robot_pose: RobotPose
    proposals: tuple[ObstacleEvidence, ...]
    reviewed_evidence: tuple[ObstacleEvidence, ...]
    obstacle_map: PhysicalObstacleMap
    plan: PathPlan
    next_position: Position | None
    output_path: Path


def _parse_position(value: str) -> Position:
    """Parse one command-line coordinate written as x,y."""

    parts = value.split(",")

    if len(parts) != 2:
        raise argparse.ArgumentTypeError(
            "position must use the form x,y."
        )

    try:
        x_coordinate = int(parts[0].strip())
        y_coordinate = int(parts[1].strip())
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "position coordinates must be integers."
        ) from error

    return Position(
        x=x_coordinate,
        y=y_coordinate,
    )


def build_parser() -> ArgumentParser:
    """Build the guarded offline diagnostic parser."""

    parser = ArgumentParser(
        description=(
            "Review camera-detected grid changes, replan, "
            "and render assisted Batch E evidence."
        )
    )
    parser.add_argument(
        "--reference",
        required=True,
        type=Path,
        help=(
            "Rectified obstacle-free reference captured with "
            "the same target-light state."
        ),
    )
    parser.add_argument(
        "--current",
        required=True,
        type=Path,
        help="Rectified current physical scene.",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Destination annotated JPEG or PNG.",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path(
            r"configs\hardware\grid_calibration_7x7.yaml"
        ),
        help="Validated grid calibration YAML.",
    )
    parser.add_argument(
        "--robot-id",
        default="robot-1",
        help="Physical robot identifier.",
    )
    parser.add_argument(
        "--marker-id",
        type=int,
        default=0,
        help="Expected ArUco robot marker ID.",
    )
    parser.add_argument(
        "--base-position",
        type=_parse_position,
        default=Position(x=0, y=0),
        help="Base cell as x,y. Default: 0,0.",
    )
    parser.add_argument(
        "--target-position",
        type=_parse_position,
        default=Position(x=5, y=5),
        help="Non-traversable physical target cell as x,y.",
    )
    parser.add_argument(
        "--confirm-obstacle",
        action="append",
        type=_parse_position,
        default=[],
        help=(
            "Camera proposal confirmed by the operator as x,y. "
            "Repeat for multiple obstacles."
        ),
    )
    parser.add_argument(
        "--clear-proposal",
        action="append",
        type=_parse_position,
        default=[],
        help=(
            "Camera proposal rejected by the operator as x,y. "
            "Repeat for multiple shadows or false positives."
        ),
    )
    parser.add_argument(
        "--target-active",
        action="store_true",
        help="Record the independently confirmed active target.",
    )
    parser.add_argument(
        "--phase",
        choices=tuple(
            phase.value
            for phase in PathPhase
        ),
        default=PathPhase.OUTBOUND.value,
        help="Path phase. Default: OUTBOUND.",
    )
    parser.add_argument(
        "--scenario-id",
        default="batch-e-assisted-physical",
        help="Evidence scenario identifier.",
    )
    parser.add_argument(
        "--step-number",
        type=int,
        default=1,
        help="Positive evidence step number.",
    )
    return parser


def _load_frame(
    path: Path,
    *,
    label: str,
) -> ImageArray:
    """Decode one required rectified image."""

    frame = cv2.imread(str(path))

    if frame is None:
        raise AssistedObstacleSceneError(
            f"unable to decode {label} image: {path}."
        )

    return frame


def _authorized_arrival_positions(
    world: GridMap,
) -> tuple[Position, ...]:
    """Return traversable cardinal cells beside the target."""

    target = world.target_position
    candidates = (
        Position(x=target.x, y=target.y - 1),
        Position(x=target.x - 1, y=target.y),
        Position(x=target.x + 1, y=target.y),
        Position(x=target.x, y=target.y + 1),
    )

    positions = tuple(
        position
        for position in candidates
        if _is_inside_world(
            position,
            world=world,
        )
        and world.is_traversable(position)
    )

    if not positions:
        raise AssistedObstacleSceneError(
            "no traversable cardinal arrival remains "
            "beside the physical target."
        )

    return positions


def _is_inside_world(
    position: Position,
    *,
    world: GridMap,
) -> bool:
    """Return whether a coordinate lies inside the grid."""

    minimum_x = world.origin.x
    minimum_y = world.origin.y

    return (
        minimum_x
        <= position.x
        < minimum_x + world.width
        and minimum_y
        <= position.y
        < minimum_y + world.height
    )


def _build_observations(
    evidence: tuple[ObstacleEvidence, ...],
) -> tuple[Observation, ...]:
    """Translate reviewed obstacle evidence into observations."""

    return tuple(
        Observation(
            kind=(
                "physical_obstacle_confirmed"
                if item.status
                is ObstacleEvidenceStatus.CONFIRMED
                else "camera_change_cleared"
            ),
            position=item.position,
            confidence=item.confidence,
        )
        for item in evidence
    )


def _run_diagnostic(
    arguments: Namespace,
) -> AssistedObstacleSceneResult:
    """Execute one complete offline assisted-scene review."""

    if arguments.step_number <= 0:
        raise AssistedObstacleSceneError(
            "step-number must be positive."
        )

    calibration = load_grid_calibration(
        arguments.calibration
    )
    reference_frame = _load_frame(
        arguments.reference,
        label="reference",
    )
    current_frame = _load_frame(
        arguments.current,
        label="current",
    )

    robot_pose = ArucoRobotDetector(
        calibration=calibration,
        marker_id=arguments.marker_id,
    ).detect(current_frame).pose

    base_world = GridMap(
        width=calibration.columns,
        height=calibration.rows,
        cell_size_cm=calibration.cell_size_cm,
        origin=Position(x=0, y=0),
        base_position=arguments.base_position,
        target_position=arguments.target_position,
    )

    proposals = GridCellChangeDetector(
        calibration=calibration,
    ).detect(
        reference_frame,
        current_frame,
        excluded_positions=frozenset(
            {
                robot_pose.position,
                arguments.target_position,
            }
        ),
    )

    reviewed_evidence = review_obstacle_proposals(
        proposals=proposals,
        confirmed_positions=frozenset(
            arguments.confirm_obstacle
        ),
        cleared_positions=frozenset(
            arguments.clear_proposal
        ),
    )

    obstacle_map = PhysicalObstacleMap(
        base_world=base_world,
        evidence=reviewed_evidence,
    )
    world = obstacle_map.confirmed_world
    arrivals = _authorized_arrival_positions(world)

    plan = AStarPlanner().create_plan(
        mission_id=arguments.scenario_id,
        robot_id=arguments.robot_id,
        start_pose=robot_pose,
        authorized_goals=arrivals,
        world=world,
        phase=PathPhase(arguments.phase),
        version=arguments.step_number,
    )

    next_position = (
        plan.positions[1]
        if len(plan.positions) > 1
        else None
    )

    snapshot = PerceptionSnapshot(
        robot_id=arguments.robot_id,
        captured_at=datetime.now(UTC),
        robot_pose=robot_pose,
        observations=_build_observations(
            reviewed_evidence
        ),
        target_active=arguments.target_active,
        hazard_detected=False,
    )

    evidence = MissionEvidence(
        scenario_id=arguments.scenario_id,
        step_number=arguments.step_number,
        stage="operator-confirmed-replan",
        status=MissionEvidenceStatus.REPLANNED,
        world=world,
        perception=snapshot,
        obstacle_map=obstacle_map,
        plan=plan,
        confirmed_positions=(robot_pose.position,),
        next_position=next_position,
    )

    rendered = MissionEvidenceRenderer(
        calibration=calibration,
    ).render(
        current_frame,
        evidence,
    )

    arguments.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    if not cv2.imwrite(
        str(arguments.output),
        rendered,
    ):
        raise AssistedObstacleSceneError(
            "unable to write annotated evidence image."
        )

    return AssistedObstacleSceneResult(
        robot_pose=robot_pose,
        proposals=proposals,
        reviewed_evidence=reviewed_evidence,
        obstacle_map=obstacle_map,
        plan=plan,
        next_position=next_position,
        output_path=arguments.output.resolve(),
    )


def _format_positions(
    positions: Sequence[Position],
) -> str:
    """Render one compact coordinate sequence."""

    if not positions:
        return "none"

    return " -> ".join(
        f"({position.x},{position.y})"
        for position in positions
    )


def main(
    argv: Sequence[str] | None = None,
) -> int:
    """Run one guarded assisted physical-scene diagnostic."""

    parser = build_parser()
    arguments = parser.parse_args(argv)

    try:
        result = _run_diagnostic(arguments)
    except (
        AssistedObstacleSceneError,
        DomainError,
        GridCalibrationFileError,
        OSError,
        ValueError,
    ) as error:
        print(
            "Assisted obstacle scene diagnostic failed: "
            f"{error}"
        )
        return 2

    confirmed = tuple(
        item.position
        for item in result.reviewed_evidence
        if item.status
        is ObstacleEvidenceStatus.CONFIRMED
    )
    cleared = tuple(
        item.position
        for item in result.reviewed_evidence
        if item.status
        is ObstacleEvidenceStatus.CLEARED
    )

    proposal_positions = tuple(
        item.position
        for item in result.proposals
    )
    planning_obstacles = tuple(
        sorted(
            result.obstacle_map.confirmed_world.obstacles,
            key=lambda value: (value.y, value.x),
        )
    )
    next_positions = (
        ()
        if result.next_position is None
        else (result.next_position,)
    )

    print("Assisted obstacle scene diagnostic succeeded.")
    print(
        "Robot pose: "
        f"({result.robot_pose.position.x},"
        f"{result.robot_pose.position.y}) "
        f"{result.robot_pose.heading.value}"
    )
    print(
        "Camera proposals: "
        f"{_format_positions(proposal_positions)}"
    )
    print(
        "Operator-confirmed obstacles: "
        f"{_format_positions(confirmed)}"
    )
    print(
        "Operator-cleared proposals: "
        f"{_format_positions(cleared)}"
    )
    print(
        "Confirmed planning obstacles: "
        f"{_format_positions(planning_obstacles)}"
    )
    print(
        "Selected arrival: "
        f"({result.plan.goal.x},{result.plan.goal.y})"
    )
    print(
        "Planned path: "
        f"{_format_positions(result.plan.positions)}"
    )
    print(
        "Next position: "
        f"{_format_positions(next_positions)}"
    )
    print(f"Annotated evidence: {result.output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())