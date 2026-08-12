"""Immutable validated settings for the V1 reference scenario."""

from collections.abc import Mapping
from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import cast

import yaml

from ai_logistics_robot.domain.enums import CellLayer, Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.world import GridMap


def _validate_text(name: str, value: object) -> None:
    """Require a non-empty string."""

    if not isinstance(value, str) or not value.strip():
        raise DomainValidationError(
            f"{name} must be a non-empty string."
        )


def _validate_non_negative_number(
    name: str,
    value: object,
    *,
    allow_none: bool = False,
) -> None:
    """Require a finite non-negative number or an explicit None."""

    if value is None and allow_none:
        return

    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not isfinite(float(value))
        or float(value) < 0
    ):
        suffix = " or None" if allow_none else ""
        raise DomainValidationError(
            f"{name} must be a finite non-negative number{suffix}."
        )


def _validate_optional_positive_integer(
    name: str,
    value: object,
) -> None:
    """Require a positive integer or an explicit None."""

    if value is None:
        return

    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < 1
    ):
        raise DomainValidationError(
            f"{name} must be a positive integer or None."
        )


@dataclass(frozen=True, slots=True)
class ScenarioSettings:
    """Deterministic simulation identity and clock settings."""

    scenario_id: str
    clock: str
    random_seed: int | None = None

    def __post_init__(self) -> None:
        """Validate scenario settings."""

        _validate_text("scenario_id", self.scenario_id)
        _validate_text("clock", self.clock)

        if (
            self.random_seed is not None
            and (
                isinstance(self.random_seed, bool)
                or not isinstance(self.random_seed, int)
            )
        ):
            raise DomainValidationError(
                "random_seed must be an integer or None."
            )


@dataclass(frozen=True, slots=True)
class RobotSettings:
    """Platform-independent robot identity and initial geometry."""

    robot_id: str
    initial_pose: RobotPose
    simulated_footprint_cells: int
    safety_margin_cm: float | None = None

    def __post_init__(self) -> None:
        """Validate robot settings without guessing physical values."""

        _validate_text("robot_id", self.robot_id)

        if not isinstance(self.initial_pose, RobotPose):
            raise DomainValidationError(
                "initial_pose must be a RobotPose instance."
            )

        if (
            isinstance(self.simulated_footprint_cells, bool)
            or not isinstance(self.simulated_footprint_cells, int)
            or self.simulated_footprint_cells < 1
        ):
            raise DomainValidationError(
                "simulated_footprint_cells must be a positive integer."
            )

        _validate_non_negative_number(
            "safety_margin_cm",
            self.safety_margin_cm,
            allow_none=True,
        )


@dataclass(frozen=True, slots=True)
class TargetSettings:
    """Target identity, activation, and arrival settings."""

    target_id: str
    target_position: Position
    arrival_policy: str
    debounce_ms: float | None = None
    confidence_threshold: float | None = None

    def __post_init__(self) -> None:
        """Validate target settings."""

        _validate_text("target_id", self.target_id)
        _validate_text("arrival_policy", self.arrival_policy)

        if not isinstance(self.target_position, Position):
            raise DomainValidationError(
                "target_position must be a Position instance."
            )

        _validate_non_negative_number(
            "debounce_ms",
            self.debounce_ms,
            allow_none=True,
        )

        if self.confidence_threshold is not None:
            if (
                isinstance(self.confidence_threshold, bool)
                or not isinstance(
                    self.confidence_threshold,
                    (int, float),
                )
                or not isfinite(float(self.confidence_threshold))
                or not 0.0
                <= float(self.confidence_threshold)
                <= 1.0
            ):
                raise DomainValidationError(
                    "confidence_threshold must lie within "
                    "[0.0, 1.0] or be None."
                )


@dataclass(frozen=True, slots=True)
class MissionSettings:
    """Collection duration and optional mission limits."""

    collection_duration_s: float
    maximum_replans: int | None = None
    timeout_s: float | None = None

    def __post_init__(self) -> None:
        """Validate mission timing and limit settings."""

        _validate_non_negative_number(
            "collection_duration_s",
            self.collection_duration_s,
        )
        _validate_optional_positive_integer(
            "maximum_replans",
            self.maximum_replans,
        )
        _validate_non_negative_number(
            "timeout_s",
            self.timeout_s,
            allow_none=True,
        )


@dataclass(frozen=True, slots=True)
class Settings:
    """Validated configuration required to assemble V1."""

    schema_version: int
    scenario: ScenarioSettings
    grid_map: GridMap
    robot: RobotSettings
    target: TargetSettings
    mission: MissionSettings

    def __post_init__(self) -> None:
        """Validate configuration types and cross-object consistency."""

        if (
            isinstance(self.schema_version, bool)
            or not isinstance(self.schema_version, int)
            or self.schema_version < 1
        ):
            raise DomainValidationError(
                "schema_version must be a positive integer."
            )

        if not isinstance(self.scenario, ScenarioSettings):
            raise DomainValidationError(
                "scenario must be a ScenarioSettings instance."
            )

        if not isinstance(self.grid_map, GridMap):
            raise DomainValidationError(
                "grid_map must be a GridMap instance."
            )

        if not isinstance(self.robot, RobotSettings):
            raise DomainValidationError(
                "robot must be a RobotSettings instance."
            )

        if not isinstance(self.target, TargetSettings):
            raise DomainValidationError(
                "target must be a TargetSettings instance."
            )

        if not isinstance(self.mission, MissionSettings):
            raise DomainValidationError(
                "mission must be a MissionSettings instance."
            )

        if not self.grid_map.contains(
            self.robot.initial_pose.position
        ):
            raise DomainValidationError(
                "the initial robot pose must lie within the grid."
            )

        if not self.grid_map.is_traversable(
            self.robot.initial_pose.position
        ):
            raise DomainValidationError(
                "the initial robot pose must be traversable."
            )

        if (
            self.grid_map.target_position
            != self.target.target_position
        ):
            raise DomainValidationError(
                "grid map and target settings must reference "
                "the same target position."
            )

def _as_mapping(
    name: str,
    value: object,
) -> Mapping[str, object]:
    """Validate an internal YAML mapping."""

    if not isinstance(value, Mapping):
        raise DomainValidationError(
            f"{name} must be a mapping."
        )

    if not all(isinstance(key, str) for key in value):
        raise DomainValidationError(
            f"{name} must contain only string keys."
        )

    return cast(Mapping[str, object], value)


def _required(
    mapping: Mapping[str, object],
    key: str,
) -> object:
    """Return one required configuration value."""

    if key not in mapping:
        raise DomainValidationError(
            f"missing required configuration value: {key}."
        )

    return mapping[key]


def _required_mapping(
    mapping: Mapping[str, object],
    key: str,
) -> Mapping[str, object]:
    """Return one required nested configuration mapping."""

    return _as_mapping(key, _required(mapping, key))


def _parse_position(
    name: str,
    value: object,
) -> Position:
    """Convert one YAML coordinate pair into a Position."""

    if (
        not isinstance(value, (list, tuple))
        or len(value) != 2
    ):
        raise DomainValidationError(
            f"{name} must contain exactly two coordinates."
        )

    return Position(
        x=cast(int, value[0]),
        y=cast(int, value[1]),
    )


def _parse_heading(value: object) -> Heading:
    """Convert one configured heading into its domain enum."""

    if not isinstance(value, str):
        raise DomainValidationError(
            "heading must be a string."
        )

    try:
        return Heading(value)
    except ValueError as error:
        raise DomainValidationError(
            f"unsupported heading: {value}."
        ) from error


def _parse_layers(value: object) -> tuple[CellLayer, ...]:
    """Convert configured layer names into domain enums."""

    if not isinstance(value, list):
        raise DomainValidationError(
            "grid layers must be a list."
        )

    parsed_layers: list[CellLayer] = []

    for layer in value:
        if not isinstance(layer, str):
            raise DomainValidationError(
                "every configured grid layer must be a string."
            )

        try:
            parsed_layers.append(CellLayer(layer))
        except ValueError as error:
            raise DomainValidationError(
                f"unsupported grid layer: {layer}."
            ) from error

    return tuple(parsed_layers)


def _parse_obstacles(
    value: object,
) -> frozenset[Position]:
    """Convert configured obstacle pairs into immutable positions."""

    if not isinstance(value, list):
        raise DomainValidationError(
            "obstacles must be a list."
        )

    positions = tuple(
        _parse_position("obstacle", obstacle)
        for obstacle in value
    )

    if len(positions) != len(set(positions)):
        raise DomainValidationError(
            "configured obstacle positions must be unique."
        )

    return frozenset(positions)


def load_settings(path: str | Path) -> Settings:
    """Load and validate the V1 simulation YAML configuration."""

    configuration_path = Path(path)

    try:
        yaml_text = configuration_path.read_text(
            encoding="utf-8"
        )
    except OSError as error:
        raise DomainValidationError(
            f"unable to read configuration: "
            f"{configuration_path}."
        ) from error

    try:
        raw_document = yaml.safe_load(yaml_text)
    except yaml.YAMLError as error:
        raise DomainValidationError(
            f"invalid YAML configuration: "
            f"{configuration_path}."
        ) from error

    root = _as_mapping("configuration", raw_document)
    scenario_data = _required_mapping(root, "scenario")
    grid_data = _required_mapping(root, "grid")
    base_data = _required_mapping(root, "base")
    robot_data = _required_mapping(root, "robot")
    pose_data = _required_mapping(robot_data, "initial_pose")
    target_data = _required_mapping(root, "target")
    mission_data = _required_mapping(root, "mission")

    origin = _parse_position(
        "grid origin",
        _required(grid_data, "origin"),
    )
    base_position = _parse_position(
        "base position",
        _required(base_data, "position"),
    )
    target_position = _parse_position(
        "target position",
        _required(target_data, "position"),
    )

    robot_pose = RobotPose(
        position=Position(
            x=cast(int, _required(pose_data, "x")),
            y=cast(int, _required(pose_data, "y")),
        ),
        heading=_parse_heading(
            _required(pose_data, "heading")
        ),
    )

    grid_map = GridMap(
        width=cast(int, _required(grid_data, "width")),
        height=cast(int, _required(grid_data, "height")),
        cell_size_cm=cast(
            float,
            _required(grid_data, "cell_size_cm"),
        ),
        origin=origin,
        base_position=base_position,
        target_position=target_position,
        obstacles=_parse_obstacles(
            _required(root, "obstacles")
        ),
        layers=_parse_layers(
            _required(grid_data, "layers")
        ),
    )

    return Settings(
        schema_version=cast(
            int,
            _required(root, "schema_version"),
        ),
        scenario=ScenarioSettings(
            scenario_id=cast(
                str,
                _required(scenario_data, "id"),
            ),
            clock=cast(
                str,
                _required(scenario_data, "clock"),
            ),
            random_seed=cast(
                int | None,
                scenario_data.get("random_seed"),
            ),
        ),
        grid_map=grid_map,
        robot=RobotSettings(
            robot_id=cast(
                str,
                _required(robot_data, "id"),
            ),
            initial_pose=robot_pose,
            simulated_footprint_cells=cast(
                int,
                _required(
                    robot_data,
                    "simulated_footprint_cells",
                ),
            ),
            safety_margin_cm=None,
        ),
        target=TargetSettings(
            target_id=cast(
                str,
                _required(target_data, "id"),
            ),
            target_position=target_position,
            arrival_policy=cast(
                str,
                _required(target_data, "arrival_policy"),
            ),
            debounce_ms=cast(
                float | None,
                target_data.get("debounce_ms"),
            ),
            confidence_threshold=cast(
                float | None,
                target_data.get("confidence_threshold"),
            ),
        ),
        mission=MissionSettings(
            collection_duration_s=cast(
                float,
                _required(
                    mission_data,
                    "collection_duration_s",
                ),
            ),
            maximum_replans=cast(
                int | None,
                mission_data.get("maximum_replans"),
            ),
            timeout_s=cast(
                float | None,
                mission_data.get("timeout_s"),
            ),
        ),
    )