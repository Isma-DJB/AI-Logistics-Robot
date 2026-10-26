"""Calculate stable ambient-light baselines from Arduino telemetry."""

from dataclasses import dataclass

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoSensorSnapshot,
)
from ai_logistics_robot.domain.errors import DomainValidationError

MAX_ANALOG_RAW_VALUE = 1023
MINIMUM_AMBIENT_SAMPLE_COUNT = 5
DEFAULT_MAXIMUM_AMBIENT_LIGHT_RANGE = 10


@dataclass(frozen=True, slots=True)
class ArduinoAmbientLightCalibration:
    """One immutable ambient-light calibration result."""

    ambient_light_raw: int
    sample_count: int
    minimum_light_raw: int
    maximum_light_raw: int

    def __post_init__(self) -> None:
        """Validate calibration values and their relationships."""

        _require_bounded_integer(
            self.ambient_light_raw,
            field_name="ambient_light_raw",
            minimum=0,
            maximum=MAX_ANALOG_RAW_VALUE,
        )
        _require_positive_integer(
            self.sample_count,
            field_name="sample_count",
        )
        _require_bounded_integer(
            self.minimum_light_raw,
            field_name="minimum_light_raw",
            minimum=0,
            maximum=MAX_ANALOG_RAW_VALUE,
        )
        _require_bounded_integer(
            self.maximum_light_raw,
            field_name="maximum_light_raw",
            minimum=0,
            maximum=MAX_ANALOG_RAW_VALUE,
        )

        if self.minimum_light_raw > self.maximum_light_raw:
            raise DomainValidationError(
                "minimum_light_raw must not exceed "
                "maximum_light_raw."
            )

        if not (
            self.minimum_light_raw
            <= self.ambient_light_raw
            <= self.maximum_light_raw
        ):
            raise DomainValidationError(
                "ambient_light_raw must be within the "
                "observed light range."
            )

    @property
    def light_range(self) -> int:
        """Return the observed maximum-minus-minimum range."""

        return (
            self.maximum_light_raw
            - self.minimum_light_raw
        )


def calibrate_ambient_light(
    snapshots: tuple[ArduinoSensorSnapshot, ...],
    *,
    maximum_light_range: int = (
        DEFAULT_MAXIMUM_AMBIENT_LIGHT_RANGE
    ),
) -> ArduinoAmbientLightCalibration:
    """Return a stable baseline from lamp-off sensor snapshots."""

    if not isinstance(snapshots, tuple):
        raise DomainValidationError(
            "snapshots must be an immutable tuple."
        )

    if not all(
        isinstance(snapshot, ArduinoSensorSnapshot)
        for snapshot in snapshots
    ):
        raise DomainValidationError(
            "every snapshot must be an "
            "ArduinoSensorSnapshot instance."
        )

    if len(snapshots) < MINIMUM_AMBIENT_SAMPLE_COUNT:
        raise DomainValidationError(
            "ambient calibration requires at least "
            f"{MINIMUM_AMBIENT_SAMPLE_COUNT} snapshots."
        )

    normalized_maximum_range = _require_bounded_integer(
        maximum_light_range,
        field_name="maximum_light_range",
        minimum=0,
        maximum=MAX_ANALOG_RAW_VALUE,
    )

    light_values = tuple(
        snapshot.light_raw
        for snapshot in snapshots
    )
    minimum_light_raw = min(light_values)
    maximum_light_raw = max(light_values)
    observed_range = (
        maximum_light_raw - minimum_light_raw
    )

    if observed_range > normalized_maximum_range:
        raise DomainValidationError(
            "ambient light samples are too unstable: "
            f"observed range {observed_range} exceeds "
            f"the allowed range {normalized_maximum_range}."
        )

    sample_count = len(light_values)
    ambient_light_raw = (
        sum(light_values) + sample_count // 2
    ) // sample_count

    return ArduinoAmbientLightCalibration(
        ambient_light_raw=ambient_light_raw,
        sample_count=sample_count,
        minimum_light_raw=minimum_light_raw,
        maximum_light_raw=maximum_light_raw,
    )


def _require_positive_integer(
    value: object,
    *,
    field_name: str,
) -> int:
    """Return one validated positive integer."""

    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value <= 0
    ):
        raise DomainValidationError(
            f"{field_name} must be a positive integer."
        )

    return value


def _require_bounded_integer(
    value: object,
    *,
    field_name: str,
    minimum: int,
    maximum: int,
) -> int:
    """Return one integer within an inclusive range."""

    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
        or value > maximum
    ):
        raise DomainValidationError(
            f"{field_name} must be an integer from "
            f"{minimum} through {maximum}."
        )

    return value