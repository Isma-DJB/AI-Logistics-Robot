"""Translate raw Arduino telemetry into stable physical sensor facts."""

from dataclasses import dataclass

from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoSensorSnapshot,
)
from ai_logistics_robot.domain.errors import DomainValidationError

MAX_ANALOG_RAW_VALUE = 1023


@dataclass(frozen=True, slots=True)
class ArduinoSensorInterpretation:
    """One immutable interpretation of a raw Arduino sensor snapshot."""

    sequence_id: int
    left_obstacle_detected: bool
    right_obstacle_detected: bool
    boundary_detected: bool
    frontal_obstacle_detected: bool
    distance_mm: int | None
    light_raw: int
    light_delta: int
    target_active: bool


class ArduinoSensorInterpreter:
    """Apply explicit polarity, thresholds, debounce, and hysteresis."""

    __slots__ = (
        "_activation_count",
        "_ambient_light_raw",
        "_frontal_obstacle_distance_mm",
        "_release_count",
        "_required_stable_readings",
        "_target_activation_delta",
        "_target_active",
        "_target_release_delta",
    )

    def __init__(
        self,
        *,
        ambient_light_raw: int,
        frontal_obstacle_distance_mm: int = 200,
        target_activation_delta: int = 50,
        target_release_delta: int = 30,
        required_stable_readings: int = 5,
    ) -> None:
        """Validate and retain one deterministic sensor policy."""

        self._ambient_light_raw = _require_bounded_integer(
            ambient_light_raw,
            field_name="ambient_light_raw",
            minimum=0,
            maximum=MAX_ANALOG_RAW_VALUE,
        )
        self._frontal_obstacle_distance_mm = (
            _require_positive_integer(
                frontal_obstacle_distance_mm,
                field_name="frontal_obstacle_distance_mm",
            )
        )
        self._target_activation_delta = (
            _require_bounded_integer(
                target_activation_delta,
                field_name="target_activation_delta",
                minimum=1,
                maximum=MAX_ANALOG_RAW_VALUE,
            )
        )
        self._target_release_delta = _require_bounded_integer(
            target_release_delta,
            field_name="target_release_delta",
            minimum=0,
            maximum=MAX_ANALOG_RAW_VALUE,
        )
        self._required_stable_readings = (
            _require_positive_integer(
                required_stable_readings,
                field_name="required_stable_readings",
            )
        )

        if (
            self._target_release_delta
            >= self._target_activation_delta
        ):
            raise DomainValidationError(
                "target_release_delta must be lower than "
                "target_activation_delta."
            )

        self._activation_count = 0
        self._release_count = 0
        self._target_active = False

    @property
    def ambient_light_raw(self) -> int:
        """Return the current tracked ambient-light baseline."""

        return self._ambient_light_raw

    @property
    def target_active(self) -> bool:
        """Return the current debounced target-light state."""

        return self._target_active

    def interpret(
        self,
        snapshot: ArduinoSensorSnapshot,
    ) -> ArduinoSensorInterpretation:
        """Translate one raw snapshot without causing physical action."""

        if not isinstance(snapshot, ArduinoSensorSnapshot):
            raise DomainValidationError(
                "snapshot must be an ArduinoSensorSnapshot."
            )

        light_delta = (
            snapshot.light_raw - self._ambient_light_raw
        )
        self._update_target_state(light_delta)
        self._track_ambient_light(light_delta)

        distance_mm = snapshot.distance_mm
        frontal_obstacle_detected = (
            distance_mm is not None
            and distance_mm
            <= self._frontal_obstacle_distance_mm
        )

        return ArduinoSensorInterpretation(
            sequence_id=snapshot.sequence_id,
            left_obstacle_detected=(
                snapshot.left_ir_raw == 0
            ),
            right_obstacle_detected=(
                snapshot.right_ir_raw == 0
            ),
            boundary_detected=(snapshot.line_raw == 1),
            frontal_obstacle_detected=(
                frontal_obstacle_detected
            ),
            distance_mm=distance_mm,
            light_raw=snapshot.light_raw,
            light_delta=light_delta,
            target_active=self._target_active,
        )

    def _track_ambient_light(
        self,
        light_delta: int,
    ) -> None:
        """Follow small ambient drift without absorbing a target."""

        if (
            self._target_active
            or self._activation_count > 0
            or abs(light_delta)
            > self._target_release_delta
        ):
            return

        if light_delta > 0:
            self._ambient_light_raw += 1
        elif light_delta < 0:
            self._ambient_light_raw -= 1

    def _update_target_state(
        self,
        light_delta: int,
    ) -> None:
        """Update target state through debounce and hysteresis."""

        if self._target_active:
            self._activation_count = 0

            if light_delta <= self._target_release_delta:
                self._release_count += 1
            else:
                self._release_count = 0

            if (
                self._release_count
                >= self._required_stable_readings
            ):
                self._target_active = False
                self._release_count = 0

            return

        self._release_count = 0

        if light_delta >= self._target_activation_delta:
            self._activation_count += 1
        else:
            self._activation_count = 0

        if (
            self._activation_count
            >= self._required_stable_readings
        ):
            self._target_active = True
            self._activation_count = 0


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