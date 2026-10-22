"""Fuse Arduino sensor state with calibrated physical-grid perception."""

from collections.abc import Callable
from typing import Protocol, runtime_checkable

from ai_logistics_robot.adapters.hardware.arduino_sensor_interpreter import (
    ArduinoSensorInterpretation,
    ArduinoSensorInterpreter,
)
from ai_logistics_robot.adapters.hardware.arduino_serial_client import (
    ArduinoSensorSnapshot,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.perception import PerceptionSnapshot

SensorReader = Callable[[], ArduinoSensorSnapshot]


@runtime_checkable
class MutablePhysicalGridPerception(Protocol):
    """Physical perception surface accepting confirmed external state."""

    def set_target_active(
        self,
        target_active: bool,
    ) -> None:
        """Set the current confirmed target-light state."""

        ...

    def set_hazard_detected(
        self,
        hazard_detected: bool,
    ) -> None:
        """Set the current confirmed critical-hazard state."""

        ...

    def observe(self) -> PerceptionSnapshot:
        """Return one immutable physical perception snapshot."""

        ...


class SensorFusedGridPerception:
    """Fuse read-only Arduino sensors with visual robot localization."""

    __slots__ = (
        "_interpreter",
        "_last_interpretation",
        "_perception",
        "_sensor_reader",
    )

    def __init__(
        self,
        *,
        perception: MutablePhysicalGridPerception,
        sensor_reader: SensorReader,
        interpreter: ArduinoSensorInterpreter,
    ) -> None:
        """Validate and retain the composed perception dependencies."""

        if not isinstance(
            perception,
            MutablePhysicalGridPerception,
        ):
            raise DomainValidationError(
                "perception must support mutable physical "
                "grid perception."
            )

        if not callable(sensor_reader):
            raise DomainValidationError(
                "sensor_reader must be callable."
            )

        if not isinstance(
            interpreter,
            ArduinoSensorInterpreter,
        ):
            raise DomainValidationError(
                "interpreter must be an "
                "ArduinoSensorInterpreter."
            )

        self._perception = perception
        self._sensor_reader = sensor_reader
        self._interpreter = interpreter
        self._last_interpretation: (
            ArduinoSensorInterpretation | None
        ) = None

    @property
    def perception(
        self,
    ) -> MutablePhysicalGridPerception:
        """Return the wrapped visual-grid perception adapter."""

        return self._perception

    @property
    def interpreter(self) -> ArduinoSensorInterpreter:
        """Return the configured sensor interpreter."""

        return self._interpreter

    @property
    def last_interpretation(
        self,
    ) -> ArduinoSensorInterpretation | None:
        """Return the latest successfully fused sensor state."""

        return self._last_interpretation

    def observe(self) -> PerceptionSnapshot:
        """Read sensors, update safe state, and localize the robot."""

        raw_snapshot = self._sensor_reader()
        interpretation = self._interpreter.interpret(
            raw_snapshot
        )

        self._perception.set_target_active(
            interpretation.target_active
        )
        self._perception.set_hazard_detected(
            interpretation.boundary_detected
        )

        snapshot = self._perception.observe()

        if not isinstance(snapshot, PerceptionSnapshot):
            raise DomainValidationError(
                "perception must return a PerceptionSnapshot."
            )

        self._last_interpretation = interpretation
        return snapshot