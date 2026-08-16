"""Externally controlled deterministic GridWorld perception."""

from ai_logistics_robot.adapters.simulation.grid_world import GridWorld
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.perception import (
    Observation,
    PerceptionSnapshot,
)
from ai_logistics_robot.ports.clock_port import ClockPort


class GridWorldPerception:
    """Produce normalized snapshots from confirmed simulation state."""

    __slots__ = (
        "_clock",
        "_hazard_detected",
        "_observations",
        "_robot_id",
        "_simulation",
        "_target_active",
    )

    def __init__(
        self,
        *,
        robot_id: str,
        simulation: GridWorld,
        clock: ClockPort,
    ) -> None:
        """Validate dependencies and initialize safe external inputs."""

        if not isinstance(robot_id, str) or not robot_id.strip():
            raise DomainValidationError(
                "robot_id must be a non-empty string."
            )

        if not isinstance(simulation, GridWorld):
            raise DomainValidationError(
                "simulation must be a GridWorld instance."
            )

        if not isinstance(clock, ClockPort):
            raise DomainValidationError(
                "clock must satisfy ClockPort."
            )

        self._robot_id = robot_id
        self._simulation = simulation
        self._clock = clock
        self._observations: tuple[Observation, ...] = ()
        self._target_active = False
        self._hazard_detected = False

    def set_target_active(
        self,
        active: bool,
    ) -> None:
        """Set the externally controlled simulated target signal."""

        if not isinstance(active, bool):
            raise DomainValidationError(
                "active must be a boolean."
            )

        self._target_active = active

    def set_hazard_detected(
        self,
        detected: bool,
    ) -> None:
        """Set the externally controlled simulated hazard signal."""

        if not isinstance(detected, bool):
            raise DomainValidationError(
                "detected must be a boolean."
            )

        self._hazard_detected = detected

    def set_observations(
        self,
        observations: tuple[Observation, ...],
    ) -> None:
        """Replace the immutable simulated observation set."""

        if not isinstance(observations, tuple):
            raise DomainValidationError(
                "observations must be an immutable tuple."
            )

        if not all(
            isinstance(observation, Observation)
            for observation in observations
        ):
            raise DomainValidationError(
                "every observation must be an Observation instance."
            )

        self._observations = observations

    def observe(self) -> PerceptionSnapshot:
        """Return one passive immutable simulation snapshot."""

        return PerceptionSnapshot(
            robot_id=self._robot_id,
            captured_at=self._clock.now(),
            robot_pose=self._simulation.current_pose,
            observations=self._observations,
            target_active=self._target_active,
            hazard_detected=self._hazard_detected,
        )
