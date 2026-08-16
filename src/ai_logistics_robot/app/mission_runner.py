"""Deterministic application coordination without domain decisions."""

from ai_logistics_robot.app.settings import Settings
from ai_logistics_robot.domain.enums import BrainState, FailureReason
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvalidStateTransitionError,
)
from ai_logistics_robot.domain.events import MissionEvent
from ai_logistics_robot.domain.safety import SafetyStatus
from ai_logistics_robot.domain.status import SystemStatus
from ai_logistics_robot.domain.world import GridMap
from ai_logistics_robot.ports import (
    BrainPort,
    ControlPort,
    MonitoringPort,
    RendererPort,
    SimulationPort,
)


class MissionRunner:
    """Coordinate one configured application execution loop."""

    __slots__ = (
        "_brain",
        "_configured",
        "_control",
        "_displayed_sequences",
        "_monitoring",
        "_renderer",
        "_running",
        "_settings",
        "_simulation",
    )

    def __init__(
        self,
        *,
        brain: BrainPort,
        control: ControlPort,
        simulation: SimulationPort,
        monitoring: MonitoringPort,
        renderer: RendererPort,
    ) -> None:
        """Validate and retain public application dependencies."""

        if not isinstance(brain, BrainPort):
            raise DomainValidationError(
                "brain must satisfy BrainPort."
            )

        if not isinstance(control, ControlPort):
            raise DomainValidationError(
                "control must satisfy ControlPort."
            )

        if not isinstance(simulation, SimulationPort):
            raise DomainValidationError(
                "simulation must satisfy SimulationPort."
            )

        if not isinstance(monitoring, MonitoringPort):
            raise DomainValidationError(
                "monitoring must satisfy MonitoringPort."
            )

        if not isinstance(renderer, RendererPort):
            raise DomainValidationError(
                "renderer must satisfy RendererPort."
            )

        self._brain = brain
        self._control = control
        self._simulation = simulation
        self._monitoring = monitoring
        self._renderer = renderer
        self._settings: Settings | None = None
        self._configured = False
        self._running = False
        self._displayed_sequences: dict[str, int] = {}

    @property
    def configured(self) -> bool:
        """Return whether validated settings were accepted."""

        return self._configured

    @property
    def running(self) -> bool:
        """Return whether one start invocation is active."""

        return self._running

    def configure(
        self,
        settings: Settings,
    ) -> None:
        """Validate and retain one complete application configuration."""

        if not isinstance(settings, Settings):
            raise DomainValidationError(
                "settings must be a Settings instance."
            )

        if self._running:
            raise InvalidStateTransitionError(
                "the runner cannot be configured while active."
            )

        world = self._read_world()

        if world != settings.grid_map:
            raise DomainValidationError(
                "the configured grid map must match the "
                "active simulation world."
            )

        status = self._read_status()

        if status.robot_id != settings.robot.robot_id:
            raise DomainValidationError(
                "the Brain robot_id must match the "
                "configured robot_id."
            )

        if status.robot_pose != settings.robot.initial_pose:
            raise DomainValidationError(
                "the confirmed Brain pose must equal "
                "the configured initial pose."
            )

        self._settings = settings
        self._configured = True

    def start(
        self,
        maximum_cycles: int | None = None,
    ) -> SystemStatus:
        """Run deterministic cycles until one invocation guard stops."""

        self._validate_maximum_cycles(maximum_cycles)
        self._require_settings()

        if self._running:
            raise InvalidStateTransitionError(
                "the runner is already active."
            )

        status = self._read_status()

        if (
            status.brain_state is BrainState.SAFETY_STOP
            or status.safety_status.latched
        ):
            raise InvalidStateTransitionError(
                "a safety-stopped application must be "
                "rearmed and reset before starting."
            )

        completed_cycles = 0
        self._running = True

        try:
            while (
                self._running
                and (
                    maximum_cycles is None
                    or completed_cycles < maximum_cycles
                )
            ):
                self._brain.update()
                status = self._read_status()

                self._display_new_events(status)
                self._renderer.render(
                    self._read_world(),
                    status,
                )

                completed_cycles += 1

                if (
                    status.brain_state
                    is BrainState.SAFETY_STOP
                    or status.safety_status.latched
                ):
                    self._running = False
        finally:
            self._running = False

        return status

    def stop(self) -> None:
        """Request one normal controlled stop and halt the loop."""

        self._control.stop()
        self._running = False

    def request_emergency_stop(
        self,
        reason: FailureReason,
    ) -> SystemStatus:
        """Latch Control before allowing Brain to observe safety."""

        if not isinstance(reason, FailureReason):
            raise DomainValidationError(
                "reason must be a FailureReason instance."
            )

        self._require_settings()
        self._running = False

        safety_status = self._control.emergency_stop(
            reason
        )

        if not isinstance(safety_status, SafetyStatus):
            raise DomainValidationError(
                "Control must return a SafetyStatus "
                "after emergency stop."
            )

        self._brain.update()
        status = self._read_status()

        self._display_new_events(status)
        self._renderer.render(
            self._read_world(),
            status,
        )

        return status

    def request_safety_rearm(self) -> SafetyStatus:
        """Perform an explicit manual rearm without resetting."""

        self._require_settings()

        if self._running:
            raise InvalidStateTransitionError(
                "safety cannot be rearmed while the "
                "runner is active."
            )

        status = self._read_status()
        safety_status = self._read_safety_status()

        if (
            status.brain_state is not BrainState.SAFETY_STOP
            or not safety_status.latched
        ):
            raise InvalidStateTransitionError(
                "safety rearm requires a latched "
                "safety-stop state."
            )

        rearmed = self._control.reset_safety_latch()

        if not isinstance(rearmed, SafetyStatus):
            raise DomainValidationError(
                "Control must return a SafetyStatus "
                "after safety rearm."
            )

        return rearmed

    def reset(self) -> None:
        """Reset temporary state only from a confirmed safe pose."""

        settings = self._require_settings()

        if self._running:
            raise InvalidStateTransitionError(
                "the runner cannot be reset while active."
            )

        status = self._read_status()
        safety_status = self._read_safety_status()
        initial_pose = settings.robot.initial_pose

        if safety_status.latched:
            raise InvalidStateTransitionError(
                "the safety latch must be rearmed "
                "before reset."
            )

        if status.robot_pose != initial_pose:
            raise InvalidStateTransitionError(
                "the confirmed Brain pose must equal "
                "the configured initial pose before reset."
            )

        self._simulation.reset()
        self._brain.reset()

        reset_status = self._read_status()

        if reset_status.robot_pose != initial_pose:
            raise InvalidStateTransitionError(
                "reset did not preserve the configured "
                "confirmed initial pose."
            )

    def get_status(self) -> SystemStatus:
        """Return the current immutable Brain status."""

        return self._read_status()

    @staticmethod
    def _validate_maximum_cycles(
        maximum_cycles: int | None,
    ) -> None:
        """Require an optional positive non-boolean cycle bound."""

        if maximum_cycles is None:
            return

        if (
            isinstance(maximum_cycles, bool)
            or not isinstance(maximum_cycles, int)
            or maximum_cycles <= 0
        ):
            raise DomainValidationError(
                "maximum_cycles must be a positive "
                "integer or None."
            )

    def _require_settings(self) -> Settings:
        """Return configured settings or reject the operation."""

        if not self._configured or self._settings is None:
            raise InvalidStateTransitionError(
                "the runner must be configured first."
            )

        return self._settings

    def _read_status(self) -> SystemStatus:
        """Read and validate one immutable Brain status."""

        status = self._brain.get_status()

        if not isinstance(status, SystemStatus):
            raise DomainValidationError(
                "Brain must return a SystemStatus instance."
            )

        return status

    def _read_safety_status(self) -> SafetyStatus:
        """Read and validate the current Control safety status."""

        safety_status = self._control.get_safety_status()

        if not isinstance(safety_status, SafetyStatus):
            raise DomainValidationError(
                "Control must return a SafetyStatus instance."
            )

        return safety_status

    def _read_world(self) -> GridMap:
        """Read and validate the immutable active world."""

        world = self._simulation.read_world()

        if not isinstance(world, GridMap):
            raise DomainValidationError(
                "Simulation must return a GridMap instance."
            )

        return world


    def _display_new_events(
        self,
        status: SystemStatus,
    ) -> None:
        """Forward new mission events once in sequence order."""

        mission_id = status.mission_id

        if mission_id is None:
            return

        events = self._monitoring.events_for(
            mission_id
        )

        if not isinstance(events, tuple):
            raise DomainValidationError(
                "Monitoring must return an immutable event tuple."
            )

        last_sequence = self._displayed_sequences.get(
            mission_id,
            0,
        )

        for event in events:
            if not isinstance(event, MissionEvent):
                raise DomainValidationError(
                    "Monitoring history must contain "
                    "MissionEvent instances."
                )

            if event.sequence_number <= last_sequence:
                continue

            self._renderer.display_event(event)
            last_sequence = event.sequence_number
            self._displayed_sequences[mission_id] = (
                last_sequence
            )
