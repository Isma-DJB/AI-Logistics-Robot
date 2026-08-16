"""Reference dependency assembly for deterministic simulation."""

from dataclasses import dataclass
from datetime import datetime

from ai_logistics_robot.adapters.monitoring import InMemoryMonitoring
from ai_logistics_robot.adapters.simulation import (
    GridWorld,
    GridWorldPerception,
    HeadlessRenderer,
    SimulatedClock,
)
from ai_logistics_robot.app.mission_runner import MissionRunner
from ai_logistics_robot.app.settings import Settings
from ai_logistics_robot.brain import DeterministicBrain
from ai_logistics_robot.control import SafeRobotControl
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.memory import InMemoryMissionMemory
from ai_logistics_robot.planning import AStarPlanner
from ai_logistics_robot.ports import RendererPort


@dataclass(frozen=True, slots=True)
class SimulationApplication:
    """Expose one complete isolated simulation assembly."""

    settings: Settings
    simulation: GridWorld
    clock: SimulatedClock
    perception: GridWorldPerception
    planning: AStarPlanner
    control: SafeRobotControl
    memory: InMemoryMissionMemory
    monitoring: InMemoryMonitoring
    brain: DeterministicBrain
    renderer: RendererPort
    runner: MissionRunner


def _build_renderer(
    settings: Settings,
) -> RendererPort:
    """Select one passive renderer without eager Pygame import."""

    if not settings.renderer.enabled:
        return HeadlessRenderer()

    from ai_logistics_robot.adapters.visualization import (
        PygameRenderer,
    )

    return PygameRenderer(
        settings=settings.renderer,
    )


def build_simulation_application(
    *,
    settings: Settings,
    epoch: datetime,
) -> SimulationApplication:
    """Assemble one validated deterministic simulation application."""

    if not isinstance(settings, Settings):
        raise DomainValidationError(
            "settings must be a Settings instance."
        )

    simulation = GridWorld(
        world=settings.grid_map,
        robot_id=settings.robot.robot_id,
        initial_pose=settings.robot.initial_pose,
    )
    clock = SimulatedClock(
        simulation=simulation,
        epoch=epoch,
    )
    perception = GridWorldPerception(
        robot_id=settings.robot.robot_id,
        simulation=simulation,
        clock=clock,
    )
    planning = AStarPlanner()
    control = SafeRobotControl(
        robot_id=settings.robot.robot_id,
        initial_pose=settings.robot.initial_pose,
        simulation=simulation,
        clock=clock,
    )
    memory = InMemoryMissionMemory()
    monitoring = InMemoryMonitoring()
    brain = DeterministicBrain(
        scenario_id=settings.scenario.scenario_id,
        robot_id=settings.robot.robot_id,
        target_id=settings.target.target_id,
        world=settings.grid_map,
        initial_pose=settings.robot.initial_pose,
        collection_duration_s=(
            settings.mission.collection_duration_s
        ),
        maximum_replans=(
            settings.mission.maximum_replans
        ),
        timeout_s=settings.mission.timeout_s,
        perception=perception,
        planning=planning,
        control=control,
        memory=memory,
        monitoring=monitoring,
        clock=clock,
    )
    renderer = _build_renderer(settings)
    runner = MissionRunner(
        brain=brain,
        control=control,
        simulation=simulation,
        monitoring=monitoring,
        renderer=renderer,
    )
    runner.configure(settings)

    return SimulationApplication(
        settings=settings,
        simulation=simulation,
        clock=clock,
        perception=perception,
        planning=planning,
        control=control,
        memory=memory,
        monitoring=monitoring,
        brain=brain,
        renderer=renderer,
        runner=runner,
    )
