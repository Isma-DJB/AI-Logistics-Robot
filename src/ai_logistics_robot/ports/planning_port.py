"""Public planning contract for versioned paths."""

from typing import Protocol, runtime_checkable

from ai_logistics_robot.domain.enums import PathPhase
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.world import GridMap


@runtime_checkable
class PlanningPort(Protocol):
    """Create a versioned path to one authorized goal."""

    def create_plan(
        self,
        *,
        mission_id: str,
        robot_id: str,
        start_pose: RobotPose,
        authorized_goals: tuple[Position, ...],
        world: GridMap,
        phase: PathPhase,
        version: int,
    ) -> PathPlan:
        """Return a versioned plan ending at an authorized position."""

        ...