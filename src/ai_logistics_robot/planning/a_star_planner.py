"""Deterministic A* planning over immutable grid maps."""

from heapq import heappop, heappush
from itertools import count

from ai_logistics_robot.domain.enums import PathPhase
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    NoPathError,
)
from ai_logistics_robot.domain.geometry import Position, RobotPose
from ai_logistics_robot.domain.paths import PathPlan
from ai_logistics_robot.domain.world import GridMap

_OpenEntry = tuple[int, int, int, Position]


def _validate_identifier(name: str, value: object) -> None:
    """Require a non-empty string identifier."""

    if not isinstance(value, str) or not value.strip():
        raise DomainValidationError(
            f"{name} must be a non-empty string."
        )


def _manhattan_distance(
    position: Position,
    goal: Position,
) -> int:
    """Return the cardinal lower-bound distance to one goal."""

    return (
        abs(position.x - goal.x)
        + abs(position.y - goal.y)
    )


def _reconstruct_path(
    predecessors: dict[Position, Position],
    start: Position,
    goal: Position,
) -> tuple[Position, ...]:
    """Reconstruct one discovered path from goal to start."""

    reversed_positions = [goal]

    while reversed_positions[-1] != start:
        reversed_positions.append(
            predecessors[reversed_positions[-1]]
        )

    reversed_positions.reverse()
    return tuple(reversed_positions)


def _find_path(
    *,
    start: Position,
    goal: Position,
    world: GridMap,
) -> tuple[Position, ...] | None:
    """Return the deterministic shortest path to one goal."""

    if start == goal:
        return (start,)

    insertion_order = count()
    frontier: list[_OpenEntry] = []
    costs: dict[Position, int] = {start: 0}
    predecessors: dict[Position, Position] = {}

    heappush(
        frontier,
        (
            _manhattan_distance(start, goal),
            0,
            next(insertion_order),
            start,
        ),
    )

    while frontier:
        _, current_cost, _, current = heappop(frontier)

        if current_cost != costs.get(current):
            continue

        if current == goal:
            return _reconstruct_path(
                predecessors,
                start,
                goal,
            )

        for neighbor in world.adjacent_positions(current):
            if not world.is_traversable(neighbor):
                continue

            candidate_cost = current_cost + 1
            known_cost = costs.get(neighbor)

            if (
                known_cost is not None
                and candidate_cost >= known_cost
            ):
                continue

            costs[neighbor] = candidate_cost
            predecessors[neighbor] = current

            heappush(
                frontier,
                (
                    candidate_cost
                    + _manhattan_distance(neighbor, goal),
                    candidate_cost,
                    next(insertion_order),
                    neighbor,
                ),
            )

    return None


class AStarPlanner:
    """Create deterministic shortest paths for one immutable world."""

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
        """Create a shortest plan to an authorized reachable goal."""

        self._validate_request(
            mission_id=mission_id,
            robot_id=robot_id,
            start_pose=start_pose,
            authorized_goals=authorized_goals,
            world=world,
            phase=phase,
            version=version,
        )

        best_path: tuple[Position, ...] | None = None
        best_goal: Position | None = None

        for goal in authorized_goals:
            candidate_path = _find_path(
                start=start_pose.position,
                goal=goal,
                world=world,
            )

            if candidate_path is None:
                continue

            if (
                best_path is None
                or len(candidate_path) < len(best_path)
            ):
                best_path = candidate_path
                best_goal = goal

        if best_path is None or best_goal is None:
            raise NoPathError(
                "no authorized goal is reachable "
                "from the start pose."
            )

        return PathPlan(
            mission_id=mission_id,
            robot_id=robot_id,
            phase=phase,
            version=version,
            positions=best_path,
            goal=best_goal,
        )

    @staticmethod
    def _validate_request(
        *,
        mission_id: object,
        robot_id: object,
        start_pose: object,
        authorized_goals: object,
        world: object,
        phase: object,
        version: object,
    ) -> None:
        """Validate one planning request before path calculation."""

        _validate_identifier("mission_id", mission_id)
        _validate_identifier("robot_id", robot_id)

        if not isinstance(start_pose, RobotPose):
            raise DomainValidationError(
                "start_pose must be a RobotPose instance."
            )

        if not isinstance(world, GridMap):
            raise DomainValidationError(
                "world must be a GridMap instance."
            )

        if not isinstance(phase, PathPhase):
            raise DomainValidationError(
                "phase must be a PathPhase instance."
            )

        if (
            isinstance(version, bool)
            or not isinstance(version, int)
            or version < 1
        ):
            raise DomainValidationError(
                "version must be a positive integer."
            )

        if not isinstance(authorized_goals, tuple):
            raise DomainValidationError(
                "authorized_goals must be an immutable tuple."
            )

        if not authorized_goals:
            raise DomainValidationError(
                "at least one authorized goal is required."
            )

        if not all(
            isinstance(goal, Position)
            for goal in authorized_goals
        ):
            raise DomainValidationError(
                "every authorized goal must be "
                "a Position instance."
            )

        if len(authorized_goals) != len(
            set(authorized_goals)
        ):
            raise DomainValidationError(
                "authorized goals must be unique."
            )

        if not world.is_traversable(start_pose.position):
            raise DomainValidationError(
                "start_pose must be traversable."
            )

        if not all(
            world.is_traversable(goal)
            for goal in authorized_goals
        ):
            raise DomainValidationError(
                "every authorized goal must be traversable."
            )
