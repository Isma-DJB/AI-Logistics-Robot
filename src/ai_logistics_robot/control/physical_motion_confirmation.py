"""Confirm physical motion through exact domain-level robot poses."""

from ai_logistics_robot.domain.enums import CommandType, Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position, RobotPose

_CARDINAL_HEADINGS = (
    Heading.NORTH,
    Heading.EAST,
    Heading.SOUTH,
    Heading.WEST,
)



def required_heading_for_transition(
    *,
    position_before: Position,
    position_after: Position,
) -> Heading:
    """Return the heading required by one adjacent transition."""

    if not isinstance(position_before, Position):
        raise DomainValidationError(
            "position_before must be a Position instance."
        )

    if not isinstance(position_after, Position):
        raise DomainValidationError(
            "position_after must be a Position instance."
        )

    displacement = (
        position_after.x - position_before.x,
        position_after.y - position_before.y,
    )
    heading_by_displacement = {
        (0, 1): Heading.NORTH,
        (1, 0): Heading.EAST,
        (0, -1): Heading.SOUTH,
        (-1, 0): Heading.WEST,
    }

    try:
        return heading_by_displacement[displacement]
    except KeyError as error:
        raise DomainValidationError(
            "positions must describe one cardinal adjacent "
            "transition."
        ) from error


def expected_pose_after_command(
    *,
    pose_before: RobotPose,
    command_type: CommandType,
) -> RobotPose:
    """Return the exact domain pose required after one command."""

    if not isinstance(pose_before, RobotPose):
        raise DomainValidationError(
            "pose_before must be a RobotPose instance."
        )

    if not isinstance(command_type, CommandType):
        raise DomainValidationError(
            "command_type must be a CommandType instance."
        )

    if command_type is CommandType.STOP:
        return pose_before

    if command_type is CommandType.TURN_LEFT:
        return RobotPose(
            position=pose_before.position,
            heading=_rotate_heading(
                pose_before.heading,
                quarter_turns=-1,
            ),
        )

    if command_type is CommandType.TURN_RIGHT:
        return RobotPose(
            position=pose_before.position,
            heading=_rotate_heading(
                pose_before.heading,
                quarter_turns=1,
            ),
        )

    if command_type is CommandType.MOVE_FORWARD:
        return RobotPose(
            position=_forward_position(pose_before),
            heading=pose_before.heading,
        )

    raise DomainValidationError(
        "command_type is not supported for physical confirmation."
    )


def pose_confirms_command(
    *,
    pose_before: RobotPose,
    command_type: CommandType,
    observed_pose: RobotPose,
) -> bool:
    """Return whether camera pose exactly confirms the command."""

    if not isinstance(observed_pose, RobotPose):
        raise DomainValidationError(
            "observed_pose must be a RobotPose instance."
        )

    return observed_pose == expected_pose_after_command(
        pose_before=pose_before,
        command_type=command_type,
    )


def _rotate_heading(
    heading: Heading,
    *,
    quarter_turns: int,
) -> Heading:
    """Rotate one cardinal heading by signed quarter turns."""

    current_index = _CARDINAL_HEADINGS.index(heading)
    next_index = (
        current_index + quarter_turns
    ) % len(_CARDINAL_HEADINGS)

    return _CARDINAL_HEADINGS[next_index]


def _forward_position(pose: RobotPose) -> Position:
    """Return the adjacent cell directly ahead of one pose."""

    if pose.heading is Heading.NORTH:
        displacement = (0, 1)
    elif pose.heading is Heading.EAST:
        displacement = (1, 0)
    elif pose.heading is Heading.SOUTH:
        displacement = (0, -1)
    elif pose.heading is Heading.WEST:
        displacement = (-1, 0)
    else:
        raise DomainValidationError(
            "pose heading is not supported."
        )

    return Position(
        x=pose.position.x + displacement[0],
        y=pose.position.y + displacement[1],
    )
