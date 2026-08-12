"""Platform-independent enumerations for the V1 domain model."""

from enum import StrEnum, unique


@unique
class BrainState(StrEnum):
    """Execution states controlled exclusively by the Brain."""

    INITIALIZATION = "INITIALIZATION"
    WAITING_FOR_MISSION = "WAITING_FOR_MISSION"
    OUTBOUND_PLANNING = "OUTBOUND_PLANNING"
    OUTBOUND_NAVIGATION = "OUTBOUND_NAVIGATION"
    OUTBOUND_REPLANNING = "OUTBOUND_REPLANNING"
    COLLECTION = "COLLECTION"
    RETURN_PREPARATION = "RETURN_PREPARATION"
    RETURN_NAVIGATION = "RETURN_NAVIGATION"
    RETURN_REPLANNING = "RETURN_REPLANNING"
    MISSION_COMPLETED = "MISSION_COMPLETED"
    MISSION_FAILED = "MISSION_FAILED"
    SYSTEM_ERROR = "SYSTEM_ERROR"
    SAFETY_STOP = "SAFETY_STOP"


@unique
class Heading(StrEnum):
    """Cardinal heading of the robot."""

    NORTH = "NORTH"
    EAST = "EAST"
    SOUTH = "SOUTH"
    WEST = "WEST"


@unique
class MissionStatus(StrEnum):
    """Lifecycle and final outcome of a mission."""

    CREATED = "CREATED"
    ACTIVE = "ACTIVE"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


@unique
class PathPhase(StrEnum):
    """Mission phase associated with a path."""

    OUTBOUND = "OUTBOUND"
    RETURN = "RETURN"
    DETOUR = "DETOUR"


@unique
class CommandType(StrEnum):
    """Normal motion commands accepted by Control."""

    MOVE_FORWARD = "MOVE_FORWARD"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    STOP = "STOP"


@unique
class CommandStatus(StrEnum):
    """Outcome of a motion-command execution."""

    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    ABORTED = "ABORTED"
    TIMEOUT = "TIMEOUT"


@unique
class SafetySeverity(StrEnum):
    """Severity assigned to a safety event."""

    INFO = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


@unique
class FailureReason(StrEnum):
    """Normalized failure reasons used across the V1 domain."""

    BLOCKED = "BLOCKED"
    NO_PATH = "NO_PATH"
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"
    SAFETY_LATCHED = "SAFETY_LATCHED"
    TIMEOUT = "TIMEOUT"
    COMMUNICATION_LOSS = "COMMUNICATION_LOSS"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    INTERNAL_ERROR = "INTERNAL_ERROR"