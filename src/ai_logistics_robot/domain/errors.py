"""Exception hierarchy for platform-independent domain failures."""


class DomainError(Exception):
    """Base exception for all domain-level errors."""


class DomainValidationError(DomainError):
    """Raised when a domain object receives invalid data."""


class InvalidCoordinateError(DomainValidationError):
    """Raised when a coordinate is not represented by an integer."""


class InvalidStateTransitionError(DomainError):
    """Raised when a requested state transition is forbidden."""


class InvariantViolationError(DomainError):
    """Raised when an established domain invariant is violated."""

class NoPathError(DomainError):
    """Raised when planning cannot reach any authorized goal."""
