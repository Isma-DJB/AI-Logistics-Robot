"""Unit tests for the domain-error hierarchy."""

import unittest

from ai_logistics_robot.domain.errors import (
    DomainError,
    DomainValidationError,
    InvalidCoordinateError,
    InvalidStateTransitionError,
    InvariantViolationError,
)


class DomainErrorTests(unittest.TestCase):
    """Verify that specialized errors remain domain errors."""

    def test_validation_errors_share_the_expected_hierarchy(self) -> None:
        self.assertTrue(issubclass(DomainValidationError, DomainError))
        self.assertTrue(
            issubclass(InvalidCoordinateError, DomainValidationError)
        )

    def test_behavior_errors_share_the_domain_base(self) -> None:
        self.assertTrue(
            issubclass(InvalidStateTransitionError, DomainError)
        )
        self.assertTrue(
            issubclass(InvariantViolationError, DomainError)
        )


if __name__ == "__main__":
    unittest.main()