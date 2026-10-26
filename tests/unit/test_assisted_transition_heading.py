"""Unit tests for assisted transition heading derivation."""

import unittest

from ai_logistics_robot.control.physical_motion_confirmation import (
    required_heading_for_transition,
)
from ai_logistics_robot.domain.enums import Heading
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.domain.geometry import Position


class AssistedTransitionHeadingTests(unittest.TestCase):
    """Verify logical-grid cardinal transition headings."""

    def test_cardinal_adjacent_transitions(self) -> None:
        cases = (
            (
                Position(x=3, y=3),
                Position(x=3, y=4),
                Heading.NORTH,
            ),
            (
                Position(x=3, y=3),
                Position(x=4, y=3),
                Heading.EAST,
            ),
            (
                Position(x=3, y=3),
                Position(x=3, y=2),
                Heading.SOUTH,
            ),
            (
                Position(x=3, y=3),
                Position(x=2, y=3),
                Heading.WEST,
            ),
        )

        for before, after, expected in cases:
            with self.subTest(
                before=before,
                after=after,
            ):
                self.assertEqual(
                    required_heading_for_transition(
                        position_before=before,
                        position_after=after,
                    ),
                    expected,
                )

    def test_non_adjacent_transitions_are_rejected(
        self,
    ) -> None:
        invalid_after_positions = (
            Position(x=3, y=3),
            Position(x=4, y=4),
            Position(x=3, y=5),
            Position(x=1, y=3),
        )

        for position_after in invalid_after_positions:
            with self.subTest(
                position_after=position_after,
            ):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "cardinal adjacent",
                ):
                    required_heading_for_transition(
                        position_before=Position(
                            x=3,
                            y=3,
                        ),
                        position_after=position_after,
                    )

    def test_position_types_are_validated(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "position_before",
        ):
            required_heading_for_transition(
                position_before="invalid",  # type: ignore[arg-type]
                position_after=Position(x=3, y=4),
            )

        with self.assertRaisesRegex(
            DomainValidationError,
            "position_after",
        ):
            required_heading_for_transition(
                position_before=Position(x=3, y=3),
                position_after="invalid",  # type: ignore[arg-type]
            )


if __name__ == "__main__":
    unittest.main()
