"""Unit tests for explicit assisted obstacle review."""

import unittest

from ai_logistics_robot.control.assisted_obstacle_review import (
    review_obstacle_proposals,
)
from ai_logistics_robot.control.physical_obstacle_evidence import (
    ObstacleEvidence,
    ObstacleEvidenceSource,
    ObstacleEvidenceStatus,
)
from ai_logistics_robot.domain.errors import (
    DomainValidationError,
    InvariantViolationError,
)
from ai_logistics_robot.domain.geometry import Position


def _proposal(
    x_coordinate: int,
    y_coordinate: int,
    confidence: float = 0.5,
) -> ObstacleEvidence:
    """Build one valid camera proposal."""

    return ObstacleEvidence(
        position=Position(
            x=x_coordinate,
            y=y_coordinate,
        ),
        source=ObstacleEvidenceSource.CAMERA,
        status=ObstacleEvidenceStatus.DETECTED,
        confidence=confidence,
    )


class AssistedObstacleReviewTests(unittest.TestCase):
    """Verify complete explicit review of camera proposals."""

    def test_confirmed_and_cleared_decisions_are_applied(
        self,
    ) -> None:
        confirmed = Position(x=1, y=2)
        cleared_shadow = Position(x=1, y=3)
        cleared_robot_shadow = Position(x=3, y=4)

        reviewed = review_obstacle_proposals(
            proposals=(
                _proposal(1, 2, 0.421),
                _proposal(1, 3, 0.278),
                _proposal(3, 4, 0.449),
            ),
            confirmed_positions=frozenset(
                {
                    confirmed,
                }
            ),
            cleared_positions=frozenset(
                {
                    cleared_shadow,
                    cleared_robot_shadow,
                }
            ),
        )

        self.assertEqual(
            tuple(
                item.position
                for item in reviewed
            ),
            (
                confirmed,
                cleared_shadow,
                cleared_robot_shadow,
            ),
        )
        self.assertEqual(
            tuple(
                item.source
                for item in reviewed
            ),
            (
                ObstacleEvidenceSource.OPERATOR,
                ObstacleEvidenceSource.OPERATOR,
                ObstacleEvidenceSource.OPERATOR,
            ),
        )
        self.assertEqual(
            tuple(
                item.status
                for item in reviewed
            ),
            (
                ObstacleEvidenceStatus.CONFIRMED,
                ObstacleEvidenceStatus.CLEARED,
                ObstacleEvidenceStatus.CLEARED,
            ),
        )
        self.assertEqual(
            tuple(
                item.confidence
                for item in reviewed
            ),
            (
                0.421,
                0.278,
                0.449,
            ),
        )

    def test_empty_review_is_valid(self) -> None:
        self.assertEqual(
            review_obstacle_proposals(
                proposals=(),
                confirmed_positions=frozenset(),
                cleared_positions=frozenset(),
            ),
            (),
        )

    def test_unreviewed_proposal_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            InvariantViolationError,
            "every camera proposal",
        ):
            review_obstacle_proposals(
                proposals=(
                    _proposal(1, 2),
                    _proposal(1, 3),
                ),
                confirmed_positions=frozenset(
                    {
                        Position(x=1, y=2),
                    }
                ),
                cleared_positions=frozenset(),
            )

    def test_unknown_operator_decision_is_rejected(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            InvariantViolationError,
            "must reference camera proposals",
        ):
            review_obstacle_proposals(
                proposals=(
                    _proposal(1, 2),
                ),
                confirmed_positions=frozenset(
                    {
                        Position(x=4, y=4),
                    }
                ),
                cleared_positions=frozenset(
                    {
                        Position(x=1, y=2),
                    }
                ),
            )

    def test_overlapping_decisions_are_rejected(self) -> None:
        position = Position(x=1, y=2)

        with self.assertRaisesRegex(
            InvariantViolationError,
            "must be disjoint",
        ):
            review_obstacle_proposals(
                proposals=(
                    _proposal(1, 2),
                ),
                confirmed_positions=frozenset(
                    {
                        position,
                    }
                ),
                cleared_positions=frozenset(
                    {
                        position,
                    }
                ),
            )

    def test_only_camera_detected_evidence_is_accepted(
        self,
    ) -> None:
        invalid_items = (
            ObstacleEvidence(
                position=Position(x=1, y=2),
                source=ObstacleEvidenceSource.OPERATOR,
                status=ObstacleEvidenceStatus.CONFIRMED,
                confidence=1.0,
            ),
            ObstacleEvidence(
                position=Position(x=1, y=2),
                source=ObstacleEvidenceSource.OPERATOR,
                status=ObstacleEvidenceStatus.CLEARED,
                confidence=1.0,
            ),
        )

        for invalid_item in invalid_items:
            with self.subTest(
                invalid_item=invalid_item,
            ):
                with self.assertRaisesRegex(
                    InvariantViolationError,
                    "CAMERA/DETECTED",
                ):
                    review_obstacle_proposals(
                        proposals=(invalid_item,),
                        confirmed_positions=frozenset(
                            {
                                invalid_item.position,
                            }
                        ),
                        cleared_positions=frozenset(),
                    )

    def test_duplicate_proposal_positions_are_rejected(
        self,
    ) -> None:
        position = Position(x=1, y=2)

        with self.assertRaisesRegex(
            InvariantViolationError,
            "must be unique",
        ):
            review_obstacle_proposals(
                proposals=(
                    _proposal(1, 2, 0.4),
                    _proposal(1, 2, 0.8),
                ),
                confirmed_positions=frozenset(
                    {
                        position,
                    }
                ),
                cleared_positions=frozenset(),
            )

    def test_proposals_must_be_an_immutable_tuple(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "proposals must be a tuple",
        ):
            review_obstacle_proposals(
                proposals=[_proposal(1, 2)],  # type: ignore[arg-type]
                confirmed_positions=frozenset(
                    {
                        Position(x=1, y=2),
                    }
                ),
                cleared_positions=frozenset(),
            )

    def test_proposal_items_are_validated(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "ObstacleEvidence",
        ):
            review_obstacle_proposals(
                proposals=("invalid",),  # type: ignore[arg-type]
                confirmed_positions=frozenset(),
                cleared_positions=frozenset(),
            )

    def test_decision_sets_must_be_frozensets(
        self,
    ) -> None:
        proposal = _proposal(1, 2)

        with self.assertRaisesRegex(
            DomainValidationError,
            "confirmed_positions must be a frozenset",
        ):
            review_obstacle_proposals(
                proposals=(proposal,),
                confirmed_positions={  # type: ignore[arg-type]
                    proposal.position,
                },
                cleared_positions=frozenset(),
            )

        with self.assertRaisesRegex(
            DomainValidationError,
            "cleared_positions must be a frozenset",
        ):
            review_obstacle_proposals(
                proposals=(proposal,),
                confirmed_positions=frozenset(),
                cleared_positions={  # type: ignore[arg-type]
                    proposal.position,
                },
            )

    def test_decision_items_must_be_positions(self) -> None:
        with self.assertRaisesRegex(
            DomainValidationError,
            "every confirmed_positions item",
        ):
            review_obstacle_proposals(
                proposals=(),
                confirmed_positions=frozenset(
                    {
                        "invalid",  # type: ignore[arg-type]
                    }
                ),
                cleared_positions=frozenset(),
            )

        with self.assertRaisesRegex(
            DomainValidationError,
            "every cleared_positions item",
        ):
            review_obstacle_proposals(
                proposals=(),
                confirmed_positions=frozenset(),
                cleared_positions=frozenset(
                    {
                        "invalid",  # type: ignore[arg-type]
                    }
                ),
            )


if __name__ == "__main__":
    unittest.main()