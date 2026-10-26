"""Convert camera proposals into explicit operator decisions."""

from __future__ import annotations

from math import isfinite

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


def review_obstacle_proposals(
    *,
    proposals: tuple[ObstacleEvidence, ...],
    confirmed_positions: frozenset[Position],
    cleared_positions: frozenset[Position],
) -> tuple[ObstacleEvidence, ...]:
    """Require and apply one explicit decision for every proposal."""

    _validate_proposals(proposals)
    _validate_position_set(
        "confirmed_positions",
        confirmed_positions,
    )
    _validate_position_set(
        "cleared_positions",
        cleared_positions,
    )

    overlap = confirmed_positions & cleared_positions

    if overlap:
        raise InvariantViolationError(
            "confirmed and cleared positions must be disjoint."
        )

    proposal_positions = frozenset(
        proposal.position
        for proposal in proposals
    )
    reviewed_positions = (
        confirmed_positions
        | cleared_positions
    )

    unknown_positions = (
        reviewed_positions
        - proposal_positions
    )

    if unknown_positions:
        raise InvariantViolationError(
            "operator decisions must reference camera proposals."
        )

    unreviewed_positions = (
        proposal_positions
        - reviewed_positions
    )

    if unreviewed_positions:
        raise InvariantViolationError(
            "every camera proposal requires an explicit "
            "operator decision."
        )

    reviewed: list[ObstacleEvidence] = []

    for proposal in proposals:
        if proposal.position in confirmed_positions:
            status = ObstacleEvidenceStatus.CONFIRMED
        else:
            status = ObstacleEvidenceStatus.CLEARED

        reviewed.append(
            ObstacleEvidence(
                position=proposal.position,
                source=ObstacleEvidenceSource.OPERATOR,
                status=status,
                confidence=proposal.confidence,
            )
        )

    return tuple(reviewed)



def require_localized_camera_proposals(
    *,
    proposals: tuple[ObstacleEvidence, ...],
    observable_cell_count: int,
    maximum_changed_fraction: float = 0.25,
) -> None:
    """Reject scene-wide changes before operator review."""

    _validate_proposals(proposals)

    if (
        isinstance(observable_cell_count, bool)
        or not isinstance(observable_cell_count, int)
        or observable_cell_count <= 0
    ):
        raise DomainValidationError(
            "observable_cell_count must be a positive integer."
        )

    if len(proposals) > observable_cell_count:
        raise InvariantViolationError(
            "proposal count cannot exceed observable cells."
        )

    if (
        isinstance(maximum_changed_fraction, bool)
        or not isinstance(
            maximum_changed_fraction,
            (int, float),
        )
    ):
        raise DomainValidationError(
            "maximum_changed_fraction must be numeric."
        )

    normalized_fraction = float(
        maximum_changed_fraction
    )

    if (
        not isfinite(normalized_fraction)
        or normalized_fraction <= 0.0
        or normalized_fraction > 1.0
    ):
        raise DomainValidationError(
            "maximum_changed_fraction must be finite "
            "and within (0, 1]."
        )

    changed_fraction = (
        len(proposals)
        / observable_cell_count
    )

    if changed_fraction > normalized_fraction:
        raise InvariantViolationError(
            "camera proposals indicate scene-wide visual "
            "drift; capture a fresh same-state reference."
        )


def merge_persistent_obstacle_evidence(
    *,
    existing_confirmed_positions: frozenset[Position],
    reviewed_evidence: tuple[ObstacleEvidence, ...],
) -> tuple[ObstacleEvidence, ...]:
    """Carry prior obstacles unless explicitly reviewed again."""

    _validate_position_set(
        "existing_confirmed_positions",
        existing_confirmed_positions,
    )
    _validate_reviewed_evidence(reviewed_evidence)

    reviewed_positions = frozenset(
        item.position
        for item in reviewed_evidence
    )

    retained = tuple(
        ObstacleEvidence(
            position=position,
            source=ObstacleEvidenceSource.OPERATOR,
            status=ObstacleEvidenceStatus.CONFIRMED,
            confidence=1.0,
        )
        for position in sorted(
            (
                existing_confirmed_positions
                - reviewed_positions
            ),
            key=lambda value: (value.y, value.x),
        )
    )

    return retained + reviewed_evidence


def _validate_reviewed_evidence(
    reviewed_evidence: object,
) -> None:
    """Validate unique explicit operator decisions."""

    if not isinstance(reviewed_evidence, tuple):
        raise DomainValidationError(
            "reviewed_evidence must be a tuple."
        )

    positions: set[Position] = set()

    for item in reviewed_evidence:
        if not isinstance(item, ObstacleEvidence):
            raise DomainValidationError(
                "every reviewed item must be "
                "an ObstacleEvidence."
            )

        if (
            item.source
            is not ObstacleEvidenceSource.OPERATOR
            or item.status
            not in {
                ObstacleEvidenceStatus.CONFIRMED,
                ObstacleEvidenceStatus.CLEARED,
            }
        ):
            raise InvariantViolationError(
                "reviewed evidence must be an explicit "
                "OPERATOR decision."
            )

        if item.position in positions:
            raise InvariantViolationError(
                "reviewed evidence positions must be unique."
            )

        positions.add(item.position)


def _validate_proposals(
    proposals: object,
) -> None:
    """Validate immutable unique camera-detected proposals."""

    if not isinstance(proposals, tuple):
        raise DomainValidationError(
            "proposals must be a tuple."
        )

    positions: set[Position] = set()

    for proposal in proposals:
        if not isinstance(proposal, ObstacleEvidence):
            raise DomainValidationError(
                "every proposal must be an ObstacleEvidence."
            )

        if (
            proposal.source
            is not ObstacleEvidenceSource.CAMERA
            or proposal.status
            is not ObstacleEvidenceStatus.DETECTED
        ):
            raise InvariantViolationError(
                "every proposal must be CAMERA/DETECTED."
            )

        if proposal.position in positions:
            raise InvariantViolationError(
                "proposal positions must be unique."
            )

        positions.add(proposal.position)


def _validate_position_set(
    name: str,
    positions: object,
) -> None:
    """Validate one immutable set of domain positions."""

    if not isinstance(positions, frozenset):
        raise DomainValidationError(
            f"{name} must be a frozenset."
        )

    if not all(
        isinstance(position, Position)
        for position in positions
    ):
        raise DomainValidationError(
            f"every {name} item must be a Position."
        )