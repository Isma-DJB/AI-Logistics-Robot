"""Unit tests for the V1 domain enumerations."""

import unittest

from ai_logistics_robot.domain.enums import (
    BrainState,
    CommandStatus,
    CommandType,
    FailureReason,
    Heading,
    MissionStatus,
    PathPhase,
    SafetySeverity,
)


class DomainEnumerationTests(unittest.TestCase):
    """Verify the approved enumeration catalogs."""

    def test_brain_state_catalog_is_complete(self) -> None:
        self.assertEqual(len(BrainState), 13)
        self.assertEqual(BrainState.INITIALIZATION.value, "INITIALIZATION")
        self.assertEqual(BrainState.SAFETY_STOP.value, "SAFETY_STOP")

    def test_heading_catalog_is_complete(self) -> None:
        self.assertEqual(
            set(Heading),
            {
                Heading.NORTH,
                Heading.EAST,
                Heading.SOUTH,
                Heading.WEST,
            },
        )

    def test_mission_status_catalog_is_complete(self) -> None:
        self.assertEqual(len(MissionStatus), 5)
        self.assertIn(MissionStatus.SUCCESS, MissionStatus)
        self.assertIn(MissionStatus.ABORTED, MissionStatus)

    def test_path_phase_catalog_is_complete(self) -> None:
        self.assertEqual(
            set(PathPhase),
            {
                PathPhase.OUTBOUND,
                PathPhase.RETURN,
                PathPhase.DETOUR,
            },
        )

    def test_command_catalogs_are_complete(self) -> None:
        self.assertEqual(len(CommandType), 4)
        self.assertEqual(len(CommandStatus), 4)
        self.assertEqual(CommandType.MOVE_FORWARD.value, "MOVE_FORWARD")
        self.assertEqual(CommandStatus.TIMEOUT.value, "TIMEOUT")

    def test_safety_and_failure_catalogs_are_complete(self) -> None:
        self.assertEqual(len(SafetySeverity), 3)
        self.assertEqual(len(FailureReason), 8)
        self.assertEqual(
            FailureReason.EMERGENCY_STOP.value,
            "EMERGENCY_STOP",
        )

    def test_enumerations_are_string_serializable(self) -> None:
        self.assertIsInstance(Heading.NORTH, str)
        self.assertEqual(str(Heading.NORTH), "NORTH")


if __name__ == "__main__":
    unittest.main()