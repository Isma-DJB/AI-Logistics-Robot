"""Unit tests for the real physical-runtime clock adapter."""

import unittest
from datetime import UTC, datetime

from ai_logistics_robot.adapters.hardware.system_clock import (
    SystemClock,
)
from ai_logistics_robot.domain.errors import DomainValidationError
from ai_logistics_robot.ports.clock_port import ClockPort


class SystemClockTests(unittest.TestCase):
    """Verify wall time, monotonic time, and bounded waiting."""

    def test_satisfies_public_clock_port(self) -> None:
        clock = SystemClock()

        self.assertIsInstance(clock, ClockPort)

    def test_returns_timezone_aware_wall_time(self) -> None:
        expected = datetime(
            2026,
            10,
            22,
            18,
            30,
            tzinfo=UTC,
        )
        clock = SystemClock(
            wall_clock_provider=lambda: expected,
        )

        self.assertIs(clock.now(), expected)

    def test_normalizes_valid_monotonic_value(self) -> None:
        clock = SystemClock(
            monotonic_provider=lambda: 12,
        )

        self.assertEqual(clock.monotonic(), 12.0)
        self.assertIsInstance(clock.monotonic(), float)

    def test_waits_only_for_remaining_duration(self) -> None:
        sleep_calls: list[float] = []
        clock = SystemClock(
            monotonic_provider=lambda: 10.5,
            sleep_provider=sleep_calls.append,
        )

        clock.wait_until(13.0)

        self.assertEqual(sleep_calls, [2.5])

    def test_equal_deadline_performs_zero_duration_wait(
        self,
    ) -> None:
        sleep_calls: list[float] = []
        clock = SystemClock(
            monotonic_provider=lambda: 10.0,
            sleep_provider=sleep_calls.append,
        )

        clock.wait_until(10.0)

        self.assertEqual(sleep_calls, [0.0])

    def test_rejects_past_deadline_without_sleeping(self) -> None:
        sleep_calls: list[float] = []
        clock = SystemClock(
            monotonic_provider=lambda: 10.0,
            sleep_provider=sleep_calls.append,
        )

        with self.assertRaisesRegex(
            DomainValidationError,
            "must not precede",
        ):
            clock.wait_until(9.0)

        self.assertEqual(sleep_calls, [])

    def test_rejects_non_callable_dependencies(self) -> None:
        cases = (
            (
                {
                    "wall_clock_provider": object(),
                },
                "wall_clock_provider",
            ),
            (
                {
                    "monotonic_provider": object(),
                },
                "monotonic_provider",
            ),
            (
                {
                    "sleep_provider": object(),
                },
                "sleep_provider",
            ),
        )

        for arguments, expected_message in cases:
            with self.subTest(arguments=arguments):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    expected_message,
                ):
                    SystemClock(**arguments)

    def test_rejects_invalid_wall_clock_results(self) -> None:
        invalid_values = (
            datetime(2026, 10, 22, 18, 30),
            object(),
        )

        for value in invalid_values:
            with self.subTest(value=value):
                clock = SystemClock(
                    wall_clock_provider=(
                        lambda value=value: value
                    ),
                )

                with self.assertRaisesRegex(
                    DomainValidationError,
                    "timezone-aware",
                ):
                    clock.now()

    def test_rejects_invalid_monotonic_results(self) -> None:
        invalid_values = (
            True,
            -1.0,
            float("nan"),
            float("inf"),
            "10",
        )

        for value in invalid_values:
            with self.subTest(value=value):
                clock = SystemClock(
                    monotonic_provider=(
                        lambda value=value: value
                    ),
                )

                with self.assertRaisesRegex(
                    DomainValidationError,
                    "finite non-negative",
                ):
                    clock.monotonic()

    def test_rejects_invalid_deadlines(self) -> None:
        clock = SystemClock(
            monotonic_provider=lambda: 10.0,
        )
        invalid_deadlines = (
            True,
            float("nan"),
            float("inf"),
            "12",
        )

        for deadline in invalid_deadlines:
            with self.subTest(deadline=deadline):
                with self.assertRaisesRegex(
                    DomainValidationError,
                    "finite number",
                ):
                    clock.wait_until(deadline)


if __name__ == "__main__":
    unittest.main()