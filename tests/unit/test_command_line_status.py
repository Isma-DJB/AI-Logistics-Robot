"""Unit tests for the public command-line status."""

import unittest
from contextlib import redirect_stdout
from io import StringIO

from ai_logistics_robot.__main__ import main


class CommandLineStatusTests(unittest.TestCase):
    """Verify that the public status describes the current milestone."""

    def test_main_reports_completed_i_0_6_capabilities(
        self,
    ) -> None:
        output = StringIO()

        with redirect_stdout(output):
            exit_code = main()

        status_text = output.getvalue()

        self.assertEqual(exit_code, 0)
        self.assertIn(
            "Implementation Draft I-0.6",
            status_text,
        )
        self.assertIn(
            "Brain orchestration",
            status_text,
        )
        self.assertIn(
            "safety-aware Control",
            status_text,
        )
        self.assertIn(
            "remain deferred",
            status_text,
        )
        self.assertNotIn(
            "Implementation Draft I-0.5",
            status_text,
        )
        self.assertNotIn(
            "not implemented yet",
            status_text,
        )


if __name__ == "__main__":
    unittest.main()
