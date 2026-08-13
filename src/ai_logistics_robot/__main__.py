"""Command-line status entry point for the current implementation draft."""

from ai_logistics_robot import __version__


def main() -> int:
    """Report the currently implemented simulation foundation."""

    print(
        f"AI-Logistics-Robot {__version__} "
        "- Implementation Draft I-0.4"
    )
    print(
        "Domain model, validated configuration, public ports, and "
        "deterministic headless GridWorld behavior are ready. "
        "Planning, Memory, Brain, Control, and physical execution "
        "are not implemented yet."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())