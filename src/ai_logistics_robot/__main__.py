"""Command-line status entry point for the current implementation draft."""

from ai_logistics_robot import __version__


def main() -> int:
    """Report the implemented Planning and Memory foundation."""

    print(
        f"AI-Logistics-Robot {__version__} "
        "- Implementation Draft I-0.5"
    )
    print(
        "Domain model, validated configuration, public ports, "
        "deterministic GridWorld, A* planning, and in-memory "
        "mission recording are ready. Perception behavior, Brain "
        "orchestration, Control, graphical rendering, and physical "
        "execution are not implemented yet."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
