"""Command-line status entry point for the current implementation draft."""

from ai_logistics_robot import __version__


def main() -> int:
    """Report the currently implemented public-contract foundation."""

    print(
        f"AI-Logistics-Robot {__version__} "
        "— Implementation Draft I-0.3"
    )
    print(
        "Domain model, validated configuration, and public ports are ready. "
        "Concrete adapters and mission execution are not implemented yet."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())