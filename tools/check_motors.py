"""Motor diagnostic placeholder reserved for Implementation Draft I-0.9."""


def main() -> int:
    """Refuse to energize motors before the diagnostic is implemented."""
    print("Motor diagnostic is not implemented in I-0.1; no motor command was sent.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
