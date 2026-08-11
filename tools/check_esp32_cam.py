"""ESP32-CAM diagnostic placeholder reserved for Implementation Draft I-0.9."""


def main() -> int:
    """Refuse to run a hardware diagnostic before its implementation."""
    print("ESP32-CAM diagnostic is not implemented in I-0.1; no hardware action was performed.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
