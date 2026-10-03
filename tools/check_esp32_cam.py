"""Run one explicit ESP32-CAM still-image diagnostic."""

import sys
from argparse import ArgumentParser
from collections.abc import Sequence
from pathlib import Path

from ai_logistics_robot.adapters.hardware import (
    Esp32CameraClient,
    Esp32CameraError,
)
from ai_logistics_robot.domain.errors import DomainValidationError


def build_parser() -> ArgumentParser:
    """Build the explicit hardware-diagnostic command line."""

    parser = ArgumentParser(
        description=(
            "Retrieve and validate one JPEG frame from an ESP32-CAM."
        )
    )
    parser.add_argument(
        "--url",
        required=True,
        help="Camera HTTP origin, for example http://192.168.1.109.",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Destination path for the validated JPEG frame.",
    )
    parser.add_argument(
        "--timeout",
        default=5.0,
        type=float,
        help="Positive HTTP timeout in seconds. Default: 5.",
    )
    return parser


def main(
    argv: Sequence[str] | None = None,
) -> int:
    """Capture and save one explicitly requested camera frame."""

    arguments = build_parser().parse_args(argv)

    try:
        client = Esp32CameraClient(
            base_url=arguments.url,
            timeout_seconds=arguments.timeout,
        )
        frame = client.capture_jpeg()
    except (
        DomainValidationError,
        Esp32CameraError,
    ) as error:
        print(
            f"ESP32-CAM diagnostic failed: {error}",
            file=sys.stderr,
        )
        return 2

    output_path: Path = arguments.output

    try:
        output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )
        output_path.write_bytes(frame)
    except OSError as error:
        print(
            f"ESP32-CAM diagnostic could not save the frame: {error}",
            file=sys.stderr,
        )
        return 2

    print(
        f"ESP32-CAM capture succeeded: {len(frame)} bytes."
    )
    print(
        f"Endpoint: {client.capture_url}"
    )
    print(
        f"Saved to: {output_path.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())