"""HTTP capture adapter for one fixed overhead ESP32-CAM."""

from collections.abc import Callable
from math import isfinite
from urllib.parse import urlsplit
from urllib.request import urlopen

from ai_logistics_robot.domain.errors import DomainValidationError

FrameFetcher = Callable[[str, float], bytes]


class Esp32CameraError(RuntimeError):
    """Report one failed or invalid ESP32-CAM capture."""


def _fetch_bytes(
    url: str,
    timeout_seconds: float,
) -> bytes:
    """Download one HTTP response body from the camera."""

    try:
        with urlopen(
            url,
            timeout=timeout_seconds,
        ) as response:
            payload = response.read()
    except OSError as error:
        raise Esp32CameraError(
            "ESP32-CAM capture request failed."
        ) from error

    if not isinstance(payload, bytes):
        raise Esp32CameraError(
            "ESP32-CAM returned a non-binary response."
        )

    return payload


class Esp32CameraClient:
    """Retrieve validated JPEG frames from an ESP32-CAM."""

    __slots__ = (
        "_base_url",
        "_fetcher",
        "_timeout_seconds",
    )

    def __init__(
        self,
        *,
        base_url: str,
        timeout_seconds: float = 5.0,
        fetcher: FrameFetcher | None = None,
    ) -> None:
        """Validate and retain the camera connection settings."""

        if not isinstance(base_url, str):
            raise DomainValidationError(
                "base_url must be a string."
            )

        normalized_base_url = base_url.strip().rstrip("/")
        parsed_url = urlsplit(normalized_base_url)

        if (
            parsed_url.scheme not in {"http", "https"}
            or not parsed_url.netloc
            or parsed_url.path not in {"", "/"}
            or parsed_url.query
            or parsed_url.fragment
        ):
            raise DomainValidationError(
                "base_url must be an HTTP origin without a path, query, or fragment."
            )

        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not isfinite(float(timeout_seconds))
            or float(timeout_seconds) <= 0.0
        ):
            raise DomainValidationError(
                "timeout_seconds must be a finite positive number."
            )

        if fetcher is not None and not callable(fetcher):
            raise DomainValidationError(
                "fetcher must be callable."
            )

        self._base_url = normalized_base_url
        self._timeout_seconds = float(timeout_seconds)
        self._fetcher = fetcher or _fetch_bytes

    @property
    def base_url(self) -> str:
        """Return the normalized camera origin."""

        return self._base_url

    @property
    def capture_url(self) -> str:
        """Return the camera still-image endpoint."""

        return f"{self._base_url}/capture"

    @property
    def timeout_seconds(self) -> float:
        """Return the configured HTTP timeout."""

        return self._timeout_seconds

    def capture_jpeg(self) -> bytes:
        """Download and validate one complete JPEG frame."""

        try:
            payload = self._fetcher(
                self.capture_url,
                self._timeout_seconds,
            )
        except Esp32CameraError:
            raise
        except OSError as error:
            raise Esp32CameraError(
                "ESP32-CAM capture request failed."
            ) from error

        if (
            not isinstance(payload, bytes)
            or len(payload) < 4
            or not payload.startswith(b"\xff\xd8")
            or not payload.endswith(b"\xff\xd9")
        ):
            raise Esp32CameraError(
                "ESP32-CAM response is not a complete JPEG frame."
            )

        return payload