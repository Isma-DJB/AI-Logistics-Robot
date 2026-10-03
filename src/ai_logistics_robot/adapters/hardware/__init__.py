"""Physical robot, sensor, motor, and ESP32-CAM adapter package."""

from ai_logistics_robot.adapters.hardware.esp32_camera import (
    Esp32CameraClient,
    Esp32CameraError,
)

__all__ = (
    "Esp32CameraClient",
    "Esp32CameraError",
)