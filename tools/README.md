# Diagnostic Tools

Hardware diagnostics are explicit commands. They perform no network or physical
action unless the user provides the required connection parameters.

## ESP32-CAM Still-Image Diagnostic

`check_esp32_cam.py` retrieves one JPEG frame from the fixed overhead camera,
validates its JPEG boundaries, and writes it to an explicitly selected path.

Example:

```powershell
python tools\check_esp32_cam.py `
    --url "http://192.168.1.109" `
    --output "artifacts\i-1.0\esp32-camera-check.jpg"