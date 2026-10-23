# Diagnostic and Calibration Tools

Hardware diagnostics and calibration operations are exposed as explicit
commands. Network access, camera capture, graphical interaction, and generated
files occur only when the corresponding command is intentionally executed.

## ESP32-CAM Still-Image Diagnostic

`check_esp32_cam.py` retrieves one JPEG frame from the fixed overhead camera,
validates its JPEG boundaries, and writes it to an explicitly selected path.

Example:

```powershell
python tools\check_esp32_cam.py `
    --url "http://192.168.1.109" `
    --output "artifacts\i-1.0\esp32-camera-check.jpg"
```

The diagnostic returns a nonzero exit code when the camera cannot be reached,
the response is not a valid JPEG frame, or the output cannot be written.

## Grid Perspective Calibration

The calibration tool requires the optional vision dependencies:

```powershell
python -m pip install -e ".[vision]"
```

`calibrate_grid.py` opens the stored ESP32-CAM reference image and collects the
four inner corners of the black operational boundary in this order:

1. top-left;
2. top-right;
3. bottom-right;
4. bottom-left.

Run the default I-1.0 calibration with:

```powershell
python tools\calibrate_grid.py
```

Selection controls:

- left click: select the next corner;
- right click or `R`: clear the current selection;
- `Enter`: validate four selected corners;
- `Esc`: cancel without writing calibration files.

Review controls:

- `Enter` or `S`: save the accepted calibration;
- `R`: return to corner selection;
- `Esc`: cancel without saving.

The accepted 7 by 7 calibration produces:

- `configs/hardware/grid_calibration_7x7.yaml`;
- `docs/assets/i-1.0/images/grid-reference-7x7-rectified.jpg`;
- a 700 by 700 pixel rectified image;
- 100 rectified pixels per 20 cm physical grid cell.

The calibration is valid only while the overhead camera remains in its fixed
position. It must be repeated after any camera displacement, rotation, or
change in framing.

## Live Calibrated Grid Diagnostic

`check_calibrated_grid.py` composes the ESP32-CAM capture adapter with the
validated perspective calibration. It captures one live 800 by 600 JPEG frame,
checks its source geometry, and writes a rectified 700 by 700 grid image.

Example:

```powershell
python tools\check_calibrated_grid.py `
    --url "http://192.168.1.109" `
    --output "artifacts\i-1.0\esp32-grid-rectified.jpg"
```

The default calibration file is:

```text
configs/hardware/grid_calibration_7x7.yaml
```

A different calibration can be selected explicitly with `--calibration`.
Diagnostic output under `artifacts` is intentionally ignored by Git.

## ArUco Robot-Pose Diagnostic

`check_robot_pose.py` detects the configured robot marker in either an existing
rectified image or a newly captured ESP32-CAM frame. It converts the marker
center and orientation into the existing platform-independent `RobotPose`
domain model.

The validated V1 robot marker uses:

- OpenCV dictionary `DICT_4X4_50`;
- marker identifier `0`;
- an 8 cm by 8 cm printed image, including its white quiet-zone margin;
- the marker image's top edge pointing toward the front of the robot.

The printable marker is stored at
`docs/assets/i-1.0/images/robot-marker-aruco-id-0.png`.

The rectified-grid coordinate convention is:

- the bottom-left cell is `(0, 0)`;
- `x` increases toward the right;
- `y` increases toward the top;
- the image top corresponds to `NORTH`.

Offline diagnostic example:

```powershell
python tools\check_robot_pose.py `
    --input "artifacts\i-1.0\rectified-grid.png" `
    --output "artifacts\i-1.0\robot-pose-annotated.png"
```

## Arduino Serial Diagnostic

The Arduino serial diagnostic requires the optional hardware dependency:

```powershell
python -m pip install -e ".[hardware]"
```

`check_arduino_serial.py` opens the supervised Arduino connection, verifies
liveness through `PING/PONG`, and reads the current controller and safety
states.

It does not wait for the boot-only `READY` announcement. This allows the
diagnostic to connect safely when the Arduino was already running before the
host opened the serial port.

Run the validated UNO R4 Minima diagnostic with:

```powershell
python tools\check_arduino_serial.py `
    --port "COM3" `
    --timeout 5
```

The Arduino IDE Serial Monitor must be closed before running the command because
only one process can own the serial port.

The diagnostic is intentionally read-only:

- it does not send `REARM`;
- it does not send any motion command;
- it does not change the controller safety state;
- it reports a nonzero exit code after a connection, timeout, protocol, or
  session failure.

The validated I-1.0 sensors-readonly firmware starts in
`ESTOPPED/LATCHED` and keeps every motor output at zero. It accepts `CMD|STOP`
in every safety state and reports `ACK` followed by `DONE|OK`.

The read-only request `ALR|1|SENSORS|<sequence>` returns the raw left infrared,
right infrared, boundary-line, ultrasonic-distance, and light values. A missing
ultrasonic echo is represented by `NONE`. Sensor inspection is available while
safety remains latched and never rearms or activates the motors.

All movement commands remain rejected until bounded execution is implemented
and physically calibrated.

## Fused Physical-Perception Diagnostic

The fused diagnostic requires both optional hardware and vision dependencies:

```powershell
python -m pip install -e ".[hardware,vision]"
```

`check_fused_physical_perception.py` combines live rectified ESP32-CAM
localization with one sequenced Arduino sensor snapshot per observation. It
produces the same immutable `PerceptionSnapshot` contract used by the
platform-independent Brain.

The target lamp must remain off during automatic ambient calibration. Run the
validated passive diagnostic with:

```powershell
python tools\check_fused_physical_perception.py `
    --url "http://192.168.1.109" `
    --port "COM3" `
    --samples 6 `
    --serial-timeout 5 `
    --camera-timeout 15
```

Automatic calibration uses ten lamp-off readings and rejects an unstable
baseline whose raw range exceeds ten. A previously confirmed baseline can be
provided explicitly with `--ambient-light-raw`.

The validated target-light policy uses:

- activation delta: `50`;
- release delta: `30`;
- consecutive stable readings: `5`;
- one-unit ambient tracking for small inactive-state drift;
- complete baseline freezing for ambiguous, candidate, and active targets.

Each reported sample includes the raw light value, delta, tracked
`baseline_after`, target state, robot pose, local obstacle facts, boundary
hazard, ultrasonic distance, and capture timestamp.

The diagnostic is strictly passive. It requires `ESTOPPED/LATCHED` before and
after observation, sends neither `REARM` nor motion commands, and fails
explicitly when ArUco localization, serial communication, calibration, or
sensor interpretation is unavailable.
