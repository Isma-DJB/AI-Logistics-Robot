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

The supervised firmware accepts bounded `MOVE_FORWARD`, `TURN_LEFT`, and
`TURN_RIGHT` primitives. The forward pulse does not independently certify a
20 cm transition, and the coarse turn window does not independently certify a
90-degree rotation. The host must confirm the resulting logical pose through
the overhead camera.

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

## Bounded Arduino Motion Diagnostic

`check_bounded_arduino_motion.py` validates one host-controlled bounded forward
exchange without claiming a complete 20 cm cell transition. The robot must be
secured on a stable support with both wheels lifted and free.

Close the Arduino Serial Monitor, keep the power switch accessible, and run:

~~~powershell
python tools\check_bounded_arduino_motion.py `
    --port "COM3" `
    --confirm-wheels-lifted `
    --timeout 5
~~~

The diagnostic refuses to open the serial connection unless
`--confirm-wheels-lifted` is present. It then requires an initial
`ESTOPPED/LATCHED` state, clear infrared sensors, a safe boundary reading, and
an available ultrasonic distance greater than 200 mm.

After preflight it sends `REARM`, executes exactly one bounded
`MOVE_FORWARD`, requires matching `ACK` and terminal `DONE|OK`, and always
requests `ESTOP` during cleanup. Successful completion therefore ends in the
confirmed `ESTOPPED/LATCHED` state.


## Camera-Confirmed Motion Diagnostic

`check_camera_confirmed_motion.py` composes fused physical perception,
`PhysicalRobotControl`, and the supervised Arduino client. It accepts a motion
as successful only when the post-command camera observation exactly matches
the expected adjacent cell or cardinal heading.

The diagnostic is validated offline and has been exercised live with a fully
charged battery, stable calibrated camera, clear local sensors, and direct
operator supervision. A confirmed left turn succeeded exactly, while
insufficient right-turn and forward outcomes were rejected and ended in
`ESTOPPED/LATCHED`.

The target lamp must remain off when automatic ambient calibration is used.
Close the Arduino Serial Monitor and run exactly one command with:

~~~powershell
python tools\check_camera_confirmed_motion.py `
    --url "http://192.168.1.109" `
    --port "COM3" `
    --command "TURN_LEFT" `
    --confirm-ground-test `
    --serial-timeout 5 `
    --camera-timeout 15
~~~

The `--command` value must be `MOVE_FORWARD`, `TURN_LEFT`, or `TURN_RIGHT`.
The tool refuses hardware access without `--confirm-ground-test`. It requires
an initial `ESTOPPED/LATCHED` controller, performs fused pre-command
perception, rearms explicitly, executes one bounded command, and always
requests final `ESTOP`.

A firmware `DONE|OK` alone is insufficient. The command succeeds only if the
camera confirms the exact domain pose. A hazard, unexpected pose, serial
failure, or unconfirmed result fails explicitly and preserves or restores the
safety latch. Interrupted ESP32-CAM response bodies are retried at most three
times; repeated incomplete responses still fail explicitly.

The current chassis has a documented intermittent wheel-start imbalance.
Multiple I-0.9 motor changes and calibration attempts did not eliminate it.
The diagnostic must therefore be interpreted as a software supervision and
pose-confirmation tool, not as a guarantee that this particular chassis can
repeat every requested trajectory.

## Assisted Physical-Mission Evidence Diagnostic

`check_assisted_obstacle_scene.py` reviews physical grid changes, retains
operator-confirmed obstacles, replans from the current camera-confirmed pose,
derives the required next heading, and renders annotated Batch E evidence.

Camera proposals never become planning obstacles automatically. Every proposal
must be passed through `--confirm-obstacle` or `--clear-proposal`. Previously
confirmed cells are retained with repeatable `--existing-obstacle` arguments.
The target cell itself is non-traversable; outbound goals are its traversable
cardinal neighbours, while `RETURN` selects the configured physical base.

The detector retains a conservative changed-cell default of `0.12`.
`--minimum-changed-fraction` allows an explicitly reviewed scene to use another
finite fraction in the interval `]0,1]`. A separate scene-wide drift guard
rejects excessive visual change and requires a fresh same-state reference.

Example initial review for the validated final scene:

~~~powershell
python tools\check_assisted_obstacle_scene.py `
    --reference "artifacts\i-1.0\batch-e-final-scene-reference-02-2026-10-26.jpg" `
    --current "artifacts\i-1.0\batch-e-final-scene-initial-2026-10-26.jpg" `
    --output "artifacts\i-1.0\batch-e-final-outbound-step-01-accepted-2026-10-26.jpg" `
    --target-position "4,4" `
    --minimum-changed-fraction 0.06 `
    --confirm-obstacle "0,2" `
    --confirm-obstacle "2,0" `
    --confirm-obstacle "2,1" `
    --confirm-obstacle "4,2" `
    --confirm-obstacle "2,4" `
    --clear-proposal "0,1" `
    --target-active `
    --phase "OUTBOUND" `
    --scenario-id "batch-e-final-mission" `
    --step-number 1
~~~

Subsequent stages use the new frame as both `--reference` and `--current`,
retain the five cells through `--existing-obstacle`, and select either
`OUTBOUND` or `RETURN`. A one-position plan produces `COMPLETED`; otherwise the
tool reports the next position and heading as `REPLANNED`.

The validated sixteen-stage scenario used manual physical repositioning because
the current chassis cannot guarantee exact cell motion. The software still
owned all perception, obstacle-state validation, A* planning, heading
derivation, fail-safe rejection, target-arrival selection, collection evidence,
return replanning, and final completion decisions.
