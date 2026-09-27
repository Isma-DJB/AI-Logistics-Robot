# AI-Logistics-Robot

AI-Logistics-Robot is a modular autonomous-logistics-robot project developed
through simulation first and physical integration second.

The Version 1 mission is triggered by a light target. One robot travels to a
safe cell near that target, avoids obstacles, simulates collection, and returns
to its base using the path of confirmed outbound poses.

![Physical AI-Logistics-Robot test grid](docs/assets/i-0.9/images/grid-overview.jpg)

*I-0.9 physical test environment with the robot, illuminated target, grid
boundary, and representative movable obstacles.*

## Current status

### I-0.9 physical hardware diagnostics - Validated

I-0.9 validates the first assembled physical prototype and the controlled
transition from simulation to repeatable real-hardware experiments.

The current prototype includes:

- an Arduino UNO R4 Minima and an L9110S-compatible motor driver;
- two calibrated TT geared motors;
- two local infrared obstacle sensors;
- one frontal HC-SR04 ultrasonic sensor;
- one downward black-boundary sensor;
- one photoresistor with a 10 kOhm voltage-divider resistor;
- EEPROM-based motor calibration;
- reduced-speed obstacle and boundary recovery manoeuvres;
- controlled light-triggered mission activation;
- combined light and distance confirmation near the illuminated target.

Two consecutive physical mission runs passed after the forward motor-restart
correction. Mission activation, obstacle recovery, black-boundary recovery,
motor relaunch, and illuminated-target arrival were validated without manual
intervention.

The repository now contains the
[experimental Arduino firmware](firmware/arduino/i_0_9_hardware_prototype/i_0_9_hardware_prototype.ino)
and the
[I-0.9 hardware record](docs/implementation/draft_I-0.9.md).

The next physical integration milestone is an ESP32-CAM installed at a fixed
elevated position and angled downward toward the complete grid. It will not be
mounted on the robot. Its future role is to provide a stable global visual
observation of the robot, grid, target areas, and obstacles.

A future vision adapter will translate these observations into data accepted
through the I-0.8 `Perception` boundary. The validated I-0.8 Brain, Planning,
Control, and Memory components will remain responsible for high-level mission
decisions.

### I-0.8 validated software baseline

**Implementation Draft I-0.8 - Complete Software Scenarios and Acceptance Tests**

I-0.8 delivers the complete simulation-side V1 software through one validated
public application boundary.

The current implementation includes:

- immutable validated domain values and YAML settings;
- explicit public ports for every core dependency;
- deterministic `GridWorld` execution and A* path planning;
- confirmed outbound and return-path mission memory;
- complete deterministic Brain orchestration;
- safety-aware Control with a priority latched emergency stop;
- `SimulatedClock`, `GridWorldPerception`, and `InMemoryMonitoring`;
- passive headless and optional Pygame renderers;
- a guarded `MissionRunner` execution and reset lifecycle;
- one public reference dependency assembly;
- transient scenario obstacles for real rejection and detour verification;
- deterministic multi-mission replay.

Six complete acceptance scenarios exercise the assembled headless application.
They cover AC-01 through AC-12 and SEQ-01 through SEQ-03, including:

- inactive target stationarity and one mission per activation edge;
- collision-free nominal navigation and authorized target arrival;
- stationary timed collection;
- exact confirmed return paths and safe optional detours;
- atomic blocked movement and replanning;
- ordered immutable mission events;
- priority hazard stop, manual safety rearm, and no automatic resumption;
- two missions without restarting Python;
- identical outcomes for identical inputs.

The complete repository suite discovers and passes 370 automated tests.

Pygame remains an optional simulation dependency. Importing and assembling the
headless application does not import Pygame, and the platform-independent core
contains no direct graphical or hardware dependency.

Physical hardware diagnostics and calibration are validated through I-0.9.
Final camera-based perception, microcontroller communication, and
end-to-end physical V1 integration remain assigned to I-1.0.

## Public status command

After installation, either command reports the completed software milestone:

```bash
python -m ai_logistics_robot
ai-logistics-robot
```

The command reports project status. Mission execution remains explicit through
the public application interfaces so callers and tests retain control over the
validated settings, deterministic epoch, external target state, hazards, and
cycle bounds.

## Public application interfaces

`ai_logistics_robot.app` exports:

- `SimulationApplication`;
- `build_simulation_application`;
- `MissionRunner`.

`build_simulation_application()` is the reference composition root. It creates
fresh isolated Simulation, Clock, Perception, Planning, Control, Memory,
Monitoring, Brain, Renderer, and Runner state from one validated `Settings`
instance and one explicit timezone-aware epoch.

The Brain remains the only component that makes mission decisions.
`MissionRunner` coordinates public operations without planning movement,
constructing commands, interpreting navigation outcomes, or changing Brain
state directly.

## Architecture

The V1 core is divided into the following modules:

- `domain`: immutable data objects, enumerations, and domain rules;
- `ports`: public contracts known by the core;
- `brain`: mission orchestration and state machine;
- `planning`: path calculation and validation;
- `perception`: normalized observations and local hazard information;
- `control`: motion-step construction and safety operations;
- `memory`: confirmed path, events, and mission result;
- `adapters`: simulation, visualization, monitoring, and physical hardware;
- `app`: configuration, dependency assembly, and execution loop.

Concrete platforms remain behind adapters. The Brain must never import Pygame,
GridWorld, Arduino, or ESP32-specific code.

## Requirements

- Python 3.11 or newer
- Git
- VS Code with the Python extension recommended

## Local setup

### Windows PowerShell

```powershell
py -3.11 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev,simulation]"
```

### Linux or macOS

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev,simulation]"
```

The `simulation` extra installs the optional Pygame adapter required by the
complete development test suite. A base installation remains headless and does
not require Pygame.

## Verification

```bash
python tools/check_project_structure.py
python -m unittest discover -s tests -p "test_*.py"
python -m ruff check .
python -m mypy src
python -m pip check
python -m build
python -m ai_logistics_robot
```

## Documentation

- [`docs/requirements/v1`](docs/requirements/v1/) contains the preserved V1
  requirements drafts.
- [`docs/architecture`](docs/architecture/) contains diagrams and architecture
  decisions.
- [`docs/implementation`](docs/implementation/) contains implementation-session
  records.
- [`docs/implementation/draft_I-0.9.md`](docs/implementation/draft_I-0.9.md)
  records the physical hardware diagnostics, calibration results, limitations,
  and the planned overhead ESP32-CAM transition.
- [`firmware/arduino/i_0_9_hardware_prototype`](firmware/arduino/i_0_9_hardware_prototype/)
  contains the verified experimental Arduino firmware.
- [`docs/assets/i-0.9/images`](docs/assets/i-0.9/images/)
  contains the curated I-0.9 visual evidence.

Repository documentation is maintained in English. Personal learning records
may also be produced separately in French and German.
