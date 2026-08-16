# AI-Logistics-Robot

AI-Logistics-Robot is a modular autonomous-logistics-robot project developed
through simulation first and physical integration second.

The Version 1 mission is triggered by a light target. One robot travels to a
safe cell near that target, avoids obstacles, simulates collection, and returns
to its base using the path of confirmed outbound poses.

## Current status

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

Physical hardware diagnostics and calibration remain assigned to I-0.9.
Camera, microcontroller, motor, sensor, communication, and final physical V1
integration remain assigned to I-1.0.

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

Repository documentation is maintained in English. Personal learning records
may also be produced separately in French.
