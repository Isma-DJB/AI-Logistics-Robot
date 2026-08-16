# AI-Logistics-Robot

AI-Logistics-Robot is a modular autonomous-logistics-robot project developed through simulation first and physical integration second.

The Version 1 mission is triggered by a light target. One robot travels to a safe cell near that target, avoids obstacles, simulates collection, and returns to its base using the path of confirmed outbound poses.

## Current status

**Implementation Draft I-0.7 - PygameRenderer**

This draft adds a deterministic passive Pygame visualization adapter behind
the existing `RendererPort`.

`PygameRenderer` displays:

- the origin-aware immutable grid and its traversable cells;
- obstacles, the base, the target, and authorized arrival cells;
- the confirmed robot pose and all four cardinal headings;
- the active path and its authorized goal;
- the Brain state, mission status, plan version, safety latch, and latest error;
- a bounded list of recent immutable mission events.

Rendering remains strictly passive. The adapter does not plan movement, execute
commands, mutate confirmed state, or influence Brain and Control decisions.
Closing the SDL window closes only the visualization adapter.

Pygame remains an optional simulation dependency. The platform-independent
core and headless `GridWorld` continue to work without importing it.

The reference integration combines the validated YAML configuration,
`AStarPlanner`, `SystemStatus`, `MissionEvent`, and `PygameRenderer`. It verifies
the complete reference world, plan, status panel, bounded events, and renderer
shutdown with the SDL dummy video driver.

The complete application runner, concrete perception, monitoring and clock
adapters, complete acceptance scenarios, and physical integration remain
intentionally deferred beyond I-0.7.

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

Concrete platforms remain behind adapters. The Brain must never import Pygame, GridWorld, Arduino, or ESP32-specific code.

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
python -m ai_logistics_robot
```

## Documentation

- [`docs/requirements/v1`](docs/requirements/v1/) contains the preserved V1 requirements drafts.
- [`docs/architecture`](docs/architecture/) contains diagrams and architecture decisions.
- [`docs/implementation`](docs/implementation/) contains implementation-session records.

Repository documentation is maintained in English. Personal learning records may also be produced separately in French.