# AI-Logistics-Robot

AI-Logistics-Robot is a modular autonomous-logistics-robot project developed through simulation first and physical integration second.

The Version 1 mission is triggered by a light target. One robot travels to a safe cell near that target, avoids obstacles, simulates collection, and returns to its base using the path of confirmed outbound poses.

## Current status

**Implementation Draft I-0.5 - Planning and Memory**

This draft adds deterministic path planning and in-memory mission
recording behind the existing PlanningPort and MemoryPort contracts.

AStarPlanner uses cardinal A* search with a Manhattan heuristic. It selects
the shortest reachable authorized goal, preserves deterministic tie
ordering, supports shifted grid origins, and reports an unreachable valid
request through NoPathError.

InMemoryMissionMemory records confirmed outbound and return poses, ordered
mission events, and terminal mission results. It builds the exact reversed
outbound record for return preparation and requires an explicit reset
before recording another mission.

Both components are publicly exported and verified together against the
validated V1 reference configuration. The existing deterministic GridWorld
adapter remains the headless execution platform.

Perception-driven map updates, Brain orchestration, Control, graphical
rendering, and physical integration remain intentionally deferred. I-0.6
will introduce the Brain state machine and Control behavior.

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
python -m pip install -e ".[dev]"
```

### Linux or macOS

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

Install the optional simulation dependency when the Pygame work begins:

```bash
python -m pip install -e ".[dev,simulation]"
```

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