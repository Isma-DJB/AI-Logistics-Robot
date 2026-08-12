# V1 Implementation — Draft I-0.2

## Session objective

Implement the platform-independent V1 domain model, its enumerations,
validation rules, and configuration model.

## Approved implementation order

1. Enumerations, domain errors, and geometry.
2. Commands, paths, and mission objects.
3. Perception, safety, events, and system status.
4. Grid map and validated configuration models.

## Domain decisions

- Domain objects are immutable.
- Domain code has no dependency on simulation, hardware, camera, serial,
  Pygame, or project adapters.
- `BrainState` represents the execution state controlled exclusively by
  the Brain.
- `MissionStatus` represents the lifecycle and final outcome of a mission.
- `Position` validates coordinate types but does not hard-code the V1
  grid size.
- `GridMap` owns bounds validation because it knows the configured origin,
  width, and height.
- Public contracts use typed objects rather than raw dictionaries.
- Unknown physical values remain explicit and are never guessed.

## Verification strategy

Each implementation lot must pass:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
python -m ruff check .
python -m mypy src
python tools/check_project_structure.py
```

## Scope completed

- Implemented the approved domain enumerations and Brain state catalog.
- Added a domain-specific exception hierarchy.
- Implemented immutable positions and robot poses.
- Implemented motion commands and confirmed command results.
- Implemented versioned path plans and confirmed path records.
- Implemented mission lifecycle objects and terminal-state invariants.
- Implemented normalized, timestamped perception objects.
- Implemented safety events and latched safety status.
- Implemented ordered mission events with typed immutable details.
- Implemented read-only system status snapshots.
- Implemented GridMap bounds, obstacle, traversal, and arrival rules.
- Implemented immutable configuration objects and cross-object validation.
- Implemented safe typed loading of the reference YAML configuration.
- Added the PyYAML type stubs to the reproducible development environment.

## Verification results

| Check | Result |
| --- | --- |
| Automated tests | 140 of 140 passed |
| Ruff | All checks passed |
| mypy | No issues found in 38 source files |
| Dependency consistency | No broken requirements found |
| I-0.1 structure baseline | Passed |
| Platform-import boundaries | Passed |
| Reference YAML loading | `v1-reference 10 10 robot_1` |
| Command-line entry point | Executed successfully |

## Issues and corrections

- Grid bounds were assigned to `GridMap` instead of being hard-coded in
  `Position`, preserving support for future map sizes and shifted origins.
- Raw YAML dictionaries remain internal to the loader and are converted
  into typed immutable objects before entering the application.
- Unknown physical and operational values remain `None`; no value was
  guessed.
- mypy initially lacked PyYAML annotations. `types-PyYAML` was added to
  the development dependencies and the full type check then passed.
- PowerShell displayed some UTF-8 punctuation incorrectly, but this was
  an output-encoding issue rather than a domain-code failure.

## Intentionally not implemented

- Ports and public protocols.
- Brain state transitions and mission orchestration.
- Planning and Memory behavior.
- GridWorld command execution and movement rules.
- Pygame rendering.
- Complete scenario and acceptance tests.
- Camera, motor, and other hardware communication.

These responsibilities belong to later approved implementation drafts.

## Current status

Implementation Draft I-0.2 is complete and ready for final review and
integration into `main`.

## Next draft

**I-0.3 — Ports and public contracts.**