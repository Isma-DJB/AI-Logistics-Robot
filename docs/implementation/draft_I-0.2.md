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

## Current status

Implementation Draft I-0.2 started from the validated I-0.1 baseline.