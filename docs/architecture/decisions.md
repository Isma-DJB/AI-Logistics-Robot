# V1 Architecture Decisions

This file is the implementation-facing index of approved architectural decisions. Detailed rationale remains in the V1 requirements drafts.

## ADR-001 — Source layout

- **Status:** Accepted
- Python packages live under `src/ai_logistics_robot`.
- Tests, tools, configuration, and documentation remain outside the installable package.

## ADR-002 — Ports and adapters

- **Status:** Accepted
- The core depends on public contracts, never on concrete simulation or hardware implementations.
- GridWorld and the physical robot must remain interchangeable through configuration and adapters.

## ADR-003 — Domain independence

- **Status:** Accepted
- `domain` depends on no other project package.
- Core packages must not import Pygame, Arduino, ESP32, serial, or camera-specific libraries.
- `app/bootstrap.py` is the only assembly location allowed to know all concrete implementations.

## ADR-004 — Headless simulation first

- **Status:** Accepted
- GridWorld must run without a graphical interface.
- Pygame is an optional passive renderer and must not influence decisions.

## ADR-005 — Safety locality

- **Status:** Accepted
- Immediate stopping must not depend on the ESP32-CAM, monitoring, or a remote central station.
- Hardware diagnostics remain disabled until explicit implementation and verification.

## ADR-006 — English repository documentation

- **Status:** Accepted
- All documentation committed to the GitHub repository is written in English.
- French learning documents are maintained separately and are not committed.

## ADR-007 — Unknown physical values

- **Status:** Accepted
- Robot dimensions, turning radius, safety margin, speeds, and timing values remain `null` until measured or calibrated.
- Implementation must not silently substitute guessed values.
