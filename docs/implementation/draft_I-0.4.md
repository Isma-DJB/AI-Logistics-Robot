# Implementation Draft I-0.4 - GridWorld and Movement Rules

Status: IN PROGRESS
Branch: implementation/i-0.4
Base commit: 9704569
Started: August 13, 2026

## 1. Objective

Implementation Draft I-0.4 introduces a deterministic and headless
GridWorld adapter that executes the V1 movement rules through the public
SimulationPort contract.

GridWorld executes commands. It does not plan paths, make mission
decisions, control physical hardware, or provide graphical rendering.

## 2. Batch A - State Lifecycle

GridWorld now validates and stores one immutable GridMap, one robot
identity, and one traversable initial RobotPose.

Implemented behavior:

- Read-only access to the confirmed current pose.
- Read-only access to elapsed simulated time.
- Immutable world access through read_world().
- Finite and non-negative simulated-time advancement.
- Atomic rejection of invalid durations and numeric overflow.
- Deterministic reset to the initial pose and 0.0 seconds.

Verification:

- 11 GridWorld state tests passed.
- The complete suite passed with 175 tests.
- Command execution remains intentionally deferred to Batch B.

## 3. Batch B - Normal Command Execution

GridWorld now implements the complete apply_command() operation required
by SimulationPort.

Implemented behavior:

- STOP succeeds without changing the confirmed pose.
- TURN_LEFT rotates counterclockwise by one quarter turn.
- TURN_RIGHT rotates clockwise by one quarter turn.
- MOVE_FORWARD advances exactly one cell in the current heading.
- Every successful command produces an immutable CommandResult.
- Sequential commands use the latest confirmed pose.
- Commands do not advance simulated time implicitly.
- A command must belong to the configured robot.
- Movement is guarded by configured bounds and traversability rules.
- State changes occur only after a valid result is constructed.

Verification:

- 7 normal-command tests passed.
- The 11 state-lifecycle tests remained green.
- The complete suite passed with 182 tests.
- Ruff and mypy passed.
- Detailed rejection cases remain assigned to Batch C.