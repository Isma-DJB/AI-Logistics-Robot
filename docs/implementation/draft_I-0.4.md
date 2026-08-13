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

## 4. Batch C - Rejection and Boundary Rules

GridWorld rejection behavior is now covered by dedicated tests.

Verified behavior:

- Commands for another robot are rejected before state mutation.
- Invalid command objects are rejected before state mutation.
- Every grid boundary rejects outward movement with OUT_OF_BOUNDS.
- Shifted grid origins are handled without hard-coded coordinates.
- Obstacles reject movement with BLOCKED.
- The target cell rejects movement with BLOCKED.
- Failed movement preserves pose_before and pose_after.
- Failed movement preserves confirmed simulated time.
- Valid commands can continue after a rejected movement.

Verification:

- 7 rejection and boundary tests passed.
- The complete suite passed with 189 tests.
- No GridWorld implementation change was required.

## 5. Batch D - Public Integration

GridWorld is now exported as the public headless simulation adapter and
has been verified against the validated V1 reference configuration.

Verified behavior:

- GridWorld is importable from ai_logistics_robot.adapters.simulation.
- The concrete adapter satisfies SimulationPort structurally.
- The reference GridMap and initial RobotPose load from simulation.yaml.
- A safe command fragment produces the expected confirmed pose.
- reset enables an identical deterministic replay.
- The configured collection duration advances simulated time to 3.0 seconds.
- The reference obstacle at (1,4) rejects forward movement with BLOCKED.
- No graphical or hardware dependency is required.

Verification:

- 4 reference-scenario integration tests passed.
- The complete suite passed with 193 tests.
- Ruff, mypy, and the I-0.1 structure check passed.