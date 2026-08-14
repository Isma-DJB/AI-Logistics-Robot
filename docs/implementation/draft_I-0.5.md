# Implementation Draft I-0.5 - Planning and Memory

Status: IN PROGRESS
Branch: implementation/i-0.5
Base commit: 15d6bd8
Started: August 14, 2026

## 1. Objective

Implementation Draft I-0.5 introduces deterministic path planning and
in-memory mission recording through the existing PlanningPort and
MemoryPort contracts.

Planning calculates paths over immutable GridMap snapshots. Memory stores
confirmed mission facts. Neither component orchestrates the mission,
executes movement, performs perception, or controls physical hardware.

## 2. Transition Review

The I-0.4 baseline is synchronized at commit 15d6bd8 and passes all
193 automated tests.

Approved design decisions:

- Planning uses deterministic A* with cardinal movement.
- Every plan starts at the supplied current position.
- Every plan ends at one supplied authorized goal.
- Planned positions must remain inside the grid and traversable.
- The shortest reachable authorized goal is selected.
- Equal-cost goals are resolved by their supplied order.
- Absence of a path is reported through a dedicated `NoPathError`.
- Mapping `NoPathError` to FailureReason.NO_PATH belongs to the Brain.
- Memory records only poses explicitly confirmed by its caller.
- Return preparation reverses the confirmed outbound pose history.
- Detour planning is supported without creating DETOUR PathRecords.
- Events remain ordered and isolated by mission identity.
- A completed recording requires an explicit reset before a new mission.

## 3. Planned Batches

### Batch A - Deterministic A* Planning

Implement nominal shortest-path creation, authorized-goal selection,
cardinal adjacency, plan versioning, and deterministic tie-breaking.

### Batch B - Planning Validation and Failure Rules

Verify invalid inputs, blocked goals, unreachable destinations,
shifted origins, start-at-goal behavior, and atomic `NoPathError` handling.

### Batch C - Memory Path Lifecycle

Implement mission start, confirmed outbound and return pose recording,
immutable snapshots, and reverse return-path construction.

### Batch D - Memory Events and Completion

Implement ordered event recording, identity validation, terminal mission
completion, lifecycle rejection rules, and deterministic reset.

### Batch E - Public Integration and Closure

Export the concrete implementations, verify the reference scenario across
Planning and Memory, update public documentation, and run the complete
quality gate.

## 4. Explicitly Deferred

The following concerns remain outside I-0.5:

- Brain state-machine orchestration.
- Control command construction and execution.
- Perception-driven map updates.
- Automatic replanning decisions.
- Return-detour orchestration.
- Graphical rendering.
- Persistent database or file-backed storage.
- Physical hardware integration.


## 5. Batch A - Deterministic `A*` Planning

`AStarPlanner` now creates deterministic shortest paths over immutable
`GridMap` snapshots and structurally satisfies `PlanningPort`.

Implemented behavior:

- Cardinal `A*` search using Manhattan distance as the heuristic.
- Every generated plan includes the supplied starting position.
- Every generated plan ends at one authorized traversable goal.
- Obstacles, the target cell, and positions outside the grid are excluded.
- The shortest reachable authorized goal is selected.
- Equal-cost goals preserve the order supplied by the caller.
- Equal-cost search nodes preserve `GridMap` cardinal-neighbor order.
- Mission identity, robot identity, phase, and version are preserved.
- Repeated calls with identical inputs produce identical `PathPlan` values.
- A dedicated `NoPathError` represents an unreachable planning outcome.

Verification:

- 5 nominal `AStarPlanner` tests passed.
- The complete suite passed with 198 tests.
- No graphical, hardware, or external planning dependency was introduced.
- Detailed validation and no-path cases remain assigned to Batch B.

## 6. Batch B - Planning Validation and Failure Rules

Planning validation and unreachable-path behavior are now covered by
dedicated tests.

Verified behavior:

- Mission and robot identifiers must be non-empty strings.
- The start pose, world, phase, and version are validated before search.
- Plan versions must be positive integers and explicitly reject booleans.
- Authorized goals must form a non-empty immutable tuple.
- Authorized goals must be unique Position values.
- The start pose and every authorized goal must be traversable.
- Obstacles, the target cell, and out-of-bounds goals are rejected.
- A start pose already located at the goal produces a one-position plan.
- Shifted GridMap origins are handled without hard-coded coordinates.
- An unreachable goal is skipped when another authorized goal is reachable.
- NoPathError is raised when every authorized goal is unreachable.
- A valid request succeeds immediately after a no-path result.

Verification:

- 9 validation and failure tests passed.
- The complete suite passed with 207 tests.
- Ruff and mypy passed.
- No planning implementation change was required.

## 7. Batch C - Memory Path Lifecycle

InMemoryMissionMemory now records one mission and its confirmed navigation
history using immutable public snapshots.

Implemented behavior:

- The concrete memory structurally satisfies MemoryPort.
- Recording starts with one created or active Mission.
- Outbound and return poses are stored in separate immutable histories.
- Only poses explicitly supplied by the caller are recorded.
- Repeated positions and heading changes are preserved exactly.
- build_return_path reverses only the confirmed outbound pose history.
- Return-phase poses do not alter the prepared reverse path.
- Empty outbound history produces a valid empty PathRecord.
- Repeated return-path construction is deterministic and non-mutating.
- Active mission, completed mission, poses, and events are read-only.
- Mission events and terminal completion are implemented for Batch D
  validation.

Verification:

- 9 nominal memory path-lifecycle tests passed.
- The complete suite passed with 216 tests.
- Ruff, mypy, and the project-structure check passed.
- No persistent storage or external dependency was introduced.

## 8. Batch D - Memory Events and Completion

Mission-event ordering, terminal completion, lifecycle rejection, and reset
behavior are now covered by dedicated tests.

Verified behavior:

- Recording accepts only created or active missions.
- Every recording operation requires a previously started mission.
- A second mission cannot start before an explicit reset.
- Pose validation rejects invalid types and the DETOUR recording phase.
- Invalid pose operations preserve both navigation histories.
- Events must match the active mission and robot identities.
- Event identifiers must remain unique.
- Event sequence numbers must increase, while gaps remain valid.
- SUCCESS, FAILED, and ABORTED outcomes can close a recording.
- Completion requires matching mission identity and fixed geometry.
- Successful completion preserves poses, events, and return-path access.
- Completed recordings reject every further mutation.
- Reset is idempotent and clears missions, poses, events, and results.
- A new mission can start after reset.

Verification:

- 13 event, completion, and lifecycle-rule tests passed.
- The complete suite passed with 229 tests.
- Ruff and mypy passed.
- No memory implementation change was required.
