# Implementation Draft I-0.5 - Planning and Memory

Status: COMPLETE
Branch: implementation/i-0.5
Base commit: 15d6bd8
Started: August 14, 2026
Completed: August 14, 2026

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

## 9. Batch E - Public Reference Integration

Planning and Memory are now publicly exported and verified together
against the validated V1 reference configuration.

Verified behavior:

- AStarPlanner is importable from ai_logistics_robot.planning.
- InMemoryMissionMemory is importable from ai_logistics_robot.memory.
- Both concrete implementations structurally satisfy their public ports.
- The reference configuration initializes Planning and Memory inputs.
- The outbound plan begins at the configured initial robot position.
- The deterministic reference goal is the safe arrival cell at (8,6).
- Every reference-plan position is traversable.
- Confirmed outbound positions produce the exact reversed return record.
- Mission and robot identities remain consistent across both components.
- Reset permits an identical deterministic plan and memory replay.
- No Brain, Control, graphical, hardware, or persistence dependency is used.

Verification:

- 4 Planning and Memory reference-integration tests passed.
- The complete suite passed with 233 tests.
- Ruff, mypy, and the project-structure check passed.

## 10. Final Implementation Review

I-0.5 delivers the first concrete Planning and Memory implementations for
the V1 core while preserving the public contracts approved in I-0.3.

### 10.1 Planning Outcome

AStarPlanner is stateless and deterministic. It validates every planning
request before search, uses configured GridMap bounds and traversability,
and returns an immutable versioned PathPlan.

The planner includes the supplied start position, selects the shortest
reachable authorized goal, and uses caller goal order plus GridMap
neighbor order to resolve equal-cost alternatives deterministically.

NoPathError distinguishes a valid but unreachable request from malformed
planning input. Mapping that exception to FailureReason.NO_PATH remains a
Brain responsibility for I-0.6.

### 10.2 Memory Outcome

InMemoryMissionMemory records one mission at a time and exposes immutable
read-only snapshots of active mission, completed mission, confirmed poses,
and ordered events.

Outbound and return histories remain separate. Return preparation reverses
the exact outbound RobotPose sequence without deduplication, heading
replacement, path recalculation, or mutation.

Event identities must match the active mission. Event identifiers remain
unique and sequence numbers strictly increase. Mission completion accepts
only identity-matched terminal outcomes and closes further mutation until
reset.

## 11. Preserved Architectural Boundaries

- Planning reads GridMap but never mutates world state.
- Planning calculates paths but never executes movement.
- Memory stores confirmed facts but never decides mission behavior.
- Memory does not publish Monitoring events.
- DETOUR is a planning phase, not a PathRecord navigation phase.
- Replanning decisions remain outside both concrete components.
- Return detours remain a Brain, Planning, and Control collaboration.
- No Pygame, hardware, database, or network dependency was introduced.
- Both implementations remain replaceable behind their public ports.

## 12. Requirements Contribution

I-0.5 contributes the following V1 behavior:

- FR-06 to FR-11: deterministic route calculation and confirmed-pose
  recording foundations.
- FR-16 to FR-20: reverse confirmed-path preparation for return navigation.
- FR-21 to FR-22: ordered in-memory event retention for reconstruction.
- FR-26: explicit reset permits a new mission without restarting Python.
- NFR-08: the simplest deterministic A* implementation is used.
- NFR-09: mission_id and robot_id remain isolated throughout both modules.
- NFR-10: identical inputs produce identical plans and memory records.

Navigation orchestration, obstacle-triggered replanning, physical return,
base-arrival confirmation, and waiting-state transitions remain assigned
to later implementation drafts.

## 13. Delivered Files

Production additions and changes:

- src/ai_logistics_robot/domain/errors.py
- src/ai_logistics_robot/planning/a_star_planner.py
- src/ai_logistics_robot/planning/__init__.py
- src/ai_logistics_robot/memory/in_memory_mission_memory.py
- src/ai_logistics_robot/memory/__init__.py
- src/ai_logistics_robot/__main__.py
- README.md

Test additions:

- tests/unit/test_a_star_planner.py
- tests/unit/test_a_star_planner_rejections.py
- tests/unit/test_in_memory_mission_memory.py
- tests/unit/test_in_memory_mission_memory_rules.py
- tests/integration/planning_memory/__init__.py
- tests/integration/planning_memory/test_reference_planning_memory.py

Implementation record:

- docs/implementation/draft_I-0.5.md

## 14. Verification Status

Completed before the final quality gate:

- 14 Planning unit tests passed.
- 22 Memory unit tests passed.
- 4 Planning and Memory integration tests passed.
- The complete suite passed with 233 tests.
- Ruff passed for every changed source and test file.
- mypy passed across 41 source files.
- The I-0.1 project-structure check passed.

Final repository-wide quality gate:

- 233 automated tests passed.
- Ruff passed across the complete repository.
- mypy passed across 41 source files.
- The I-0.1 project-structure check passed.
- pip reported no broken requirements.
- The source distribution and wheel built successfully.
- The built wheel contains the new Planning and Memory modules.
- The command-line status entry point reports I-0.5 correctly.
- git diff --check reported no whitespace errors.

## 15. Deferred Work

The following behavior remains outside I-0.5:

- Perception-driven world updates.
- Brain state-machine orchestration.
- Control step construction and command execution.
- Automatic outbound and return replanning decisions.
- Mapping NoPathError to a terminal mission outcome.
- Collection timing and base-arrival confirmation.
- Graphical rendering and interactive simulation.
- Persistent Memory and Monitoring adapters.
- Physical hardware integration.

## 16. Exit Decision

Status: COMPLETE

All five implementation batches and the complete repository quality gate
are complete. I-0.5 is ready for pull-request review and merge into main.
