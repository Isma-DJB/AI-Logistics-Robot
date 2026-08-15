# Implementation Draft I-0.6 - Brain, State Machine, and Control

Status: COMPLETED
Branch: implementation/i-0.6
Base commit: c8de100
Started: August 15, 2026
Completed: August 16, 2026

## 1. Objective

Implementation Draft I-0.6 introduces deterministic mission orchestration
and locally safe command execution through the existing `BrainPort` and
`ControlPort` contracts.

The Brain owns mission decisions and state transitions. Control owns
command execution and the local safety latch. Neither component depends
directly on GridWorld, Pygame, physical hardware, or persistent storage.

## 2. Transition Review

The I-0.5 baseline is synchronized at commit c8de100 and passes all
233 automated tests.

Approved design decisions:

- One `Brain.update()` call performs one state action and at most one
  state transition.
- Only the Brain changes `BrainState` and constructs new immutable
  `Mission` values.
- The Brain receives an immutable `GridMap` reference and never imports
  the concrete GridWorld adapter.
- Control executes commands only through `SimulationPort`.
- Control owns the local safety latch and confirmed execution pose.
- A mission starts only on an `INACTIVE` to `ACTIVE` target edge.
- Scenario-derived counters provide deterministic mission and event
  identifiers.
- The first `PathPlan` position represents the current confirmed pose
  and is not executed as a new movement.
- Only successful confirmed navigation poses are recorded.
- Failed movements preserve the pose and are never recorded.
- A blocked intended cell produces a revised immutable map snapshot
  before replanning.
- `NoPathError` is mapped by the Brain to
  `FailureReason.NO_PATH`.
- Collection uses `Control.stop()` and `ClockPort`.
- Return preparation uses the exact reversed outbound record.
- Return commands are calculated from positions and the current heading;
  recorded outbound headings are not imposed during return.
- `SAFETY_STOP` has priority over every normal transition.
- Emergency stop aborts the active mission and prohibits automatic
  recovery.
- `Brain.reset()` never calls `Control.reset_safety_latch()`.
- Manual safety rearm and Brain reset remain separate operations.
- Movement duration is not invented while movement timing settings are
  absent from the validated configuration.

## 3. Mission-Event Scope Clarification

`MissionEvent` requires a valid `mission_id`, while initialization and
waiting occur before a mission exists.

I-0.6 therefore records ordered events for transitions and actions that
belong to an accepted mission. Pre-mission lifecycle state remains visible
through read-only `SystemStatus`.

No synthetic mission identifier is created. A separate system-event
contract may be reconsidered before the I-0.8 acceptance-test draft.

## 4. Planned Batches

### Batch A - Deterministic Control Execution

Implement command validation, platform execution, confirmed-pose tracking,
normal stop behavior, and protocol compatibility.

### Batch B - Local Safety Latch

Implement emergency stop, latched command rejection, immutable
`SafetyStatus`, idempotent inspection, and explicit manual rearm.

### Batch C - Brain Initialization and Mission Activation

Implement deterministic initialization, stationary waiting, read-only
status, target-edge detection, mission creation, identity counters, and
initial confirmed-pose recording.

### Batch D - Outbound Navigation and Collection

Implement outbound planning, command construction, one-step navigation,
confirmed-pose recording, blocked-movement replanning, safe arrival, and
timed stationary collection.

### Batch E - Return, Events, and Terminal States

Implement reverse-path preparation, return navigation, return replanning,
ordered mission events, successful completion, mission failure, system
error, emergency abortion, and guarded reset behavior.

### Batch F - Public Integration and Closure

Export the concrete implementations, verify the reference configuration,
exercise deterministic mission replay, update public documentation, and
run the complete repository quality gate.

## 5. Failure Classification

- `BLOCKED` triggers replanning while the configured policy permits it.
- Replanning-limit exhaustion produces `MISSION_FAILED` with `BLOCKED`.
- `NoPathError` produces `MISSION_FAILED` with `NO_PATH`.
- Mission timeout produces `MISSION_FAILED` with `TIMEOUT`.
- An unexpected out-of-bounds result from a validated plan produces
  `SYSTEM_ERROR`.
- Communication loss and internal execution failures produce
  `SYSTEM_ERROR`.
- A critical hazard or emergency request produces an `ABORTED` mission
  and `SAFETY_STOP`.
- Commands submitted while latched are rejected with
  `CommandStatus.ABORTED` and `FailureReason.SAFETY_LATCHED`.

## 6. Explicitly Deferred

The following concerns remain outside I-0.6:

- Concrete camera and target-perception adapters.
- General perception-driven map reconstruction.
- Movement speeds and physical command durations.
- The complete application `MissionRunner` loop.
- Pygame rendering and interactive visualization.
- Persistent Monitoring and Memory adapters.
- Network or central-station coordination.
- Multi-robot and multi-target orchestration.
- Physical motor, camera, Arduino, and ESP32 integration.
- A separate pre-mission system-event contract.

## 7. Batch A - Deterministic Control Execution

`SafeRobotControl` now provides platform-independent command execution
through `SimulationPort` and structurally satisfies `ControlPort`.

Implemented behavior:

- Constructor dependencies and the configured robot identity are validated.
- Control begins with an immutable unlatched `SafetyStatus`.
- Initial safety time is obtained from `ClockPort`.
- Valid commands are forwarded unchanged to `SimulationPort`.
- Every result retains the exact supplied `MotionCommand`.
- The confirmed pose is updated only from accepted `CommandResult` values.
- Successive commands must begin at the latest confirmed pose.
- Failed movements preserve the previously confirmed pose.
- A platform pose mismatch raises `InvariantViolationError`.
- Foreign or invalid commands are rejected before reaching the platform.
- Normal stop sends an explicit `STOP` command.
- Command execution does not invent or advance simulated time.
- Safety operations are present for protocol compatibility and receive
  dedicated validation in Batch B.

Verification:

- 8 nominal `SafeRobotControl` tests passed.
- The complete suite passed with 241 tests.
- Ruff and mypy passed.
- The project-structure check passed.
- No GridWorld, graphical, hardware, or network dependency was introduced.

## 8. Batch B - Local Safety Latch

The local safety latch, priority stop chain, rejection behavior, and
manual-rearm boundary are now covered by dedicated tests.

Verified behavior:

- Emergency stop creates a critical latched `SafetyStatus`.
- The emergency reason and `ClockPort` timestamp are preserved.
- Every emergency request sends an explicit priority `STOP`.
- A latched Control rejects normal commands without calling the platform.
- Latched rejection returns `CommandStatus.ABORTED`.
- Latched rejection uses `FailureReason.SAFETY_LATCHED`.
- Rejected commands preserve the latest confirmed pose.
- Normal `STOP` remains available while the latch is active.
- Manual rearm clears only the local latch.
- Manual rearm does not reset, move, or advance the platform.
- Command execution can resume only after explicit rearm.
- The latest confirmed pose survives the complete safety cycle.
- Invalid emergency reasons leave status and platform state unchanged.
- If the priority platform stop raises an exception, local safety remains
  latched and retains the requested failure reason.
- Invalid dependencies and malformed platform results are rejected.
- A result containing a copied command is rejected as an invariant
  violation.

The external validation that a human requested rearm belongs to the future
`MissionRunner`; `SafeRobotControl` exposes the explicit rearm operation
but never invokes it automatically.

Verification:

- 10 dedicated safety-latch tests passed.
- The complete suite passed with 251 tests.
- Ruff and mypy passed.
- The project-structure check passed.
- No production change was required after Batch A.

## 9. Batch C - Brain Initialization and Mission Activation

`DeterministicBrain` now provides the initial orchestration lifecycle,
read-only status, target-edge detection, mission creation, and reset
foundation through the existing public ports.

Implemented behavior:

- The concrete Brain structurally satisfies `BrainPort`.
- Constructor configuration and all injected ports are validated.
- The Brain depends only on domain objects and public ports.
- Initial state is `INITIALIZATION`.
- The first update confirms stationary behavior with `Control.stop()`.
- Initialization performs no perception in the same cycle.
- The following state is `WAITING_FOR_MISSION`.
- Waiting keeps the robot stationary before every observation.
- `Brain.get_status()` returns an immutable `SystemStatus`.
- Status inspection performs no movement, stop, or perception operation.
- The first target observation establishes the activation baseline.
- Starting with an already active target does not create a mission.
- A mission is created only on a later `INACTIVE` to `ACTIVE` edge.
- Accepted missions use deterministic scenario-derived identifiers.
- Mission identifiers remain unique across resets in one Python process.
- Mission, robot, target, base, and target-position identities are preserved.
- A newly accepted mission begins with `MissionStatus.ACTIVE`.
- The initial confirmed pose is recorded as outbound history.
- The first event is stored by Memory and published as the same object.
- Event identifiers and sequence numbers restart for each new mission.
- An active mission cannot accept a second target activation.
- Reset clears temporary Brain and Memory state.
- Reset preserves the monotonic mission counter and Monitoring history.
- Reset never invokes `Control.reset_safety_latch()`.

Verification:

- 7 initialization and activation tests passed.
- The complete suite passed with 258 tests.
- Ruff and mypy passed across 43 source files.
- The project-structure check passed.
- No concrete simulation, rendering, hardware, or persistence dependency
  was introduced.

## 10. Batch D - Outbound Navigation and Collection

`DeterministicBrain` now performs outbound planning, confirmed
one-command navigation cycles, obstacle-driven replanning, safe arrival,
and timed stationary collection.

Implemented behavior:

- `OUTBOUND_PLANNING` creates one plan in its own update cycle.
- Initial outbound plans use `PathPhase.OUTBOUND` and version 1.
- The plan begins at the latest confirmed robot position.
- Planning selects only an authorized safe cell adjacent to the target.
- The first planned position is treated as the current pose and is not
  executed as a new movement.
- Every navigation update performs a new normalized observation.
- At most one movement or turning command is executed per update.
- Commands are derived deterministically from the next position and the
  current confirmed heading.
- Opposite headings are resolved through two deterministic right turns.
- Every successful turn and movement records its confirmed pose.
- Successful forward movement must reach the intended adjacent position.
- Failed movement never changes or records the confirmed pose.
- A `BLOCKED` result adds the intended cell to a revised immutable map.
- Blocked outbound movement enters `OUTBOUND_REPLANNING`.
- Replanning creates a versioned `PathPhase.DETOUR` plan.
- Replanned paths exclude the newly confirmed blocked cell.
- Safe arrival is confirmed only at the selected authorized plan goal.
- Arrival enters `COLLECTION` without waiting in the same update cycle.
- Collection sends an explicit `STOP`.
- Collection waits through `ClockPort` for the configured duration.
- Collection completion creates a new immutable `Mission` value.
- Collection then enters `RETURN_PREPARATION`.
- Navigation, blocking, replanning, arrival, and collection actions
  produce ordered mission events.
- Brain reset restores the original configured map and clears temporary
  navigation state.

The existing activation test was refined because post-activation states
are no longer intentional no-ops. It now verifies that the accepted
mission remains unchanged while orchestration advances into outbound
navigation.

Replanning-limit exhaustion, `NoPathError`, mission timeout, and terminal
failure classification remain assigned to Batch E.

Verification:

- 13 combined activation and outbound tests passed.
- The complete suite passed with 264 tests.
- Ruff and mypy passed across 43 source files.
- The project-structure check passed.
- No concrete GridWorld dependency was introduced into the Brain.

## 11. Batch E1 - Nominal Return Navigation

`DeterministicBrain` now prepares and executes the nominal return from
the exact confirmed outbound history.

Implemented behavior:

- `RETURN_PREPARATION` calls `Memory.build_return_path()`.
- The returned `PathRecord` identity and phase are validated.
- The exact reversed outbound positions form the return plan.
- Return preparation performs no movement in its update cycle.
- The return plan uses `PathPhase.RETURN` and version 1.
- The return plan begins at the current confirmed arrival position.
- The return plan ends at the exact configured mission base.
- Recorded outbound headings are not imposed during return.
- Return commands are recalculated from positions and current heading.
- Opposite headings use two deterministic right turns.
- At most one return command is executed per update.
- Duplicate recorded positions are skipped without unnecessary movement.
- Every successful return turn and movement records a confirmed
  `PathPhase.RETURN` pose.
- Successful forward movement must reach the intended return position.
- Exact base arrival clears the active plan.
- Confirmed base arrival enters `MISSION_COMPLETED`.
- The immutable active mission records confirmed base arrival.
- `base_arrival_confirmed` is stored and published as an ordered event.
- Memory remains open in `MISSION_COMPLETED` so terminal completion can
  record its final event before `Memory.complete()` closes the recording.

Return obstacle handling, terminal mission completion, failure
classification, timeout, system error, and emergency abortion remain in
Batch E2.

Verification:

- 3 nominal return tests passed.
- 16 combined Brain lifecycle tests passed.
- The complete suite passed with 267 tests.
- Ruff and mypy passed across 43 source files.
- The project-structure check passed.
- Nominal return does not call PlanningPort or import GridWorld.

## 12. Batch E2 - Terminal States and Exceptional Behavior

`DeterministicBrain` now handles successful terminal completion,
mission-level failures, technical system errors, return obstacles,
timeouts, and latched emergency abortion.

Implemented behavior:

- `MISSION_COMPLETED` creates a valid immutable successful mission.
- Successful missions require collection and confirmed base arrival.
- The final `mission_completed` event is recorded before Memory closes.
- `Memory.complete()` stores the exact successful mission object.
- Successful completion returns the Brain to stationary waiting.
- A later accepted mission resets closed Memory recording state while
  preserving deterministic mission identity counters.
- `NoPathError` maps to `FailureReason.NO_PATH`.
- No-path outcomes produce `MISSION_FAILED`.
- Replanning attempts are counted across the active mission.
- Replanning-limit exhaustion produces `MISSION_FAILED` with `BLOCKED`.
- Mission timeout is measured from deterministic monotonic acceptance time.
- Timeout has no scattered or invented movement duration.
- Timeout stops movement and completes the mission with `TIMEOUT`.
- Unexpected out-of-bounds execution produces `SYSTEM_ERROR`.
- Communication loss and internal execution failures share the technical
  system-error classification.
- Failed and system-error missions are stored with explicit terminal reasons.
- A blocked return movement preserves the confirmed pose.
- Failed return movement is never recorded as successful.
- The blocked return cell is added to a revised immutable map.
- `RETURN_REPLANNING` creates a versioned detour to the exact base.
- Return detours use `PathPhase.DETOUR`.
- `SAFETY_STOP` remains higher priority than timeout and normal transitions.
- An externally latched emergency aborts the active mission.
- Emergency abortion stores `MissionStatus.ABORTED`.
- The safety reason is preserved in Mission, SystemStatus, and events.
- Manual Control rearm never resumes the aborted mission automatically.
- Brain reset and manual safety rearm remain separate operations.
- Mission-scoped terminal events are recorded before Memory closes.

Verification:

- 7 terminal and exceptional-behavior tests passed.
- 23 combined Brain lifecycle tests passed.
- The complete suite passed with 274 tests.
- Ruff and mypy passed across 43 source files.
- The project-structure check passed.
- Failure handling remains independent of GridWorld, rendering, hardware,
  networking, and persistent storage.

## 13. Batch F - Public Integration and Closure

The concrete Brain and Control implementations are now publicly exported
and verified through the complete reference configuration.

Implemented and verified behavior:

- `ai_logistics_robot.brain` exports `DeterministicBrain`.
- `ai_logistics_robot.control` exports `SafeRobotControl`.
- Both packages define explicit `__all__` contracts.
- Integration loads the real `configs/simulation.yaml`.
- The reference system uses real `GridWorld`, `SafeRobotControl`,
  `AStarPlanner`, and `InMemoryMissionMemory` implementations.
- The configured activation edge creates exactly one mission.
- The robot reaches an authorized safe cell without traversing the target.
- Collection waits for the configured three simulated seconds.
- Return uses the exact reversed confirmed outbound history.
- The robot returns to the exact configured base.
- The final mission has `MissionStatus.SUCCESS`.
- Final safety remains unlatched with no latest error.
- Memory and Monitoring retain identical ordered event objects.
- Two independent assemblies produce identical mission traces.

Final verification:

- 3 reference integration tests passed.
- The complete repository suite passed with 278 tests.
- Ruff passed across the complete repository.
- Mypy passed across 43 source files.
- Project-structure and platform-independence checks passed.
- Dependency validation passed.
- Source and wheel package builds passed.
- The application entry point reports the completed I-0.6 status.
- Git whitespace checks passed.

## 14. I-0.6 Outcome

Implementation Draft I-0.6 is complete.

The milestone now provides:

- Deterministic initialization and target-edge activation.
- Confirmed outbound planning and navigation.
- Dynamic obstacle rejection and bounded replanning.
- Safe adjacent arrival and stationary timed collection.
- Exact reverse-path return and return detours.
- Successful, failed, aborted, and technical terminal outcomes.
- Priority latched safety stop without automatic mission recovery.
- Ordered mission events and confirmed pose histories.
- Read-only system status without control effects.
- Deterministic replay of the complete reference scenario.
- Platform-independent Brain and Control implementations.

Still deferred beyond I-0.6:

- The complete application `MissionRunner`.
- Concrete perception, monitoring, and clock adapters.
- Pygame rendering and interactive controls.
- Physical movement timing and calibrated safety margins.
- Persistent storage and external monitoring destinations.
- Camera, motor, Arduino, and ESP32 integration.
- Network, multi-robot, and multi-target coordination.
- A separate pre-mission system-event contract.
