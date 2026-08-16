# Implementation Draft I-0.8 - Complete Software Scenarios and Acceptance Tests

Status: COMPLETE
Branch: implementation/i-0.8
Base commit: f002ad3
Started: August 16, 2026
Completed: August 16, 2026

## 1. Objective

Implementation Draft I-0.8 completes the simulation-side V1 software by
introducing the application execution loop, deterministic supporting adapters,
reference dependency assembly, and complete acceptance scenarios.

This draft verifies all twelve approved acceptance criteria without requiring
camera, motor, microcontroller, network, or other physical hardware.

The Brain remains the only component that makes mission decisions.
`MissionRunner` coordinates application operations without duplicating the
state machine or interpreting perception, planning, control, or safety results.

## 2. Transition Review

The I-0.7 baseline is synchronized on `main` at merge commit `f002ad3`.

The baseline provides:

- immutable validated domain values and application settings;
- final public V1 ports;
- deterministic `GridWorld`;
- deterministic A* planning;
- immutable mission memory;
- safety-aware Control;
- complete Brain orchestration;
- passive Pygame visualization;
- 310 passing automated tests;
- successful Ruff, mypy, CodeQL, structure, dependency, and package checks.

The following application files remain placeholders:

- `app/bootstrap.py`;
- `app/mission_runner.py`.

The simulation acceptance directory contains no implemented scenario.
Reference test doubles currently duplicate clock, perception, and monitoring
behavior inside one integration test.

I-0.8 replaces those temporary test-local implementations with reusable
deterministic adapters and exercises the complete public application boundary.

## 3. Requirements Covered

I-0.8 verifies the complete software coverage of:

- CMP-01 through CMP-06;
- FR-01 through FR-27;
- NFR-01 through NFR-10;
- AC-01 through AC-12;
- SEQ-01 nominal mission;
- SEQ-02 obstacle and replanning;
- SEQ-03 emergency stop and manual safety rearm.

CMP-07, multi-robot coordination, remains assigned to V2.

## 4. Approved Application-Loop Decisions

The following decisions define the I-0.8 application loop:

- `MissionRunner` belongs to `ai_logistics_robot.app`.
- `MissionRunner` depends on public ports and validated domain values.
- `bootstrap.py` is the only application module that knows all concrete
  implementations.
- `configure()` accepts only one validated `Settings` instance.
- `start()` owns the synchronous application loop.
- One loop cycle performs at most one `Brain.update()`.
- Each cycle reads one immutable `SystemStatus`.
- Each cycle passively renders the current world and status.
- Newly published mission events are displayed once in sequence order.
- The runner never plans, constructs motion commands, changes Brain state, or
  interprets navigation outcomes.
- An optional explicit positive cycle bound supports finite deterministic tests.
- The cycle bound is a runner invocation guard, not operational configuration.
- Reaching a cycle bound preserves domain state and invents no mission outcome.
- An unbounded start continues until an explicit stop or priority safety stop.
- A normal stop halts the runner and requests one controlled Control stop.
- A priority emergency request latches Control before any further normal cycle.
- The Brain then observes the latch and owns the mission-abort transition.
- A safety stop terminates the active runner invocation.
- A previous aborted mission is never resumed.
- Manual safety rearm and application reset remain separate operations.
- The runner never rearms safety automatically.
- `get_status()` remains read-only and causes no Control effect.

## 5. Reset and Replay Rules

`MissionRunner.reset()` supports FR-26 without restarting Python.

Reset is permitted only when:

- the runner is not active;
- the local safety latch is not set;
- the confirmed pose equals the configured initial pose.

A permitted reset restores deterministic simulation state and clears temporary
Brain and Memory state. It does not reset the process-wide mission identity
counter and does not delete previously published monitoring history.

A reset never:

- rearms a safety latch;
- resumes an aborted mission;
- guesses or teleports a physical pose;
- changes the configured world;
- creates a synthetic mission identifier.

After an emergency stop away from the base, manual rearm alone is insufficient
for reset. Physical repositioning or a newly assembled simulation scenario is
required before another mission begins.

Premission initialization and waiting remain visible through `SystemStatus`.
No separate premission event type or synthetic mission event is introduced.

## 6. Deterministic Supporting Adapters

I-0.8 introduces reusable simulation-side adapters.

### 6.1 Simulated Clock

The simulated clock:

- structurally satisfies `ClockPort`;
- uses one timezone-aware configured epoch;
- derives monotonic time from `GridWorld`;
- advances `GridWorld` only through `wait_until()`;
- rejects backward or invalid deadlines;
- returns to its configured epoch when `GridWorld` is reset.

No second independent simulation clock is created.

### 6.2 GridWorld Perception

The simulation perception adapter:

- structurally satisfies `PerceptionPort`;
- reports the current confirmed `GridWorld` pose;
- uses the configured deterministic clock;
- exposes explicit simulation input setters for target, hazard, and immutable
  observations;
- produces one immutable `PerceptionSnapshot` per observation;
- makes no mission or control decision;
- introduces no camera or confidence value not supplied by the scenario.

Target and hazard setters are simulation controls, not additions to
`PerceptionPort`.

### 6.3 In-Memory Monitoring

The monitoring adapter:

- structurally satisfies `MonitoringPort`;
- accepts only immutable `MissionEvent` values;
- preserves publication order;
- rejects duplicate event identities;
- preserves increasing sequence order within each mission;
- returns immutable mission-specific event tuples;
- never changes or interprets an event.

Monitoring history may contain several completed missions and therefore remains
available across a permitted application reset.

### 6.4 Headless Rendering

A passive headless renderer supports acceptance execution without creating an
SDL window.

It:

- structurally satisfies `RendererPort`;
- retains immutable rendered snapshots and displayed events for verification;
- performs no Control, Brain, Planning, Memory, or Simulation operation;
- requires no Pygame import.

Pygame remains selected lazily only when graphical rendering is enabled.

## 7. Reference Bootstrap

The reference bootstrap will assemble:

- validated `Settings`;
- `GridWorld`;
- simulated clock;
- GridWorld perception;
- A* planner;
- `SafeRobotControl`;
- `InMemoryMissionMemory`;
- in-memory monitoring;
- `DeterministicBrain`;
- the selected passive renderer;
- `MissionRunner`.

The assembled application exposes the scenario-facing simulation adapters as
read-only component references so acceptance tests can change external target
and hazard inputs without reaching into Brain internals.

Only one platform and one robot remain active in one V1 application assembly.

## 8. Planned Batches

### Batch A - Deterministic Supporting Adapters

Implement the simulated clock, GridWorld perception, in-memory monitoring, and
headless renderer with validation, protocol, reset, ordering, and passivity
tests.

### Batch B - MissionRunner Lifecycle

Implement configuration, bounded and unbounded loop execution, status
inspection, normal stop, emergency stop, explicit safety rearm, guarded reset,
event forwarding, and invalid-state rejection.

### Batch C - Reference Bootstrap Assembly

Implement one validated simulation application assembly while preserving lazy
Pygame isolation and public component boundaries.

### Batch D - Complete Acceptance Scenarios

Implement end-to-end scenario tests covering all twelve acceptance criteria,
the three approved sequences, repeated missions, deterministic replay,
replanning, return detours, event reconstruction, and latched safety.

### Batch E - Public Software Closure

Export the application interfaces, remove duplicated test-local adapters,
update public documentation and command-line status, verify package contents,
and run the complete repository quality gate.

## 9. Acceptance Scenario Matrix

### Nominal Scenario

The nominal scenario verifies:

- AC-01 stationary behavior while inactive;
- AC-02 one mission per activation edge;
- AC-03 collision-free outbound navigation;
- AC-05 authorized arrival;
- AC-06 stationary timed collection;
- AC-07 confirmed return to base;
- AC-09 reconstructable states and events;
- AC-11 successive missions without restarting Python;
- AC-12 complete GridWorld execution without hardware.

### Obstacle and Replanning Scenario

The obstacle scenario verifies:

- AC-04 rejected movement and replanning;
- AC-08 reversed confirmed path and optional safe detour;
- unchanged pose after rejected movement;
- omission of failed movement from confirmed path memory;
- versioned outbound or return detour plans.

### Emergency-Stop Scenario

The emergency scenario verifies:

- AC-10 priority latched stop;
- immediate controlled stop before further normal motion;
- aborted mission with an explicit reason;
- mandatory manual safety rearm;
- separation of rearm and reset;
- prohibition of automatic mission resumption.

### Deterministic Replay

Repeated executions verify:

- NFR-10 identical behavior from identical configured inputs;
- deterministic paths, commands, statuses, events, deadlines, and outcomes;
- reset and new mission operation without restarting Python.

## 10. Explicitly Deferred

The following concerns remain outside I-0.8:

- physical camera and target detection;
- hardware debounce and confidence calibration;
- physical motor timing and speed configuration;
- measured robot footprint and safety margin;
- Arduino, ESP32, ESP32-CAM, and serial or wireless communication;
- power-supply, sensor, motor, and camera diagnostics;
- physical coordinate and turning calibration;
- persistent external monitoring destinations;
- keyboard or mouse robot control;
- network coordination;
- multi-robot and multi-target behavior;
- physical V1 validation.

Hardware diagnostics and calibration belong to I-0.9.
Physical integration and final V1 validation belong to I-1.0.

## 11. Exit Conditions

I-0.8 is complete when:

- reusable deterministic adapters satisfy their public ports;
- `MissionRunner` implements every approved application operation;
- application reset cannot bypass manual safety rearm;
- reset cannot create an unconfirmed pose;
- the bootstrap assembles the complete reference simulation;
- all AC-01 through AC-12 tests pass;
- SEQ-01 through SEQ-03 pass end to end;
- two missions run without restarting Python;
- identical inputs replay deterministically;
- the headless software stack imports and runs without hardware;
- Pygame remains isolated as an optional simulation dependency;
- the public status accurately reports complete simulation-side V1 software;
- the full repository quality gate passes.
## 12. Batch A - Deterministic Supporting Adapters

Status: COMPLETE

Batch A replaced the test-local simulation support with four reusable passive
adapters.

`SimulatedClock` now derives both wall-clock and monotonic time from the single
confirmed `GridWorld` elapsed-time value. Waiting advances only the required
non-negative duration. Invalid, non-finite, backward, and unsupported deadlines
are rejected before simulation state changes. Resetting `GridWorld` restores
the configured clock epoch without a second mutable clock.

`GridWorldPerception` now produces immutable normalized snapshots from the
confirmed simulated pose and deterministic clock. Explicit setters represent
external target, hazard, and observation inputs without adding operations to
`PerceptionPort` or making mission decisions.

`InMemoryMonitoring` now preserves immutable multi-mission publication history.
It rejects invalid events, duplicate event identities, and non-increasing
sequence numbers within one mission. Mission-specific reads retain publication
order and expose immutable tuples.

`HeadlessRenderer` now retains passive world, status, and event inputs for
acceptance verification without opening an SDL display or importing Pygame.

All four implementations structurally satisfy their approved public ports and
are exported through their adapter packages.

Batch A verification established that:

- the initial tests failed only because the four adapters were absent;
- 24 focused adapter tests pass;
- the complete repository suite passes 334 automated tests;
- Ruff passes for every affected implementation and test file;
- mypy passes across 48 source files;
- the project-structure check passes;
- `pip check` reports no broken requirements;
- headless adapter import does not import Pygame;
- core packages remain free of platform-specific imports.
## 13. Batch B - MissionRunner Lifecycle

Status: COMPLETE

Batch B implemented the deterministic application execution loop behind the
public application boundary.

`MissionRunner` now:

- validates all Brain, Control, Simulation, Monitoring, and Renderer ports;
- accepts one validated `Settings` instance before execution;
- rejects a simulation world that differs from the configured immutable world;
- confirms the configured robot identity and initial pose through
  `SystemStatus`;
- executes at most one `Brain.update()` per loop cycle;
- supports bounded deterministic execution without inventing a terminal result;
- supports unbounded execution until an explicit normal stop or safety stop;
- renders one immutable world and status snapshot after every completed cycle;
- forwards newly published mission events once in increasing sequence order;
- exposes read-only status inspection without execution or rendering effects;
- performs a priority emergency stop before the Brain safety transition;
- terminates an active invocation after a latched safety stop;
- requires explicit manual safety rearm;
- keeps safety rearm separate from application reset;
- prevents restart from a previous safety-stop state until reset;
- rejects reset while active or while the safety latch remains set;
- permits reset only from the configured confirmed initial pose;
- resets Simulation and Brain temporary state without clearing monitoring
  history;
- never plans, constructs motion commands, interprets navigation outcomes, or
  changes Brain state directly.

The runner depends only on validated settings, immutable domain values, and
public ports. `SystemStatus` remains the authoritative application view of the
confirmed pose. The runner does not access a concrete `GridWorld.current_pose`
extension that is absent from `SimulationPort`.

Batch B verification also corrected two test assumptions:

- status snapshots are compared by immutable value because a Brain may return a
  fresh read-only snapshot for every `get_status()` call;
- the reset rejection fixture now derives a position guaranteed to differ from
  the configured initial position instead of assuming that `(1, 1)` is
  different.

The initial implementation briefly accessed `current_pose` through the static
`SimulationPort` type. mypy exposed that architectural violation, and the
access was removed rather than widening the public port or suppressing the
type error.

Batch B verification established that:

- the initial test failed only because `MissionRunner` was absent;
- all 14 focused MissionRunner lifecycle tests pass;
- bounded and explicitly stopped execution remain deterministic;
- emergency-stop ordering is verified directly;
- new monitoring events are displayed exactly once;
- safety rearm cannot implicitly reset or resume an aborted mission;
- reset cannot bypass the safety latch or an unconfirmed pose;
- the complete repository suite passes 348 automated tests;
- Ruff passes for every affected application and test file;
- mypy passes across 48 source files;
- the project-structure check confirms the platform-independent core boundary;
- `pip check` reports no broken requirements.
## 14. Batch C - Reference Bootstrap Assembly

Status: COMPLETE

Batch C implemented the complete deterministic simulation dependency assembly.

`build_simulation_application()` now validates one `Settings` instance and
constructs:

- the configured immutable `GridMap` inside a fresh `GridWorld`;
- one `SimulatedClock` backed by that `GridWorld`;
- one externally controlled `GridWorldPerception`;
- one deterministic `AStarPlanner`;
- one safety-aware `SafeRobotControl`;
- one fresh `InMemoryMissionMemory`;
- one multi-mission `InMemoryMonitoring` history;
- one complete `DeterministicBrain`;
- one passive renderer selected from validated settings;
- one configured `MissionRunner`.

The caller supplies one timezone-aware simulation epoch explicitly. This keeps
wall-clock replay deterministic without adding an unvalidated current-time
dependency or a second simulation clock.

`SimulationApplication` exposes the assembled component references through one
frozen, slotted application record. Scenario tests can control target, hazard,
observations, and execution through public component operations without
reaching into Brain internals or replacing assembled dependencies.

Every call creates isolated Simulation, Clock, Perception, Control, Memory,
Monitoring, Brain, Renderer, and Runner state. Only the immutable validated
settings values are shared.

Renderer selection remains passive and configuration-driven:

- disabled visualization creates `HeadlessRenderer`;
- enabled visualization lazily imports and creates `PygameRenderer`;
- importing or assembling the headless application does not import Pygame;
- no graphical dependency leaks into the platform-independent application
  execution path.

`bootstrap.py` is the application composition root and is therefore the only
application module that knows all reference concrete implementations.
`MissionRunner` continues to depend only on public ports.

Batch C verification established that:

- the initial test failed only because the public bootstrap was absent;
- all 8 focused bootstrap tests pass;
- every assembled component satisfies its approved public port;
- component state matches the validated settings and explicit epoch;
- graphical and headless renderer selection is deterministic;
- separate assemblies contain isolated mutable state;
- application component references are read-only;
- invalid settings are rejected with `DomainValidationError`;
- independent headless assembly does not import Pygame;
- the complete repository suite passes 356 automated tests;
- Ruff passes for every affected application and test file;
- mypy passes across 48 source files;
- the project-structure check confirms the platform-independent core boundary;
- `pip check` reports no broken requirements.
## 15. Batch D - Complete Acceptance Scenarios

Status: COMPLETE

Batch D verified the complete V1 software behavior through the public
application boundary using the assembled deterministic headless application.

`GridWorld` now supports immutable scenario-controlled transient obstacles.
These obstacles represent unplanned obstructions that were not present in the
validated configuration or the Brain's current planning map. They:

- remain separate from the configured immutable `GridMap`;
- appear in the immutable world returned by `read_world()`;
- reject an attempted forward movement atomically with `BLOCKED`;
- preserve the confirmed robot pose after rejection;
- may be replaced deterministically by the scenario;
- are cleared by an explicit simulation reset;
- cannot overlap the robot, base, target, configured obstacles, or cells
  outside the world.

This scenario control remains a concrete `GridWorld` testing extension. It does
not widen `SimulationPort` or expose platform-specific behavior to the Brain.

The complete application scenarios verify that a real rejected movement causes
the Brain to preserve its confirmed pose, add the newly observed obstruction to
its immutable planning world, and request a versioned `DETOUR` plan. Both
outbound and return replanning avoid the transient obstacle and complete
without recording a rejected pose.

The acceptance review exposed one additional lifecycle invariant. Returning to
the base cell is insufficient when the robot heading differs from the
configured initial heading. `DeterministicBrain` now performs controlled
quarter-turn commands, at most one command per update cycle, until the complete
confirmed pose matches the configured initial pose. Mission completion and a
subsequent guarded reset therefore cannot silently change robot orientation.

The legacy return test was strengthened to verify the final forward movement,
the deterministic two-turn heading alignment, and equality with the complete
configured initial pose.

Six complete deterministic scenarios now verify:

- SEQ-01 nominal mission execution;
- SEQ-02 outbound obstruction, rejection, and detour replanning;
- SEQ-02 exact reversed return path with an optional safe detour;
- SEQ-03 hazard detection, priority emergency stop, and manual rearm rules;
- two successful missions in one Python process without restarting;
- identical complete replay for identical settings, epoch, and inputs.

Together these scenarios cover AC-01 through AC-12. They verify inactive target
stationarity, one mission per activation edge, collision-free navigation,
authorized target adjacency, stationary collection, confirmed return to base,
ordered immutable events, latched safety behavior, multi-mission isolation, and
complete operation without physical hardware.

Monitoring history persists across the guarded reset between missions while
temporary Brain, Memory, Simulation, Clock, Perception, and Runner state is
restored. Mission identifiers advance deterministically and event sequence
numbers restart within each new mission.

The acceptance package root now contains an explicit package marker so standard
repository discovery includes every acceptance scenario rather than requiring
a separate direct invocation.

Batch D verification established that:

- all 6 transient-obstacle unit tests pass;
- all 6 complete software acceptance scenarios pass;
- all 3 focused return-navigation regression tests pass;
- 12 new Batch D tests are included in repository discovery;
- the complete repository suite discovers and passes 368 tests;
- all AC-01 through AC-12 are covered by executable scenarios;
- SEQ-01, SEQ-02, and SEQ-03 are verified through the application boundary;
- NFR-10 deterministic replay is verified with complete comparable traces;
- Ruff passes for every affected implementation and test file;
- mypy passes across 48 source files;
- the project-structure check confirms the platform-independent core boundary;
- `pip check` reports no broken requirements.
## 16. Batch E - Public Software Closure

Status: COMPLETE

Batch E closed the public simulation-side V1 software milestone.

The public application package exports:

- `MissionRunner`;
- `SimulationApplication`;
- `build_simulation_application()`.

The project metadata continues to expose the
`ai-logistics-robot = ai_logistics_robot.__main__:main` console script.
Both the installed console script and `python -m ai_logistics_robot` now report
Implementation Draft I-0.8 and accurately distinguish complete simulation-side
software from the physical work assigned to I-0.9 and I-1.0.

`README.md` now documents:

- the complete I-0.8 software capability;
- all public application interfaces;
- AC-01 through AC-12 and SEQ-01 through SEQ-03 coverage;
- deterministic replay and repeated mission support;
- the optional Pygame boundary;
- the 370-test repository suite;
- the public status commands;
- the remaining hardware diagnostics, calibration, and integration phases.

The old reference mission integration contained local implementations of a
clock, simulation wrapper, perception adapter, and monitoring adapter. Those
duplicates were removed. The integration now uses the public reference
bootstrap and the real `SimulatedClock`, `GridWorld`, `GridWorldPerception`,
`InMemoryMonitoring`, `HeadlessRenderer`, `SafeRobotControl`,
`DeterministicBrain`, and `MissionRunner`.

Unit-level test doubles remain intentionally local where they isolate one
component, inject invalid results, record call order, or verify structural port
behavior. Only the obsolete integration-level replacement implementations were
removed.

Command-line verification was also removed from the Pygame integration test.
Public status behavior now belongs exclusively to the dedicated command-line
test module, while the Pygame integration retains only graphical reference
verification.

Package verification established that:

- one source distribution and one universal wheel build successfully;
- the source distribution contains the README, project metadata, and public
  source modules;
- the wheel contains the complete application, adapters, Brain, Control,
  domain, memory, planning, and ports packages;
- the wheel contains the declared console entry point;
- a clean temporary environment installs the wheel and its declared `PyYAML`
  dependency;
- `pip check` passes inside that clean environment;
- the installed application interfaces import without importing Pygame;
- both installed command-line entry points return success.

The first clean-environment smoke attempt intentionally installed the wheel
with `--no-deps`. Importing the public application then correctly failed because
the required `PyYAML` dependency was absent. The verification procedure was
corrected to install declared dependencies normally. No package source or
dependency declaration was defective.

Batch E verification established that:

- all 4 dedicated public-status tests pass;
- all 3 reusable reference-application integration tests pass;
- the focused Pygame reference integration remains passive and passes;
- no obsolete reference integration adapter remains;
- the complete repository suite discovers and passes 370 tests;
- Ruff passes across the complete repository;
- mypy passes across 48 source files;
- the project-structure check passes;
- the headless core remains free of direct platform-specific imports;
- repository and clean-environment dependency checks pass;
- wheel and source-distribution contents are complete;
- the public source and installed status commands report I-0.8 accurately.

Every I-0.8 exit condition is satisfied. Complete simulation-side V1 behavior
is implemented and executable without hardware. Hardware diagnostics and
calibration proceed in I-0.9, followed by physical integration and final V1
validation in I-1.0.
