# Implementation Draft I-0.7 - PygameRenderer

Status: IN PROGRESS
Branch: implementation/i-0.7
Base commit: 5136ff7
Started: August 16, 2026

## 1. Objective

Implementation Draft I-0.7 introduces a passive Pygame visualization adapter
through the existing `RendererPort` contract.

The renderer visualizes immutable `GridMap`, `SystemStatus`, and `MissionEvent`
values. It never plans movement, executes commands, changes mission state,
modifies confirmed facts, or makes control decisions.

Pygame remains an optional simulation dependency. Headless GridWorld behavior
and every core package must continue to work without importing Pygame.

## 2. Transition Review

The I-0.6 baseline and the public community standards are synchronized at
commit `5136ff7`.

The complete baseline passes 278 automated tests. Ruff, mypy, the project
structure check, `pip check`, and the command-line status entry point also pass.

Existing contracts and data provide:

- immutable grid dimensions, origin, base, target, obstacles, and layers;
- deterministic authorized arrival positions;
- immutable robot position and cardinal heading;
- an optional versioned active plan;
- Brain state, mission identity, mission status, and latest failure;
- immutable local safety status;
- ordered immutable mission events;
- an effect-free `RendererPort` with `render` and `display_event`.

No graphical settings, concrete visualization implementation, or rendering
tests exist in the baseline.

## 3. Approved Design Decisions

The following decisions define I-0.7:

- `PygameRenderer` belongs to `adapters.visualization`.
- Pygame remains available only through the `simulation` optional dependency.
- Core packages and ports never import Pygame.
- `render()` consumes one world and status snapshot without mutating either.
- `display_event()` accepts one already-decided mission event.
- Rendering does not call Brain, Planning, Control, Memory, or Simulation.
- Closing the window closes only the visualization adapter.
- A closed renderer performs deterministic no-op render and event operations.
- Renderer shutdown is idempotent.
- Pygame event processing cannot change mission or control state.
- Grid coordinates support arbitrary configured origins.
- Increasing domain `y` is displayed upward on screen.
- The active path is an informational overlay only.
- Event history is bounded and used only for display.
- No image, font, network, or platform-specific asset is required.
- Headless automated tests use the SDL dummy video driver.

## 4. Renderer Configuration

A new immutable `RendererSettings` object will be loaded from the reference
YAML configuration.

The renderer section will define:

- `enabled`;
- `window_title`;
- `cell_size_px`;
- `status_panel_width_px`;
- `frames_per_second`;
- `recent_event_limit`.

The initial reference values are:

- enabled: `true`;
- window title: `AI-Logistics-Robot`;
- cell size: `64` pixels;
- status-panel width: `360` pixels;
- frame rate: `30` frames per second;
- recent-event limit: `6`.

Grid width and height remain derived from `GridMap`. Physical
`cell_size_cm` and graphical `cell_size_px` remain separate concepts.

Colors, line widths, and internal padding are centralized presentation
constants rather than operational configuration.

## 5. Planned Batches

### Batch A - Renderer Configuration

Implement `RendererSettings`, YAML loading, validation, immutability, reference
values, and rejection tests.

### Batch B - Deterministic World Rendering

Implement Pygame initialization, origin-aware coordinate conversion, grid
cells, base, target, obstacles, authorized arrival cells, robot position, and
cardinal heading.

### Batch C - Status, Plan, and Event Visualization

Render the active path and goal, Brain state, mission information, confirmed
pose, safety latch, latest error, and a bounded list of recent mission events.

### Batch D - Passive Lifecycle and Failure Rules

Verify runtime protocol compatibility, invalid public inputs, SDL quit
handling, idempotent close, render-after-close behavior, repeatability, and
absence of domain mutation.

### Batch E - Public Reference Integration and Closure

Export the concrete adapter, verify it against the V1 reference configuration,
update public documentation and the command-line milestone, and run the
complete repository quality gate.

## 6. Visual Semantics

The initial renderer will use the following visual distinctions:

- neutral cells for traversable grid positions;
- dark blocked cells for obstacles;
- a distinct base cell;
- a distinct non-traversable target cell;
- highlighted safe arrival cells;
- an ordered active-plan overlay;
- a directional robot marker;
- a side panel for state, mission, safety, error, and events.

These visual distinctions communicate confirmed state only. They do not add
new domain semantics.

## 7. Explicitly Deferred

The following concerns remain outside I-0.7:

- MissionRunner and the complete application loop;
- keyboard or mouse control of the robot;
- renderer-driven mission stop or reset;
- concrete Perception, Monitoring, and Clock adapters;
- scenario orchestration and acceptance tests;
- automatic screenshots or video recording;
- editable graphical configuration;
- external graphical assets and custom fonts;
- physical camera, microcontroller, and motor integration;
- hardware diagnostics and calibration.

These concerns remain assigned to I-0.8, I-0.9, I-1.0, or later versions.

## 8. Exit Conditions

I-0.7 is complete when:

- the renderer structurally satisfies `RendererPort`;
- every configured visual value is validated;
- the reference world and status render successfully in headless tests;
- shifted origins and all headings render deterministically;
- active plans and recent events are visible;
- closing the renderer cannot alter control state;
- Pygame remains isolated from the core;
- the full repository quality gate passes;
- public documentation accurately reports the I-0.7 milestone.
## 9. Batch A - Renderer Configuration

Status: COMPLETE

Batch A introduced the immutable `RendererSettings` configuration object and
integrated it into the validated application `Settings`.

The reference configuration now defines:

- whether visualization is enabled;
- the window title;
- the graphical cell size in pixels;
- the status-panel width;
- the maximum frame rate;
- the bounded recent-event display limit.

All renderer settings are required by the reference YAML configuration.
Boolean, textual, dimensional, frame-rate, and event-limit values are
validated explicitly. Invalid types, non-positive integers, blank titles, and
missing renderer configuration are rejected with `DomainValidationError`.

Physical `GridMap.cell_size_cm` remains independent from graphical
`RendererSettings.cell_size_px`.

Batch A verification established that:

- the initial test failed because `RendererSettings` did not yet exist;
- 28 focused settings and configuration-loading tests pass;
- the complete repository suite passes 288 automated tests;
- Ruff passes for the affected implementation and test files;
- mypy passes across 43 source files;
- the project-structure check passes;
- `pip check` reports no broken requirements;
- no Pygame import was introduced during the configuration batch.
## 10. Batch B - Deterministic World Rendering

Status: COMPLETE

Batch B introduced the concrete passive `PygameRenderer` adapter.

The renderer now:

- initializes its SDL display lazily on the first render operation;
- derives window dimensions from the immutable world and renderer settings;
- supports arbitrary configured grid origins;
- maps increasing domain `y` coordinates upward on screen;
- distinguishes traversable cells, obstacles, the base, the target, and
  authorized arrival cells;
- displays the confirmed robot position;
- displays all four cardinal headings deterministically;
- produces identical pixels for repeated identical snapshots;
- processes SDL close events without issuing control or mission commands;
- releases display resources idempotently.

Pygame is imported only by
`adapters.visualization.pygame_renderer`. Core packages remain independent of
the optional graphical platform.

The installed Pygame annotations import NumPy 2.5.1 stubs containing syntax
that requires a Python 3.12 parser, while the project intentionally retains
Python 3.11 as its minimum target. A scoped mypy override skips only the
transitive NumPy stubs. Strict checking remains active for all project source
files and for the Pygame renderer itself.

Batch B verification established that:

- the initial test failed because `PygameRenderer` did not yet exist;
- all five deterministic world-rendering tests pass with the SDL dummy driver;
- the complete repository suite passes 293 automated tests;
- Ruff passes across the complete repository;
- mypy passes across 44 source files;
- the project-structure check confirms that core packages contain no
  platform-specific imports;
- `pip check` reports no broken requirements.
## 11. Batch C - Status, Plan, and Event Visualization

Status: COMPLETE

Batch C completed the informational visualization required by FR-25.

The active path is displayed as a passive ordered overlay. Its authorized goal
has a distinct marker, and the confirmed robot marker remains visually
dominant at the current pose.

The status panel now displays:

- the robot identity;
- the current Brain state;
- the confirmed position and heading;
- the mission identity and lifecycle status;
- the active path phase and version;
- the confirmed local safety-latch state;
- the latest normalized error;
- a bounded list of recent ordered mission events.

Safe and latched safety states are distinguished explicitly. A latched state
includes its normalized failure reason, while the latest system error is
displayed separately.

`display_event()` stores only the bounded passive display history configured by
`recent_event_limit`. It does not publish, persist, reorder, or interpret
events. Read-only tuple views expose the most recent events and the
deterministically prepared panel lines for verification.

Batch C verification established that:

- all five initial tests failed only because the planned display features were
  absent;
- the active path and goal use distinct deterministic overlays;
- confirmed active-mission status is represented accurately;
- safety-stop, aborted-mission, and emergency-error information remain
  distinguishable;
- old events are removed when the configured display limit is exceeded;
- recent events appear in deterministic order;
- all ten Pygame world and status tests pass;
- the complete repository suite passes 298 automated tests;
- Ruff passes for all affected files;
- mypy passes across 44 source files;
- the project-structure check and `pip check` pass.
