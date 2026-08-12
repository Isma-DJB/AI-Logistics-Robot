# V1 Implementation — Draft I-0.3

## Session objective

Implement the synchronous public ports that separate the V1 core from
replaceable simulation, visualization, monitoring, clock, and hardware
implementations.

## Scope

- Define the nine approved public ports.
- Use only domain objects and standard-library types in their signatures.
- Make the contracts structurally implementable by adapters.
- Verify the protocols without implementing concrete behavior.

## Contract decisions

- Ports use synchronous `typing.Protocol` contracts.
- Ports are `runtime_checkable` for structural adapter verification.
- A port instance represents its configured robot or active platform.
- `BrainPort.update()` performs one deterministic loop iteration and
  returns `None`.
- `BrainPort.get_status()` is the read-only status operation.
- `PerceptionPort.observe()` returns one immutable
  `PerceptionSnapshot`.
- Planning receives the current world, confirmed start pose, immutable
  authorized goals, mission identity, path phase, and plan version.
- Normal command execution returns a confirmed `CommandResult`.
- Safety operations return the resulting `SafetyStatus`.
- Memory stores the mission, confirmed poses, and events separately from
  Monitoring.
- Monitoring returns ordered immutable event tuples.
- Simulation advances time in seconds and Clock waits against a monotonic
  deadline.
- Renderer receives read-only domain objects and has no control effect.
- No raw dictionary, platform library, callback, or asynchronous type is
  part of a public port.

## Public contract matrix

| Port | Operation | Result |
| --- | --- | --- |
| `BrainPort` | `update()` | `None` |
| `BrainPort` | `get_status()` | `SystemStatus` |
| `BrainPort` | `reset()` | `None` |
| `PerceptionPort` | `observe()` | `PerceptionSnapshot` |
| `PlanningPort` | `create_plan(...)` | `PathPlan` |
| `ControlPort` | `execute_step(command)` | `CommandResult` |
| `ControlPort` | `stop()` | `None` |
| `ControlPort` | `emergency_stop(reason)` | `SafetyStatus` |
| `ControlPort` | `get_safety_status()` | `SafetyStatus` |
| `ControlPort` | `reset_safety_latch()` | `SafetyStatus` |
| `MemoryPort` | `start(mission)` | `None` |
| `MemoryPort` | `record_pose(phase, pose)` | `None` |
| `MemoryPort` | `record_event(event)` | `None` |
| `MemoryPort` | `build_return_path()` | `PathRecord` |
| `MemoryPort` | `complete(mission)` | `None` |
| `MemoryPort` | `reset()` | `None` |
| `SimulationPort` | `reset()` | `None` |
| `SimulationPort` | `read_world()` | `GridMap` |
| `SimulationPort` | `apply_command(command)` | `CommandResult` |
| `SimulationPort` | `advance_time(seconds)` | `None` |
| `MonitoringPort` | `publish(event)` | `None` |
| `MonitoringPort` | `events_for(mission_id)` | `tuple[MissionEvent, ...]` |
| `RendererPort` | `render(world, status)` | `None` |
| `RendererPort` | `display_event(event)` | `None` |
| `ClockPort` | `now()` | timezone-aware `datetime` |
| `ClockPort` | `monotonic()` | `float` |
| `ClockPort` | `wait_until(deadline)` | `None` |

## Planning contract inputs

`PlanningPort.create_plan()` receives:

- `mission_id: str`
- `robot_id: str`
- `start_pose: RobotPose`
- `authorized_goals: tuple[Position, ...]`
- `world: GridMap`
- `phase: PathPhase`
- `version: int`

The returned `PathPlan` remains responsible for representing the selected
authorized goal and the versioned planned positions.

## Approved implementation order

1. Clock, Perception, and Planning.
2. Control and Simulation.
3. Memory and Monitoring.
4. Brain, Renderer, and package exports.
5. Full protocol and dependency-boundary verification.

## Intentionally not implemented

- Concrete simulation or hardware adapters.
- Brain orchestration and state transitions.
- Planning algorithms.
- Memory storage behavior.
- Monitoring or rendering behavior.
- MissionRunner execution.

## Verification strategy

```bash
python -m unittest discover -s tests -p "test_*.py" -v
python -m ruff check .
python -m mypy src
python tools/check_project_structure.py
```

## Current status

Implementation Draft I-0.3 started from the validated and merged I-0.2
baseline.