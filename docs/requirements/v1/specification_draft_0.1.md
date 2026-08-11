**AI-LOGISTICS-ROBOT**

**Requirements Specification
Version 1**

Autonomous Logistics Robot — Simple End-to-End Mission

| **Project**  | AI-Logistics-Robot                                       |
|--------------|----------------------------------------------------------|
| **Document** | Functional and Non-Functional Requirements Specification |
| **Version**  | Draft 0.1                                                |
| **Date**     | August 10, 2026                                          |

*Status: reference document to be approved before coding begins*

# 1. Purpose of This Document

This document defines the objectives, scope, expected behavior, functional architecture, requirements, and acceptance criteria for Version 1 of the AI-Logistics-Robot project. It is the reference contract to be approved before any code is written.

| **Guiding principle —** V1 must demonstrate a complete autonomous mission using a simple, modular, testable, and extensible architecture. |
|-------------------------------------------------------------------------------------------------------------------------------------------|

# 2. V1 Vision and Overall Objective

V1 must demonstrate that a logistics robot can independently complete a simple end-to-end mission:

| **V1 mission —** A light source triggers a mission. The robot travels to the target zone while avoiding obstacles, stops nearby, simulates a collection operation, and then returns to its starting point along the path it actually followed. |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

The success of this version does not depend on complex artificial intelligence. It depends on reliable behavior, clear responsibilities between modules, and the ability to test the logic before hardware integration.

# 3. Nominal Scenario

1.  The robot starts in its resting zone.

2.  It checks that its essential components are operational.

3.  It waits for a light target to become active.

4.  It detects or receives the position of that target.

5.  It accepts the mission.

6.  It travels toward the target.

7.  It detects and avoids obstacles encountered along the way.

8.  It records enough of its path to be able to return.

9.  It stops at a safe distance from the target.

10. It waits for a defined period to simulate collection.

11. It performs the return journey.

12. It reaches its starting zone.

13. It completes the mission and becomes available again.

The return journey must use the path actually followed outbound, including detours caused by obstacles.

# 4. State Machine

V1 behavior will be controlled by an explicit state machine. Every transition must be triggered by an identifiable condition and recorded in the logs.

| **State**            | **Responsibility**                                             |
|----------------------|----------------------------------------------------------------|
| INITIALIZATION       | Check system status and prepare the modules.                   |
| WAITING_FOR_MISSION  | Robot stationary and ready to receive a mission.               |
| OUTBOUND_NAVIGATION  | Autonomous progress toward the target.                         |
| OBSTACLE_AVOIDANCE   | Stop or perform a local maneuver to avoid a collision.         |
| SIMULATED_COLLECTION | Remain stationary near the target for a configurable duration. |
| RETURN_NAVIGATION    | Follow the recorded path in reverse toward the base.           |
| MISSION_COMPLETED    | Complete the mission and record its result.                    |
| ERROR                | Unrecoverable failure requiring a stop and diagnosis.          |
| SAFETY_STOP          | Immediately stop all movement in the event of danger.          |

# 5. Functional Architecture

The system is organized into independent modules connected through defined interfaces. The main loop is: sensors or simulation → perception → decision → planning/navigation → control → robot or simulator. Mission memory exchanges information with the decision module.

| **Module**            | **Responsibility in V1**                                                  |
|-----------------------|---------------------------------------------------------------------------|
| Perception            | Detect the target, obstacles, and information useful for movement.        |
| Brain / Decision      | Manage the mission, decision rules, and state transitions.                |
| Planning / Navigation | Choose the direction, produce the outbound path, and organize the return. |
| Control               | Transform decisions into movement commands.                               |
| Memory                | Record state, events, and the path actually traveled.                     |
| Simulation            | Test behavior without immediately depending on hardware.                  |
| Hardware Adapter      | Later connect the same logic to the physical robot.                       |
| Monitoring / Logs     | Explain system behavior and facilitate diagnosis.                         |

| **Architecture constraint —** The robot brain must not directly contain code specific to Arduino, ESP32, or the simulator. |
|----------------------------------------------------------------------------------------------------------------------------|

# 6. Functional Requirements

## 6.1 Mission

**FR-01 —** The robot must be able to initialize and report whether it is ready.

**FR-02 —** It must remain stationary when no mission is active.

**FR-03 —** Activation of a light target must create a mission.

**FR-04 —** Only one mission may be executed at a time.

**FR-05 —** Every mission must have an identifier and a state.

## 6.2 Outbound Navigation

**FR-06 —** The robot must travel toward the active target.

**FR-07 —** It must detect an obstacle located on its path.

**FR-08 —** It must slow down or stop before a collision.

**FR-09 —** It must select an avoidance maneuver.

**FR-10 —** It must resume progress toward the target after avoidance.

**FR-11 —** It must record its actual movement.

## 6.3 Arrival and Collection

**FR-12 —** The robot must determine that the target has been reached according to a defined tolerance.

**FR-13 —** It must stop beside the target without hitting it.

**FR-14 —** It must simulate collection for a configurable duration.

**FR-15 —** It must remain stationary during this operation.

## 6.4 Return

**FR-16 —** The robot must return to its starting point.

**FR-17 —** The return must use the recorded outbound path in reverse order.

**FR-18 —** The robot must also avoid obstacles during the return.

**FR-19 —** It must recognize that it has reached its starting zone.

**FR-20 —** After completing the mission, it must return to the waiting state.

## 6.5 Diagnostics and Safety

**FR-21 —** Every state transition must be recorded.

**FR-22 —** Important events must be logged: target detected, obstacle, arrival, collection, return, and error.

**FR-23 —** A safety stop must be able to interrupt the motors.

**FR-24 —** In the event of an unrecoverable failure, the robot must stop and enter the ERROR state.

# 7. Non-Functional Requirements

**NFR-01 —** Modularity: each module must have a single responsibility.

**NFR-02 —** Testability: Brain, navigation, and memory must be testable without a physical robot.

**NFR-03 —** Hardware independence: the core logic must not depend directly on Arduino or ESP32.

**NFR-04 —** Local autonomy: the robot must be able to make its essential decisions locally.

**NFR-05 —** Optional central station: a global camera or central ESP32 may provide a position, distance, or grid, but the robot must never depend on that station for immediate safety.

**NFR-06 —** Configuration: distances, speeds, timings, and thresholds must not be scattered throughout the code.

**NFR-07 —** Traceability: a mission must be reconstructible from the logs.

**NFR-08 —** Simplicity: V1 will use the simplest algorithms capable of satisfying the criteria.

**NFR-09 —** Extensibility: adding multiple robots and multiple targets in the future must not require a complete rewrite.

**NFR-10 —** Reproducibility: test scenarios must be replayable.

# 8. V1 Scope

## 8.1 Included Functions

- One robot.

- One active mission.

- One relevant light target at a time.

- Detection or reception of target activation.

- Outbound navigation.

- Obstacle avoidance.

- Stop near the target.

- Timed simulated collection.

- Recording of the path actually followed.

- Return along that path.

- Execution logs.

- Simulation before hardware integration.

- Preparation for integration with the SunFounder 3in1 Ultimate Kit for Arduino UNO R4 Minima and the two available ESP32 boards.

## 8.2 Intentionally Excluded Functions

- Multiple robots on missions.

- Multiple targets active simultaneously.

- Intelligent mission assignment.

- Priorities between robots.

- Intersection management.

- Robot-to-robot communication.

- Global path optimization.

- Advanced mapping or full SLAM.

- Complex computer vision.

- Actual physical grasping of an object.

- Advanced dashboard.

- Dependency on a server or central controller.

- Machine learning required for operation.

# 9. Acceptance Criteria

**AC-01 —** The robot remains stationary when no target is active.

**AC-02 —** An active target triggers exactly one mission.

**AC-03 —** The robot reaches the target zone without a collision.

**AC-04 —** At least one obstacle placed on its route triggers an avoidance maneuver.

**AC-05 —** The robot stops within the authorized arrival zone.

**AC-06 —** It remains stopped for the entire collection duration.

**AC-07 —** It returns to the starting zone.

**AC-08 —** Its return path corresponds to the recorded outbound path in reverse order, with a safety adaptation if an obstacle is present.

**AC-09 —** Every step is visible in the logs.

**AC-10 —** An error or danger causes a safe stop.

**AC-11 —** The same scenario can be executed multiple times without manually restarting the program.

**AC-12 —** Core-logic tests can run without physical hardware.

# 10. Open Technical Decisions

| **ID** | **Decision**                       | **Options to Compare**                                                       |
|--------|------------------------------------|------------------------------------------------------------------------------|
| D-01   | Initial environment representation | 2D grid, continuous coordinates, or marked route.                            |
| D-02   | Target localization                | Onboard light sensor, ESP32 beacon, global camera, or simulated information. |
| D-03   | Return-path memory                 | Command sequence, waypoints, or grid cells.                                  |
| D-04   | Minimal simulator                  | Choice of the tool used before the physical robot.                           |

# 11. Initial Recommendation

| **Recommended baseline —** A grid-based 2D simulation, static obstacles, a target represented by coordinates, and memory composed of the grid cells actually traversed. |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

This baseline is the most instructive for building and understanding the Perception, Brain, Planning, Control, and Memory modules. Through adapters, simulated data can later be replaced gradually by data from the SunFounder kit and ESP32 boards without rewriting the core logic.

# 12. Requirements Specification Approval

This document is a reference draft. It will become Version 1.0 of the requirements specification once the four open technical decisions have been resolved and the interfaces between modules have been defined.

| **Current status**          | Draft 0.1                                             |
|-----------------------------|-------------------------------------------------------|
| **Next step**               | Resolve the four open technical decisions             |
| **Condition before coding** | Approval of the technical architecture and interfaces |
