# Intake Mechanism PRD

|                       |            |
|:----------------------|:-----------|
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- The intake system is responsible for collecting, holding, and scoring game objects during both driver control and autonomous operation.
- It must respond instantly to driver inputs and autonomous state transitions, behave consistently and predictably across all game phases, and support precise speed control for different game scenarios (e.g. slow scoring, high-speed intake).
- Because the intake involves two mechanically distinct stages — a bottom roller and a top indexer — and must support multiple behavioral modes, it is implemented as a **finite state machine (FSM)**. This ensures that motor outputs for each stage are always correct and mutually consistent for the current operation mode, with no ambiguity or conflicting behavior.

## ▲ Success Metrics

|  **Goal**                                                                                            |  **Metric**                                                                                                    |
|:-----------------------------------------------------------------------------------------------------|:---------------------------------------------------------------------------------------------------------------|
| The intake correctly collects game objects when commanded                                            | Roller and indexer spin in the correct directions and at the correct speeds in `INTAKE` state                  |
| The intake correctly expels game objects when commanded                                              | Both motors reverse in `OUTTAKE` state                                                                         |
| The intake correctly scores game objects when commanded                                              | Indexer drives upward and roller feeds in `SCORE` state                                                        |
| The intake halts cleanly without back-driving when commanded                                         | Roller stops and indexer holds a small retaining voltage in `STOP` state                                       |
| Motor speeds are independently configurable per state call                                           | `set()` accepts separate top and bottom speed parameters that take effect immediately                          |
| The control loop runs on a dedicated background task and does not block the main program            | `intakeControl()` runs in a separate PROS task, responding to state changes without polling overhead           |

## ▲ Implementation Assumptions

- The physical intake is two-stage: a bottom roller (`rollerMotor`) feeds objects upward into a top indexer (`indexerMotor`)
- Motor port sign (positive or negative port number in the constructor) handles physical reversal — the FSM always commands positive speeds for forward motion

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**                           |  **Importance**    |  **Notes**                                                                             |
|:-------------------------------------------|:-------------------|:---------------------------------------------------------------------------------------|
| Finite state machine with four states      | HIGH               | `STOP`, `INTAKE`, `OUTTAKE`, `SCORE` — each defining distinct motor behavior           |
| Dedicated background control task          | HIGH               | `intakeControl()` runs in a PROS task so the FSM loop never blocks autonomous or opcontrol |
| Configurable speeds per `set()` call       | HIGH               | Both top and bottom speeds settable independently; defaults to 127 if not specified    |
| Optical color sensor integration           | MEDIUM             | `waitUntilColor()` allows autonomous routines to block until a specific game object is detected |

## ▲ State Diagram

**Diagram Instructions:**
Create a state transition diagram with four nodes:

- **`STOP`** — center/default state
- **`INTAKE`** — top right
- **`SCORE`** — bottom right
- **`OUTTAKE`** — bottom left

Draw directed arrows between every state pair (all transitions are valid from any state). Label each arrow "intake.set(STATE)". Inside each node, list the motor outputs:
- `STOP`: roller = 0, indexer = −45 (hold)
- `INTAKE`: roller = +speed, indexer = −45 (hold)
- `SCORE`: roller = +speed, indexer = +speed
- `OUTTAKE`: roller = −speed, indexer = −speed

Use color coding: green for INTAKE, blue for SCORE, red for OUTTAKE, grey for STOP.

## ◎ Out of Scope

- The intake FSM does not manage pneumatic pistons (loader, lift, flapper) — these are driven directly from `opcontrol()` and autonomous routines as separate hardware
- Color-sort logic is handled via `waitUntilColor()` as a cooperative blocking call from the caller; the FSM itself has no autonomous object-detection logic
