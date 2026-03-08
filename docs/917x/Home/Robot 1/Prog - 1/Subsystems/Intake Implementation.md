# Intake Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- The `Intake` class structure and finite state machine design
- Motor behavior defined per state in the `intakeControl()` loop
- Background task execution model
- Speed configuration via `set()` overloads
- Usage in driver control and autonomous routines

## ▲ Executive Summary

### Purpose & Need

- The intake handles three distinct physical operations — collecting objects, expelling them, and scoring — each of which requires a different combination of motor directions and speeds across two hardware stages.
- Rather than implementing these as separate functions with scattered motor calls, the system is designed as a **finite state machine (FSM)**. The FSM cleanly separates *intent* (which state to be in) from *action* (what the motors do in that state), making the code predictable, easy to debug, and safe to command from both driver control and autonomous routines simultaneously.

### Why a Finite State Machine?

A naive approach would call `motor.move()` directly at every control site. This creates two problems: conflicting motor commands when multiple code paths run concurrently, and no single source of truth for what the intake is doing. The FSM solves both:

- **One control loop, one source of truth.** `intakeControl()` is the only place motors are commanded. All other code interacts with the intake only by writing to a shared `state` variable.
- **Concurrent safe.** PROS tasks run on a cooperative scheduler. Because `set()` is a single assignment and `intakeControl()` reads `state` once per loop iteration, transitions are atomic at the application level.
- **Extensible.** Adding a new behavior requires only a new `enum` value and a new `case` — no existing code paths are disturbed.

## ▲ Architecture

### External Dependencies

|  **Component**          |  **Purpose**                                                                |
|:------------------------|:----------------------------------------------------------------------------|
| `pros::Motor`           | Motor control interface for `rollerMotor` and `indexerMotor`                |
| `pros::Optical`         | Color/proximity sensor used in `waitUntilColor()` for game object detection |
| `pros::Task`            | Runs `intakeControl()` as a background task independent of the main loop    |

### Interfaces

|  **Type**        |  **Name**                                                                                      |
|:-----------------|:-----------------------------------------------------------------------------------------------|
| Component Input  | `IntakeState` enum value + optional speed parameters via `set()`                               |
| Component Output | Voltage commands to `rollerMotor` and `indexerMotor` each loop iteration                       |

## ▲ Design Spec

### Finite State Machine

The `Intake` class defines four states in the `IntakeState` enum:

```cpp
enum IntakeState { STOP, INTAKE, OUTTAKE, SCORE };
```

The control loop runs indefinitely in a background PROS task and dispatches motor outputs based on the current state:

```cpp
void Intake::intakeControl() {
  while (true) {
    switch (state) {
    case STOP:
      // Roller off. Indexer holds a small retaining voltage (-45) to
      // prevent game objects from back-driving out of the mechanism.
      indexerMotor.move(-45);
      rollerMotor.move(0);
      break;

    case INTAKE:
      // Roller pulls objects in at the configured bottom speed.
      // Indexer holds at -45 — same as STOP — to keep the chamber
      // clear and prevent stacking until a SCORE command is issued.
      indexerMotor.move(-45);
      rollerMotor.move(bottomSpeed);
      break;

    case OUTTAKE:
      // Both stages reverse to expel objects.
      indexerMotor.move(-topSpeed);
      rollerMotor.move(-bottomSpeed);
      break;

    case SCORE:
      // Both stages run forward: roller feeds upward,
      // indexer drives the object out through the top of the mechanism.
      indexerMotor.move(topSpeed);
      rollerMotor.move(bottomSpeed);
      break;
    }
    pros::delay(10);  // 100 Hz loop — sufficient latency for motor updates
  }
}
```

The key design detail is that `INTAKE` and `STOP` share the same indexer behavior (`-45`). This is intentional: while collecting objects, the indexer acts as a gate, holding objects in place until a deliberate `SCORE` command opens the path. The two-stage intake can therefore load and hold objects without immediately launching them.

### State Transitions

State transitions are entirely external — no automatic transitions occur within the FSM itself. The caller writes a new state via `set()`:

```cpp
// Single speed (applied to both top and bottom motors)
void Intake::set(IntakeState state, int speed = 127) {
  this->state = state;
  this->topSpeed = speed;
  this->bottomSpeed = speed;
}

// Independent top (indexer) and bottom (roller) speeds
void Intake::set(IntakeState state, int topSpeed, int bottomSpeed) {
  this->state = state;
  this->topSpeed = topSpeed;
  this->bottomSpeed = bottomSpeed;
}
```

The two-parameter overload is used when the two stages need different speeds — for example, during skills where a slow indexer is paired with a fast roller to avoid jamming:

```cpp
intake.set(Intake::IntakeState::SCORE, 30, 80);  // slow indexer, medium roller
```

### Background Task Model

`intakeControl()` is launched once during `initialize()` as a PROS task. It then runs concurrently with all other code for the duration of the match:

```cpp
// main.cpp — launched once at startup, runs forever
pros::Task intakeControl{[=] { intake.intakeControl(); }};
```

This means autonomous routines and `opcontrol()` never block waiting for intake motor updates — they simply write a new state and the task picks it up within the next 10 ms loop tick. The intake therefore responds near-instantly to any command from any context.

### Color Sensor Integration

`waitUntilColor()` provides a cooperative blocking mechanism for autonomous routines that need to confirm a game object has been detected before proceeding:

```cpp
void Intake::waitUntilColor(int hue1, int hue2, double saturation, int proximity, int timeout) {
  okapi::Timer timer;
  while (timer.getDtFromStart() < timeout * okapi::millisecond) {
    int hue  = colorSensor->get_hue();
    double sat = colorSensor->get_saturation();
    int prox = colorSensor->get_proximity();
    // Exit as soon as hue range, saturation threshold, and proximity are all met
    if (sat >= saturation && (hue >= hue1 && hue <= hue2) && prox >= proximity)
      return;
    pros::delay(50);
  }
  // Falls through on timeout — autonomous continues regardless
}
```

This does not interact with the FSM state directly; the intake state is set separately by the caller before or after the wait.

## ▲ Implementation Overview

### File Structure

```txt
include/
└── subsystems/
    └── intake.hpp       # Intake class declaration, IntakeState enum

src/
├── subsystems/
│   └── intake.cpp       # intakeControl(), set(), waitUntilColor()
├── devices.cpp          # Intake instance construction, motor port assignments
└── main.cpp             # Background task launch (pros::Task intakeControl)
```

### Usage Examples

**Driver control** — state transitions mapped directly to controller buttons:

```cpp
if (controller.get_digital(R2))
    intake.set(Intake::IntakeState::SCORE, 127);
else if (controller.get_digital(L2))
    intake.set(Intake::IntakeState::INTAKE, 127);
else if (controller.get_digital(L1))
    intake.set(Intake::IntakeState::OUTTAKE, 100);
else
    intake.set(Intake::IntakeState::STOP);
```

**Autonomous** — state transitions interleaved with chassis motion:

```cpp
intake.set(Intake::IntakeState::INTAKE, 127);  // begin collecting while driving
chassis.pid_odom_ptp_set({{22_in, -21_in}, fwd, 60}, true);
chassis.pid_wait_quick_chain();

intake.set(Intake::IntakeState::SCORE);        // score once at position
pros::delay(1300);

intake.set(Intake::IntakeState::INTAKE, 127);  // resume collecting
```

### State Behavior Summary

| **State**   | **Roller (`rollerMotor`)**  | **Indexer (`indexerMotor`)** | **Purpose**                             |
|:------------|:----------------------------|:-----------------------------|:----------------------------------------|
| `STOP`      | 0 (off)                     | −45 (hold)                   | Idle; retains objects in chamber        |
| `INTAKE`    | +`bottomSpeed`              | −45 (hold)                   | Pulls objects in; indexer gates chamber |
| `OUTTAKE`   | −`bottomSpeed`              | −`topSpeed`                  | Expels objects from both stages         |
| `SCORE`     | +`bottomSpeed`              | +`topSpeed`                  | Feeds objects through and out the top   |
