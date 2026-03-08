# Motion Chaining Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- The motion chaining mechanism and its relationship to PID exit conditions
- The `pid_wait_quick_chain()` function and how it extends the motion target to produce early exit
- Chain constant configuration for each motion type
- Usage patterns within autonomous routines

## ▲ Executive Summary

### Purpose & Need

- Each autonomous routine consists of many consecutive motions. If the robot must decelerate fully to zero between every motion, total cycle time grows significantly and the path appears jerky.
- Motion chaining solves this by intentionally exiting a PID motion before the robot has fully settled, preserving residual velocity that carries naturally into the next motion.
- The result is a smoother, faster multi-motion sequence with no additional complexity in the motion commands themselves - chaining is controlled entirely through which wait function is called.

### How It Works

The key insight is that the PID controller's exit conditions fire when the robot reaches close enough to the target. By **extending the target** beyond the original destination by the *chain constant*, the exit condition is triggered earlier in the deceleration curve - at a point where the robot still has meaningful velocity:

$$
\text{effective target} = \text{original target} + \text{chain constant}
$$

The robot exits while still moving, and the next motion command is issued immediately, allowing momentum to carry into the new motion naturally.

The direction of the extension is computed using the sign of the motion so the target always extends *forward* along the direction of travel:

```cpp
// util::sgn() returns +1 or -1 based on the sign of its argument.
// (chain_target_start - chain_sensor_start) is the original error:
//   positive  → robot needs to turn clockwise  → extend target further clockwise
//   negative  → robot needs to turn counter-clockwise → extend target further counter-clockwise
// Multiplying the constant by this sign ensures the extended target is always
// pushed *beyond* the goal in the same direction the robot is already going.
used_motion_chain_scale = turn_motion_chain_scale * util::sgn(chain_target_start - chain_sensor_start);

// Add the signed extension to the live PID target.
// The PID controller now believes it must reach a point slightly past the real goal,
// so its exit condition fires while the robot is still approaching — not after it stops.
turnPID.target_set(turnPID.target_get() + used_motion_chain_scale);
```

## ▲ Architecture

### Internal Dependencies

|  **Component**              |  **Purpose**                                                                     |
|:----------------------------|:---------------------------------------------------------------------------------|
| `PID` (drive/turn/swing)    | The active PID controller whose target is extended to trigger early exit         |
| `pid_wait_quick()`          | Called internally after the target extension to await the new (extended) exit    |
| `e_mode` (drive state)      | Determines which chain constant to apply based on the currently active motion    |

### Interfaces

|  **Type**        |  **Name**                                                                                         |
|:-----------------|:--------------------------------------------------------------------------------------------------|
| Component Input  | Active PID motion, per-type chain constant (or per-call override)                                 |
| Component Output | Early return from wait loop with robot still carrying velocity; no change to motion command API   |

## ▲ Design Spec

### The Chain Mechanism

When `pid_wait_quick_chain()` is called, it performs two steps:

1. **Target extension:** The active PID controller's target is increased by the configured chain constant for the current motion type (drive, turn, or swing). This makes the PID believe it needs to reach slightly further than the actual goal.
2. **Quick-wait:** `pid_wait_quick()` is called on the now-extended target. Because the exit condition fires when error is small relative to the *extended* target, the exit triggers while the robot is still in motion — before it has decelerated all the way to the true goal.

The robot is therefore still moving at the moment control returns to the autonomous routine. Issuing the next motion command immediately capitalizes on this momentum.

### Comparison: `pid_wait_quick()` vs. `pid_wait_quick_chain()`

| **Behavior**                            | `pid_wait_quick()`           | `pid_wait_quick_chain()`                   |
|:----------------------------------------|:-----------------------------|:-------------------------------------------|
| Target modified before waiting?         | No                           | Yes — extended by chain constant            |
| Robot velocity when function returns    | ~0 (fully settled)           | Non-zero (still decelerating)               |
| Transition to next motion               | Cold start from rest          | Warm transition with residual momentum     |
| Use case                                | Final motion in a sequence    | Any motion followed immediately by another |

### Chain Constant Tuning

The chain constant controls how far before the true target the exit fires. Larger values exit earlier with more residual velocity; smaller values exit later with less.

- **Too large:** The robot exits so early that it falls short of the intended position, accumulating error across sequential chains.
- **Too small:** The benefit diminishes — the robot nearly stops before the next motion begins.
- Our configuration uses **3 inches** for drive and **3 degrees** for turns, **5 degrees** for swings, which were validated empirically to provide smooth transitions without meaningful positional drift.

```cpp
// devices.cpp — chain constant configuration
chassis.pid_turn_chain_constant_set(3_deg);
chassis.pid_swing_chain_constant_set(5_deg);
chassis.pid_drive_chain_constant_set(3_in);
```

The setters store the absolute value of the input into the appropriate scale fields. `fabs()` is used to ensure the constant is always positive — direction is handled separately via sign logic at call time, so a negative value passed by the user cannot accidentally invert the extension:

```cpp
// Turn has a single constant because turns don't have a "forward" vs "backward" distinction.
void Drive::pid_turn_chain_constant_set(double input) { turn_motion_chain_scale = fabs(input); }

// Drive provides a single convenience setter that internally applies the same value
// to both the forward and backward constants, halving the configuration burden for most use cases.
void Drive::pid_drive_chain_constant_set(double input) {
  pid_drive_chain_forward_constant_set(input);
  pid_drive_chain_backward_constant_set(input);
}
// The forward and backward fields are separate so they can be tuned independently
// if reversing deceleration behavior differs from forward (e.g. due to weight distribution).
void Drive::pid_drive_chain_forward_constant_set(double input)  { drive_forward_motion_chain_scale  = fabs(input); }
void Drive::pid_drive_chain_backward_constant_set(double input) { drive_backward_motion_chain_scale = fabs(input); }
```

At runtime, `pid_wait_quick_chain()` selects the correct scale field based on which direction the current motion is traveling. `motion_chain_backward` is a flag set when the motion command was issued with a negative (reverse) target:

```cpp
// motion_chain_backward is true if the active drive motion is in reverse.
// This selects the matching chain constant rather than always using the forward one,
// allowing the two to be tuned independently without changing call-site code.
chain_scale = motion_chain_backward ? drive_backward_motion_chain_scale
                                    : drive_forward_motion_chain_scale;
```

### Per-Call Override

For situations where a single chain needs a non-standard exit point, the chain constant can be overridden inline. The override takes priority over the stored global constant:

```cpp
// The override parameter defaults to 0.0, which is treated as "not set".
// Any non-zero value bypasses the stored constant and uses the provided value instead.
// fabs() is applied here too — the sign of the override is irrelevant;
// direction is still derived from the motion's own sign, not this value.
if (motion_chain_constant_override != 0.0)
  chain_scale = fabs(motion_chain_constant_override);
```

Passing `0.0` (the default) uses the globally configured constant for the current motion type.

## ▲ Implementation Overview

### File Structure

```txt
include/
└── EZ-Template/
    └── drive/
        └── drive.hpp          # pid_wait_quick_chain() and chain constant declarations

src/
├── EZ-Template/
│   └── drive/
│       └── ...                # Internal wait and chain logic
├── devices.cpp                # Chain constant configuration (pid_*_chain_constant_set)
└── autons.cpp                 # Usage of pid_wait_quick_chain() in routines
```

### Core Implementation

The full `pid_wait_quick_chain()` function branches on the current drive `mode` to extend the correct PID controller's target, then delegates to `pid_wait_quick()`:

```cpp
void Drive::pid_wait_quick_chain(double motion_chain_constant_override) {

  // ── DRIVE mode ────────────────────────────────────────────────────────────
  if (mode == DRIVE) {
    // Resolve which chain scale to use: per-call override beats the stored constant.
    // Within the stored constant, select forward vs. backward based on motion direction.
    double chain_scale = motion_chain_constant_override != 0.0
      ? fabs(motion_chain_constant_override)
      : (motion_chain_backward ? drive_backward_motion_chain_scale
                               : drive_forward_motion_chain_scale);

    // util::sgn(chain_target_start) gives the direction of the linear motion:
    //   +1 for a forward drive, -1 for a reverse drive.
    // Multiplying by the sign ensures the extension always pushes the target
    // further in the direction the robot is already traveling.
    used_motion_chain_scale = chain_scale * util::sgn(chain_target_start);

    // Drive uses two independent PID controllers (one per side).
    // Both targets must be extended identically to keep the chassis tracking straight.
    leftPID.target_set(leftPID.target_get()   + used_motion_chain_scale);
    rightPID.target_set(rightPID.target_get() + used_motion_chain_scale);
  }

  // ── TURN mode ─────────────────────────────────────────────────────────────
  else if (mode == TURN) {
    // (chain_target_start - chain_sensor_start) is the original angular error.
    // Its sign captures the turn direction: clockwise (+) or counter-clockwise (-).
    // Applying that sign to the constant ensures the extension pushes the PID target
    // further in the same rotational direction.
    used_motion_chain_scale = (motion_chain_constant_override != 0.0
      ? fabs(motion_chain_constant_override)
      : turn_motion_chain_scale)
      * util::sgn(chain_target_start - chain_sensor_start);

    // Turns use a single angular PID controller.
    turnPID.target_set(turnPID.target_get() + used_motion_chain_scale);
  }

  // ── SWING mode ────────────────────────────────────────────────────────────
  else if (mode == SWING) {
    // Swing also distinguishes forward vs. backward, and uses the same sign
    // logic as TURN (angular direction from the original error).
    double chain_scale = motion_chain_constant_override != 0.0
      ? fabs(motion_chain_constant_override)
      : (motion_chain_backward ? swing_backward_motion_chain_scale
                               : swing_forward_motion_chain_scale);
    used_motion_chain_scale = chain_scale * util::sgn(chain_target_start - chain_sensor_start);
    swingPID.target_set(swingPID.target_get() + used_motion_chain_scale);
  }
  // ...odometry modes handled separately

  // After the target has been extended, hand off to pid_wait_quick().
  // This blocks until the robot passes the *original* goal position —
  // which it reaches while still moving, because the PID target is now beyond it.
  pid_wait_quick();
}
```

`pid_wait_quick()` is what actually determines when the function returns. It calls `pid_wait_until` with `chain_target_start` — the **original** target that was saved before any extension happened. The wait loop therefore unblocks as soon as the robot crosses the real goal, at which point the PID is still driving toward the *extended* target and the robot has not yet decelerated to zero:

```cpp
void Drive::pid_wait_quick() {
  // chain_target_start is captured when the motion command is first issued
  // and is never modified by pid_wait_quick_chain().
  // Waiting on it means: "exit as soon as the robot reaches the original goal",
  // regardless of how far the PID target has been pushed beyond it.
  pid_wait_until(chain_target_start);
}
```

### Key API

| **Function**                            | **Parameters**                   | **Description**                                                               |
|:----------------------------------------|:---------------------------------|:------------------------------------------------------------------------------|
| `pid_wait_quick_chain(override)`        | `double override = 0.0` (inches/deg) | Extends the current target by the chain constant and exits with `pid_wait_quick()`. Pass non-zero to override the global constant for this call only. |
| `pid_drive_chain_constant_set(input)`   | `QLength` or `double` (inches)   | Sets the chain constant for both forward and backward drive motions.           |
| `pid_turn_chain_constant_set(input)`    | `QAngle` or `double` (degrees)   | Sets the chain constant for turn motions.                                      |
| `pid_swing_chain_constant_set(input)`   | `QAngle` or `double` (degrees)   | Sets the chain constant for both forward and backward swing motions.           |

### Usage in Autonomous Routines

Motion chaining is used extensively throughout our autonomous routines. A representative sequence:

```cpp
// Swing into position, chain into next drive — no stop between them
chassis.pid_swing_set(LEFT_SWING, 50_deg, 100, 5, false);
chassis.pid_wait_quick_chain();
chassis.pid_odom_ptp_set({{-19_in, -22_in}, fwd, 40}, false);
chassis.pid_wait_quick_chain();
chassis.pid_odom_ptp_set({{-18.5_in, 22.3_in}, fwd, 60}, false);
chassis.pid_wait_quick();  // Final motion in the sequence — full settle
```

Here, `pid_wait_quick_chain()` is called between every motion except the last, where `pid_wait_quick()` is used to ensure the robot has fully settled before a precision action (e.g., scoring).

### Execution Flowchart

**Flowchart Instructions:**
Create a flowchart for the `pid_wait_quick_chain()` execution path:

1. **Start:** "`pid_wait_quick_chain()` called"
2. **Query:** "Get chain constant for current motion type (drive / turn / swing)" — or use override if non-zero
3. **Modify:** "Add chain constant to current PID target → new extended target"
4. **Wait:** "Call `pid_wait_quick()` — loop until exit condition met on extended target"
5. **Exit:** "Function returns — robot is still in motion"
6. **Next:** "Autonomous code issues next motion command immediately"

Use a dashed border around steps 3–5 to group them as the internal mechanism. Use a different color for "Add chain constant to target" to highlight it as the key differentiator from a normal `pid_wait_quick()` call.
