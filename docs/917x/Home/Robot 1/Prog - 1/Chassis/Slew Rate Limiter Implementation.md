# Slew Rate Limiter Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- The `slew` class structure and its linear ramp algorithm
- How slew is initialized at motion start and iterated each control loop tick
- Integration of slew output into the drive, turn, and swing PID tasks
- Configuration of slew constants per motion type and direction

## ▲ Executive Summary

### Purpose & Need

- Without acceleration limiting, PID motions apply their full initial output the instant they begin. This causes the drivetrain to lurch, which can tip the robot or spin the wheels before they have traction.
- The `slew` class limits how quickly motor output can grow at the start of a motion by defining a linear ramp: output begins at a `min_speed` and increases proportionally as the robot travels, reaching `max_speed` after a configured `distance_to_travel`.
- Once the ramp distance is covered, the constraint lifts and the PID controller resumes full authority over motor output.

### Mathematical Foundation

Slew computes a linear speed profile using a simple slope-intercept relationship. At initialization, the ramp line is defined by two known points:

- **Start:** position = `current`, speed = `min_speed`
- **End:** position = `current + distance_to_travel`, speed = `max_speed`

The slope of this ramp is:

$$
m = \frac{v_{min} - v_{max}}{d}
$$

Where $d$ is `distance_to_travel`. Each control loop tick, the remaining distance to the ramp endpoint (`error`) is computed, and the output is evaluated as:

$$
\text{output} = (m \cdot \text{error} + v_{max}) \cdot \text{sign}
$$

This yields `min_speed` at the start and smoothly transitions to `max_speed` when `error` reaches zero. After that, the ramp is marked complete and `max_speed` is returned directly.

## ▲ Architecture

### Internal Dependencies

|  **Component**         |  **Purpose**                                                                              |
|:-----------------------|:------------------------------------------------------------------------------------------|
| `util::sgn()`          | Determines the sign (direction) of the motion to correctly orient the ramp                |
| `Drive` (PID tasks)    | Calls `iterate()` each tick and uses the output to cap PID outputs before applying them   |

### Interfaces

|  **Type**        |  **Name**                                                                                      |
|:-----------------|:-----------------------------------------------------------------------------------------------|
| Component Input  | Current sensor position (encoder or IMU), motion constants (distance, min speed, max speed)   |
| Component Output | A speed cap that grows from `min_speed` to `max_speed` over the configured ramp distance       |

## ▲ Design Spec

### The Slew Class

The `slew` class holds its state internally. The two key methods are `initialize()`, called once when a motion begins, and `iterate()`, called every tick of the PID control loop.

**Initialization** computes the geometric parameters of the ramp given the current position and target:

```cpp
void slew::initialize(bool enabled, double maximum_speed, double target, double current) {
  // Automatically disable if max_speed is below min_speed — a ramp going downward is nonsensical.
  is_enabled = maximum_speed < constants.min_speed ? false : enabled;
  max_speed = maximum_speed;

  sign = util::sgn(target - current);  // +1 forward, -1 backward

  // x_intercept: the position where the ramp ends (start + ramp distance, in the direction of travel)
  x_intercept = current + (constants.distance_to_travel * sign);

  // y_intercept: the speed at x_intercept — this is always max_speed (scaled by sign for direction)
  y_intercept = max_speed * sign;

  // Slope of the ramp line, computed from the two known points (start/min_speed and end/max_speed)
  slope = ((sign * constants.min_speed) - y_intercept) / (x_intercept - current);
}
```

**Iteration** evaluates the ramp equation each tick and detects when the ramp is complete:

```cpp
double slew::iterate(double current) {
  if (is_enabled) {
    // error = remaining distance to the ramp endpoint
    error = x_intercept - current;

    // When the sign of error flips, the robot has passed x_intercept — ramp complete
    if (util::sgn(error) != sign)
      is_enabled = false;

    // Evaluate the linear ramp: y = mx + b, scaled back by sign to get a positive speed
    else
      last_output = ((slope * error) + y_intercept) * sign;
  } else {
    // Ramp finished — return full max speed unconditionally
    last_output = max_speed;
  }
  return last_output;
}
```

### Integration into the Drive PID Task

The `Drive` class holds separate `slew` instances for each motion type. They are iterated each tick of the PID control loop, and their output is used to proportionally scale down the PID output if it exceeds the current slew ceiling.

For **drive motions**, the slew cap is applied to both sides of the drivetrain together. Taking the maximum of the two slew outputs preserves the relative difference between the sides (important for curved paths) while still enforcing the overall speed cap:

```cpp
// Iterate both slew controllers each tick
slew_left.iterate(drive_sensor_left());
slew_right.iterate(drive_sensor_right());

// Determine the highest slew-permitted speed across both sides
double max_slew_out = fmax(slew_left.output(), slew_right.output());
double faster_side = fmax(fabs(l_drive_out), fabs(r_drive_out));

// If either PID output exceeds the slew cap, scale both sides down proportionally
// Proportional scaling rather than hard clamping preserves the left/right ratio
if (faster_side > max_slew_out) {
  l_drive_out *= (max_slew_out / faster_side);
  r_drive_out *= (max_slew_out / faster_side);
}
```

This same proportional scaling is applied a second time after heading correction is blended in, ensuring the combined drive+heading output also respects the slew ceiling.

For **turn and swing motions**, the slew output is used as a symmetric clamp on the single angular PID output:

```cpp
// Turn: slew_turn.output() grows from min to max over the ramp distance
double gyro_out = util::clamp(turnPID.output, slew_turn.output(), -slew_turn.output());
```

### Configuration

Slew constants are set once in `devices.cpp` during chassis initialization. Drive supports independent forward and backward constants:

```cpp
// devices.cpp
chassis.slew_drive_constants_forward_set(3_in, 50);   // ramp over 3 in, start at speed 50
chassis.slew_drive_constants_backward_set(3_in, 50);
chassis.slew_turn_constants_set(15_deg, 40);           // ramp over 15 deg, start at speed 40
chassis.slew_swing_constants_set(5_in, 80);            // ramp over 5 in, start at speed 80
```

When a motion command is issued (e.g. `pid_drive_set()`), the relevant slew instance is re-initialized with the current position and target before the control loop begins:

```cpp
// Configured before the motion begins — sets up the ramp geometry
slew_left.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);
slew_right.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);

// Initialize with actual start/end positions so the ramp is correctly anchored
slew_left.initialize(slew_on, max_speed, l_target_encoder, drive_sensor_left());
slew_right.initialize(slew_on, max_speed, r_target_encoder, drive_sensor_right());
```

## ▲ Implementation Overview

### File Structure

```txt
include/
└── EZ-Template/
    └── slew.hpp               # slew class declaration

src/
├── EZ-Template/
│   ├── slew.cpp               # initialize() and iterate() implementation
│   └── drive/
│       ├── pid_tasks.cpp      # iterate() calls and slew-based output scaling
│       └── set_pid/
│           └── set_drive_pid.cpp  # slew.initialize() call when motion begins
└── devices.cpp                # slew constant configuration
```

### Key API

| **Function**                                   | **Parameters**                        | **Description**                                                                      |
|:-----------------------------------------------|:--------------------------------------|:-------------------------------------------------------------------------------------|
| `slew_drive_constants_forward_set(dist, speed)` | `QLength`, `int`                      | Sets ramp distance and starting speed for forward drive motions.                     |
| `slew_drive_constants_backward_set(dist, speed)`| `QLength`, `int`                      | Sets ramp distance and starting speed for backward drive motions.                    |
| `slew_turn_constants_set(dist, speed)`          | `QAngle`, `int`                       | Sets ramp distance and starting speed for in-place turns.                            |
| `slew_swing_constants_set(dist, speed)`         | `QLength`, `int`                      | Sets ramp distance and starting speed for swing turns.                               |

### Execution Flowchart

**Flowchart Instructions:**
Create a flowchart for the slew execution lifecycle:

1. **Motion command issued** (e.g. `pid_drive_set()`)
2. **`slew.initialize()`** — compute `x_intercept`, `y_intercept`, and `slope` from current position, target, and constants. Check if `max_speed < min_speed` and disable slew if so.
3. **Control loop tick begins**
4. **`slew.iterate(current)`** — compute `error = x_intercept - current`
   - Decision: "Has sign of error flipped?" (i.e. robot passed `x_intercept`)
     - **Yes** → disable slew, return `max_speed`
     - **No** → return `(slope * error + y_intercept) * sign`
5. **PID output computed** (normal PID calculation runs in parallel)
6. **Compare PID output vs. slew output**
   - If PID output exceeds slew cap → scale down proportionally
   - If PID output is within slew cap → pass through unchanged
7. **Scaled output applied to motors**
8. **Loop back to step 3** until exit condition is met

Use color coding: blue for slew path, orange for PID path, green for the merge/scaling step.
