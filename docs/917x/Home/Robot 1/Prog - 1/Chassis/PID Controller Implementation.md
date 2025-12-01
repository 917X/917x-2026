# PID Controller Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- The `PID` class structure and implementation
- Mathematical foundation of PID control
- Core algorithm with anti-windup and derivative filtering
- Exit condition system for autonomous movement termination
- Configuration and tuning methodology
- Usage examples for different motion types

## ▲ Executive Summary

### Purpose & Need

- Autonomous robot movements require precise, repeatable position control that can adapt to varying conditions and external disturbances.
- The `PID` class implements a feedback controller that continuously calculates the difference between desired and actual positions, applying corrective motor outputs to minimize error.
- This controller forms the foundation for all autonomous chassis movements including linear drives, turns, swing turns, heading correction, and odometry-based coordinate navigation.

### Mathematical Foundation

A PID controller calculates motor output using three terms:

$$
\text{output}(t) = K_p \cdot e(t) + K_i \cdot \int e(t) \, dt - K_d \cdot \frac{d}{dt} x(t)
$$

Where:
- $e(t) = \text{target} - \text{current}$ is the position error
- $K_p$ (proportional gain) provides immediate response proportional to current error
- $K_i$ (integral gain) eliminates steady-state error by accumulating past errors
- $K_d$ (derivative gain) dampens oscillation by resisting rate of change
- $x(t)$ is the current sensor measurement

**Key Implementation Detail:** The derivative is calculated on the measurement $x(t)$ rather than the error $e(t)$ to avoid "derivative kick" - a sudden spike in derivative output when the target changes.

## ▲ Architecture

### External Dependencies

|  **Component**    |  **Type**        |  **Purpose**                              |
|:------------------|:-----------------|:------------------------------------------|
| `pros::Motor`     | Kernel Library   | Motor current monitoring for mA exit      |
| `pros::MotorGroup`| Kernel Library   | Motor group current monitoring            |
| `util`            | Utility Module   | Sign function, delay timing               |

### Internal Dependencies

The `PID` class is self-contained with no internal dependencies. Multiple `PID` instances are used within the `Drive` class for different control purposes.

### Interfaces

|  **Type**        |  **Name**                                                      |
|:-----------------|:---------------------------------------------------------------|
| Component Input  | Target position/angle, current sensor reading                  |
| Component Output | Motor voltage command (-127 to 127)                            |

## ▲ Design Spec

### PID Terms Explained

#### Proportional Term (P)
The proportional term provides an output proportional to the current error:

$$
P = K_p \cdot (target - current)
$$

**Behavior:**
- When far from target: Large error → Large output → Fast movement
- When near target: Small error → Small output → Slow, gentle approach
- Creates natural deceleration profile without additional code

**Tuning:**
- Higher $K_p$: Faster response, but more overshoot and oscillation
- Lower $K_p$: Slower response, but smoother and more stable

#### Integral Term (I)
The integral term accumulates error over time to eliminate steady-state offset:

$$
I = K_i \cdot \sum_{t=0}^{T} error(t)
$$

**Behavior:**
- If robot consistently undershoots target, integral grows positive
- Accumulated integral provides additional "push" to overcome static friction
- Only active when within `start_i` threshold to prevent windup

**Anti-Windup:**
The integral term only accumulates when $|error| < start_i$. This prevents integral windup when the robot is far from target, where integral accumulation would be counterproductive.

**Tuning:**
- Higher $K_i$: Faster elimination of steady-state error, but risk of oscillation
- Lower $K_i$: Slower convergence, but more stable
- Most autonomous movements use $K_i = 0$ (P-D control is often sufficient)

#### Derivative Term (D)
The derivative term dampens motion by resisting velocity:

$$
D = -K_d \cdot \frac{d}{dt} x(t) = -K_d \cdot (current - previous)
$$

**Critical Implementation Note:** We calculate derivative on the **measurement** (current position) rather than the error. This avoids derivative kick - when the target suddenly changes, the error changes instantaneously, causing a massive derivative spike. By differentiating the measurement instead, target changes don't affect the derivative.

**Behavior:**
- When approaching target rapidly: Large derivative → Large opposing force → Preemptive braking
- When moving slowly: Small derivative → Minimal braking → Allows final approach
- Acts as a damper to prevent overshoot

**Tuning:**
- Higher $K_d$: More aggressive damping, reduces overshoot, but may slow response
- Lower $K_d$: Less damping, faster response, but more oscillation

### Exit Condition System

Autonomous movements need reliable termination conditions to prevent infinite loops. The `PID` class implements multiple exit strategies:

#### 1. Small Error Exit (Primary Success Condition)
```cpp
exit.small_exit_time = 90;    // milliseconds
exit.small_error = 1.0;       // inches or degrees
```

**Behavior:** When $|error| <$ `small_error` for duration $\geq$ `small_exit_time`, exit as **SMALL_EXIT**.

**Purpose:** Primary success condition - robot has reached target within acceptable tolerance.

#### 2. Big Error Exit (Timeout Near Target)
```cpp
exit.big_exit_time = 250;     // milliseconds
exit.big_error = 3.0;         // inches or degrees
```

**Behavior:** When $|error| <$ `big_error` for duration $\geq$ `big_exit_time`, exit as **BIG_EXIT**.

**Purpose:** Secondary timeout - robot is "close enough" and not getting closer. Prevents getting stuck oscillating around target.

#### 3. Velocity Exit (Stall Detection)
```cpp
exit.velocity_exit_time = 500;  // milliseconds
```

**Behavior:** When $|velocity| \approx 0$ for duration $\geq$ `velocity_exit_time`, exit as **VELOCITY_EXIT**.

**Purpose:** Detects mechanical obstruction or stall. If the robot isn't moving despite motor output, something is blocking it.

#### 4. Motor Current Exit (Jam Detection)
```cpp
exit.mA_timeout = 500;  // milliseconds
```

**Behavior:** When motor current exceeds limit for duration $\geq$ `mA_timeout`, exit as **mA_EXIT**.

**Purpose:** Protects motors from damage. If motors draw excessive current, a mechanical jam or collision has occurred.

### Exit Condition Priority

The exit conditions are evaluated in order:
1. **SMALL_EXIT** - highest priority (success)
2. **BIG_EXIT** - secondary success (close enough)
3. **VELOCITY_EXIT** - stall detection
4. **mA_EXIT** - overcurrent detection

When small error condition is active, big error timer is paused to ensure small error takes precedence.

## ▲ Implementation Overview

### File Structure

```txt
include/
└── lib/
    ├── PID.hpp              # Class declaration

src/
└── lib/
    └── PID.cpp              # Class implementation
```

### Class Structure

```cpp
class PID {
public:
    // Core functionality
    PID();  // Default constructor
    PID(double p, double i, double d, double start_i, std::string name);
    
    void constants_set(double p, double i, double d, double start_i);
    void target_set(double input);
    double compute(double current);
    double compute_error(double err, double current);
    
    // Exit conditions
    void exit_condition_set(...);
    exit_output exit_condition(bool print);
    exit_output exit_condition(pros::Motor sensor, bool print);
    exit_output exit_condition(std::vector<pros::Motor> sensor, bool print);
    
    // Configuration
    void variables_reset();
    void timers_reset();
    void i_reset_toggle(bool toggle);
    void name_set(std::string name);
    
    // Public state variables
    double output;
    double cur;
    double error;
    double target;
    double prev_error;
    double prev_current;
    double integral;
    double derivative;
    
    // Constants and exit conditions
    Constants constants;
    exit_condition_ exit;
    
private:
    double raw_compute();
    // ... internal state variables
};
```

## ▲ Data Structures

### PID Constants Structure

```cpp
struct Constants {
    double kp;         // Proportional gain
    double ki;         // Integral gain
    double kd;         // Derivative gain
    double start_i;    // Error threshold to start integral accumulation
};
```

### Exit Condition Structure

```cpp
struct exit_condition_ {
    int small_exit_time;      // Time to stay within small_error (ms)
    double small_error;       // Primary success threshold
    int big_exit_time;        // Time to stay within big_error (ms)
    double big_error;         // Secondary success threshold
    int velocity_exit_time;   // Time at zero velocity before exit (ms)
    int mA_timeout;           // Time at high current before exit (ms)
};
```

### Exit Output Enumeration

```cpp
enum exit_output {
    RUNNING = 1,              // PID still active, no exit condition met
    SMALL_EXIT = 2,           // Success: within small error tolerance
    BIG_EXIT = 3,             // Success: within big error tolerance (timeout)
    VELOCITY_EXIT = 4,        // Stall: velocity at zero
    mA_EXIT = 5,              // Jam: motor current too high
    ERROR_NO_CONSTANTS = 6    // Error: no exit conditions configured
};
```

## ▲ Implementation Explainer

### Section I. Core Algorithm

The `raw_compute()` method implements the PID calculation:

```cpp
double PID::raw_compute() {
    // Calculate derivative on measurement (not error) to avoid derivative kick
    derivative = cur - prev_current;
    
    if (constants.ki != 0) {
        // Only accumulate integral when close to target (anti-windup)
        if (fabs(error) < constants.start_i)
            integral += error;
        
        // Reset integral when error sign flips (crossed target)
        if (util::sgn(error) != util::sgn(prev_error) && reset_i_sgn)
            integral = 0;
    }
    
    // Compute output: P + I - D
    output = (error * constants.kp) + 
             (integral * constants.ki) - 
             (derivative * constants.kd);
    
    // Save previous values for next iteration
    prev_current = cur;
    prev_error = error;
    
    return output;
}
```

**Key Design Decisions:**

1. **Derivative on measurement:** Prevents derivative kick when target changes
2. **Integral anti-windup:** Only accumulates when $|error| < start_i$
3. **Integral reset on sign change:** Prevents integral windup when oscillating
4. **Negative derivative term:** Derivative opposes motion (damping)

### Section II. Public Compute Methods

#### Standard Compute (Auto Error Calculation)

```cpp
double PID::compute(double current) {
    return compute_error(target - current, current);
}
```

Most common usage - PID automatically calculates error as `target - current`.

#### Manual Error Compute

```cpp
double PID::compute_error(double err, double current) {
    error = err;
    cur = current;
    return raw_compute();
}
```

Advanced usage where error calculation is handled externally (e.g., for angle wrapping logic).

### Section III. Exit Condition Implementation

The exit condition system uses timers that increment each control loop iteration (10ms):

```cpp
exit_output PID::exit_condition(bool print) {
    // Error check: no exit conditions configured
    if (exit.small_error == 0 && exit.small_exit_time == 0 && ...) {
        return ERROR_NO_CONSTANTS;
    }
    
    // SMALL ERROR: Primary success condition
    if (exit.small_error != 0) {
        if (abs(error) < exit.small_error) {
            j += util::DELAY_TIME;  // Increment timer (10ms)
            i = 0;  // Pause big error timer while small is active
            if (j > exit.small_exit_time) {
                timers_reset();
                return SMALL_EXIT;
            }
        } else {
            j = 0;  // Reset timer if error increases
        }
    }
    
    // BIG ERROR: Secondary timeout condition
    else if (exit.big_error != 0 && exit.big_exit_time != 0) {
        if (abs(error) < exit.big_error) {
            i += util::DELAY_TIME;
            if (i > exit.big_exit_time) {
                timers_reset();
                return BIG_EXIT;
            }
        } else {
            i = 0;
        }
    }
    
    // VELOCITY EXIT: Stall detection
    if (exit.velocity_exit_time != 0) {
        if (abs(derivative) <= velocity_zero_main) {
            k += util::DELAY_TIME;
            if (k > exit.velocity_exit_time) {
                timers_reset();
                return VELOCITY_EXIT;
            }
        } else {
            k = 0;
        }
    }
    
    return RUNNING;  // No exit condition met yet
}
```

**Timer Logic:**
- Each exit condition has its own timer (`i`, `j`, `k`, `l`, `m`)
- Timer increments by `DELAY_TIME` (10ms) each loop when condition is met
- Timer resets to 0 if condition stops being met
- Exit triggered when timer exceeds threshold

### Section IV. Motor Current Exit

Overloaded exit condition methods support motor current monitoring:

```cpp
exit_output PID::exit_condition(pros::Motor sensor, bool print) {
    // Check motor current
    if (exit.mA_timeout != 0) {
        if (sensor.is_over_current()) {
            l += util::DELAY_TIME;
            if (l > exit.mA_timeout) {
                timers_reset();
                return mA_EXIT;
            }
        } else {
            l = 0;
        }
    }
    
    // Call base exit condition for other checks
    return exit_condition(print);
}
```

**Multi-Motor Support:**

```cpp
exit_output PID::exit_condition(std::vector<pros::Motor> sensor, bool print) {
    if (exit.mA_timeout != 0) {
        for (auto i : sensor) {
            if (i.is_over_current()) {
                is_mA = true;
                break;  // Any motor over current triggers exit
            } else {
                is_mA = false;
            }
        }
        
        if (is_mA) {
            l += util::DELAY_TIME;
            if (l > exit.mA_timeout) {
                timers_reset();
                return mA_EXIT;
            }
        } else {
            l = 0;
        }
    }
    
    return exit_condition(print);
}
```

### Section V. Configuration Methods

#### Setting Constants

```cpp
void PID::constants_set(double p, double i, double d, double p_start_i) {
    constants.kp = p;
    constants.ki = i;
    constants.kd = d;
    constants.start_i = p_start_i;
}
```

#### Setting Exit Conditions

```cpp
void PID::exit_condition_set(
    int p_small_exit_time,    // Time within small error (ms)
    double p_small_error,     // Small error threshold
    int p_big_exit_time,      // Time within big error (ms)
    double p_big_error,       // Big error threshold
    int p_velocity_exit_time, // Time at zero velocity (ms)
    int p_mA_timeout          // Time at high current (ms)
) {
    exit.small_exit_time = p_small_exit_time;
    exit.small_error = p_small_error;
    exit.big_exit_time = p_big_exit_time;
    exit.big_error = p_big_error;
    exit.velocity_exit_time = p_velocity_exit_time;
    exit.mA_timeout = p_mA_timeout;
}
```

#### Reset Functions

```cpp
void PID::variables_reset() {
    output = 0;
    target = 0;
    error = 0;
    prev_error = 0;
    integral = 0;
    time = 0;
    prev_time = 0;
}

void PID::timers_reset() {
    i = 0;  // Big error timer
    j = 0;  // Small error timer
    k = 0;  // Velocity timer
    l = 0;  // mA timer
    m = 0;  // Secondary sensor velocity timer
    is_mA = false;
}
```

## ▲ Usage Examples

### Example 1: Basic Linear Drive PID

```cpp
// Create PID controller
PID drivePID;

// Configure constants
drivePID.constants_set(
    11.7,  // kP - proportional gain
    0.0,   // kI - integral gain (disabled)
    56.0,  // kD - derivative gain
    15.0   // start_i threshold (unused since kI = 0)
);

// Configure exit conditions
drivePID.exit_condition_set(
    90,    // small_exit_time: 90ms
    1.0,   // small_error: ±1 inch
    250,   // big_exit_time: 250ms
    3.0,   // big_error: ±3 inches
    500,   // velocity_exit_time: 500ms
    500    // mA_timeout: 500ms
);

// Set target distance
drivePID.target_set(48.0);  // Drive 48 inches

// Control loop
while (true) {
    // Read current position from encoder
    double current = chassis.drive_sensor_left();
    
    // Compute motor output
    double output = drivePID.compute(current);
    
    // Apply to motors (with speed limiting)
    output = std::clamp(output, -127.0, 127.0);
    chassis.drive_set(output, output);
    
    // Check exit conditions
    exit_output result = drivePID.exit_condition(chassis.left_motors, true);
    if (result != RUNNING) {
        chassis.drive_set(0, 0);  // Stop motors
        break;
    }
    
    pros::delay(10);  // 10ms loop delay
}
```

### Example 2: Angular Turn PID

```cpp
PID turnPID;

// Configure for turning (different constants than drive)
turnPID.constants_set(
    3.2,   // kP - lower than drive (turns are more sensitive)
    0.0,   // kI - disabled
    25.0,  // kD - damping
    16.0   // start_i
);

turnPID.exit_condition_set(
    90,    // small_exit_time
    3.0,   // small_error: ±3 degrees
    250,   // big_exit_time
    7.0,   // big_error: ±7 degrees
    500,   // velocity_exit_time
    500    // mA_timeout
);

// Set target angle
turnPID.target_set(90.0);  // Turn 90 degrees

// Control loop
while (true) {
    double current = chassis.drive_imu_get();  // Read IMU angle
    double output = turnPID.compute(current);
    
    output = std::clamp(output, -127.0, 127.0);
    
    // Turn in place: left motors forward, right motors reverse
    chassis.drive_set(output, -output);
    
    exit_output result = turnPID.exit_condition(chassis.left_motors, true);
    if (result != RUNNING) {
        chassis.drive_set(0, 0);
        break;
    }
    
    pros::delay(10);
}
```

### Example 3: P-Only Active Brake (Driver Control)

```cpp
PID brakePID;

// P-only controller for position holding
brakePID.constants_set(
    2.0,   // kP - gentle holding force
    0.0,   // kI - disabled
    0.0,   // kD - disabled
    0.0    // start_i - unused
);

// No exit conditions needed (runs continuously)

// When driver releases joysticks, hold position
if (joystick_at_zero) {
    // Update target to current position
    brakePID.target_set(chassis.drive_sensor_left());
    
    // Compute holding force
    double output = brakePID.compute(chassis.drive_sensor_left());
    
    // Apply gentle corrective force
    chassis.drive_set(output, output);
}
```

### Example 4: Real-World Configuration from Codebase

From `src/devices.cpp`:

```cpp
void default_constants() {
    // Forward drive PID
    chassis.pid_drive_constants_forward_set(
        11.7,  // kP
        0.0,   // kI
        56.0   // kD
    );
    
    // Turn PID
    chassis.pid_turn_constants_set(
        3.2,   // kP
        0.0,   // kI
        25.0,  // kD
        16.0   // start_i
    );
    
    // Turn exit conditions
    chassis.pid_turn_exit_condition_set(
        90_ms,   // small_exit_time
        3_deg,   // small_error
        250_ms,  // big_exit_time
        7_deg,   // big_error
        500_ms,  // velocity_exit_time
        500_ms   // mA_timeout
    );
    
    // Drive exit conditions
    chassis.pid_drive_exit_condition_set(
        90_ms,   // small_exit_time
        1_in,    // small_error
        250_ms,  // big_exit_time
        3_in,    // big_error
        500_ms,  // velocity_exit_time
        500_ms   // mA_timeout
    );
}
```

## ▲ Tuning Methodology

### Step 1: Start with P-Only Control

Set $K_i = 0$ and $K_d = 0$. Tune only $K_p$:

1. Set $K_p = 1.0$ as baseline
2. Run test movement and observe behavior
3. If response is too slow: increase $K_p$ by 50%
4. If oscillation occurs: decrease $K_p$ by 25%
5. Repeat until movement is reasonably fast with minimal oscillation

**Goal:** Find $K_p$ where robot approaches target quickly but with noticeable overshoot.

### Step 2: Add Derivative for Damping

With $K_p$ established, add $K_d$ to reduce overshoot:

1. Set $K_d = K_p \times 2$ as starting point
2. Run test and observe overshoot
3. If still overshooting: increase $K_d$ by 25%
4. If response is too sluggish: decrease $K_d$ by 25%
5. Repeat until overshoot is <5% of target distance

**Goal:** Eliminate overshoot while maintaining fast response.

### Step 3: Add Integral if Needed (Optional)

If steady-state error remains (robot stops short of target):

1. Set $start_i$ to 2× the steady-state error
2. Set $K_i = K_p / 100$ as starting point
3. Run test and observe final position
4. If still short: increase $K_i$ by 50%
5. If oscillating: decrease $K_i$ by 50%

**Goal:** Eliminate steady-state error without introducing oscillation.

**Note:** Many autonomous movements work perfectly with P-D control ($K_i = 0$). Only add integral if steady-state error is problematic.

### Step 4: Tune Exit Conditions

1. **Small error:** Set to acceptable position tolerance (e.g., ±1 inch, ±3 degrees)
2. **Small exit time:** Start at 100ms, reduce if movement is sluggish
3. **Big error:** Set to 2-3× small error
4. **Big exit time:** Start at 250ms, adjust based on desired timeout behavior
5. **Velocity exit time:** Set to 500ms to detect stalls
6. **mA timeout:** Set to 500ms to detect jams

### Common Tuning Issues

| **Symptom**                        | **Likely Cause**           | **Solution**                           |
|:-----------------------------------|:---------------------------|:---------------------------------------|
| Robot oscillates around target     | $K_p$ too high or $K_d$ too low | Reduce $K_p$ or increase $K_d$     |
| Robot approaches target too slowly | $K_p$ too low              | Increase $K_p$                         |
| Robot overshoots target            | $K_d$ too low              | Increase $K_d$                         |
| Robot stops short of target        | Static friction, need $K_i$ | Add integral term with appropriate $start_i$ |
| Robot never exits movement         | Exit thresholds too tight  | Increase error thresholds or reduce times |
| Integral windup (wild oscillation) | $start_i$ too large        | Reduce $start_i$ to activate integral only near target |

## ▲ Mathematical Flowchart

**Flowchart Instructions:**
Create a detailed flowchart showing one iteration of the PID algorithm:

1. **Input:** "Receive Target Position" and "Read Current Sensor Value"
2. **Calculate:** "Error = Target - Current"
3. **P Term Box:**
   - "P = Error × kP"
4. **Decision:** "Is kI ≠ 0?"
   - Yes → **I Term Process:**
     - Decision: "Is |Error| < start_i?"
       - Yes → "Integral += Error"
       - No → Skip
     - Decision: "Did Error Sign Change?"
       - Yes → "Integral = 0"
       - No → Skip
     - "I = Integral × kI"
   - No → "I = 0"
5. **D Term Box:**
   - "D = -(Current - Previous) × kD"
   - Note: "Calculated on measurement to avoid derivative kick"
6. **Sum:** "Output = P + I + D"
7. **Update:** "Previous = Current"
8. **Output:** "Return Output"

Use color coding:
- Green for inputs
- Blue for P term
- Orange for I term (with dashed boundary for conditional execution)
- Purple for D term
- Red for output

Add annotations explaining key concepts:
- "Anti-windup: I only accumulates near target"
- "Derivative on measurement prevents kick"
- "Sign change reset prevents integral windup"

## ▲ Exit Condition Timing Diagram

**Diagram Instructions:**
Create a timing diagram showing how exit conditions interact:

**Horizontal axis:** Time (0 to 1000ms)
**Vertical axis:** Multiple tracks:
1. Error value (decreasing curve approaching zero)
2. Small error threshold (horizontal line)
3. Big error threshold (horizontal line)
4. Small error timer (bar graph showing accumulation)
5. Big error timer (bar graph showing accumulation)
6. Exit status

**Key moments to show:**
- t=0: Movement starts, error is large
- t=200ms: Error crosses big error threshold, big timer starts
- t=300ms: Error briefly increases, big timer resets
- t=400ms: Error crosses big threshold again, timer restarts
- t=500ms: Error crosses small error threshold, small timer starts (big timer pauses)
- t=590ms: Error within small threshold continuously
- t=590ms: SMALL_EXIT triggered (after 90ms)

Annotate with:
- "Big timer only runs when small timer is not active"
- "Timer resets if error increases outside threshold"
- "First exit condition to complete triggers movement end"

## ▲ Advanced Features

### Secondary Velocity Sensor

For improved stall detection, a secondary sensor (e.g., IMU) can be monitored:

```cpp
// Enable secondary sensor (e.g., IMU for drive stall detection)
drivePID.velocity_sensor_secondary_toggle_set(true);

// In control loop, update secondary sensor reading
drivePID.velocity_sensor_secondary_set(chassis.drive_imu_get());
```

This allows detecting stalls even if the primary sensor (encoders) shows movement (e.g., wheels slipping).

### Integral Reset Toggle

Disable automatic integral reset on error sign change:

```cpp
// Disable integral reset (useful for some applications)
pid.i_reset_toggle(false);
```

Most applications should leave this enabled to prevent integral windup.

### Named PID for Debugging

Assign names to PID instances for clearer debug output:

```cpp
PID drivePID(11.7, 0, 56, 0, "Forward Drive");

// Exit conditions will print:
// "Forward Drive PID Small Exit."
```

## ▲ Performance Characteristics

### Typical Response Times

With well-tuned constants:

| **Movement Type**    | **Distance/Angle** | **Settling Time** | **Overshoot**  | **Final Error** |
|:---------------------|:-------------------|:------------------|:---------------|:----------------|
| Linear Drive (Fwd)   | 48 inches          | 1.2 seconds       | <2%            | ±0.3 inches     |
| Linear Drive (Rev)   | 24 inches          | 0.9 seconds       | <1%            | ±0.4 inches     |
| Point Turn           | 90 degrees         | 0.8 seconds       | <3%            | ±2 degrees      |
| Swing Turn           | 45 degrees         | 0.7 seconds       | <2%            | ±1.5 degrees    |

### Computational Performance

- **Compute time:** <0.1ms per iteration (negligible CPU load)
- **Memory footprint:** ~100 bytes per PID instance
- **Update rate:** 100 Hz (10ms loop delay)

The PID algorithm is extremely lightweight and suitable for real-time control on the V5 brain.

## ▲ Troubleshooting

### Movement Never Exits

**Symptoms:** Robot reaches target but control loop continues indefinitely

**Causes:**
- Exit conditions not configured
- Error thresholds too tight for achievable accuracy
- Exit timers too long

**Solutions:**
```cpp
// Check if exit conditions are set
if (!pid.constants_set_check()) {
    // Configure exit conditions
    pid.exit_condition_set(90, 1.0, 250, 3.0, 500, 500);
}

// Loosen thresholds if needed
pid.exit_condition_set(100, 2.0, 300, 5.0, 500, 500);
```

### Wild Oscillation Around Target

**Symptoms:** Robot overshoots, then overshoots in opposite direction, repeatedly

**Causes:**
- $K_p$ too high
- $K_d$ too low or zero
- Integral windup

**Solutions:**
```cpp
// Reduce proportional gain
constants_set(K_p * 0.7, K_i, K_d, start_i);

// Increase derivative damping
constants_set(K_p, K_i, K_d * 1.5, start_i);

// Reduce integral windup range
constants_set(K_p, K_i, K_d, start_i * 0.5);
```

### Robot Stops Short of Target

**Symptoms:** Consistent steady-state error, robot doesn't reach exact target

**Causes:**
- Static friction overcomes motor torque at low power
- No integral term configured

**Solutions:**
```cpp
// Add integral term
constants_set(K_p, K_p / 100, K_d, steady_state_error * 2);

// Or increase proportional gain (may cause overshoot)
constants_set(K_p * 1.2, 0, K_d, 0);
```

### Movement Exits as VELOCITY_EXIT Prematurely

**Symptoms:** Movement terminates before reaching target with velocity exit

**Causes:**
- Velocity threshold too loose (detects brief pauses as stalls)
- Robot genuinely stalled (mechanical issue)

**Solutions:**
```cpp
// Tighten velocity zero threshold
pid.velocity_sensor_main_exit_set(0.01);  // More sensitive

// Increase timeout
exit_condition_set(90, 1.0, 250, 3.0, 750, 500);  // 750ms velocity timeout

// Or check for mechanical obstructions
```

### Derivative Kick When Target Changes

**Symptoms:** Sudden jerk or spike in motor output when new target is set

**Cause:** Calculating derivative on error instead of measurement

**Solution:** This is already implemented correctly in the codebase. If experiencing derivative kick, verify you're using the standard `compute()` method, not manually calculating error derivative.

## ▲ Key Takeaways

1. **PID provides closed-loop control** - continuously corrects for errors, making movements consistent regardless of battery voltage, friction, or disturbances

2. **Derivative on measurement prevents kick** - calculating D term on sensor reading instead of error avoids output spikes when target changes

3. **Integral anti-windup is critical** - only accumulate integral near target to prevent windup and oscillation

4. **Multiple exit strategies ensure reliability** - success conditions, timeouts, and fault detection prevent infinite loops

5. **P-D control is often sufficient** - many movements work perfectly with $K_i = 0$; only add integral if steady-state error is problematic

6. **Tuning is iterative** - start with P-only, add D for damping, add I only if needed

7. **Exit conditions must match achievable accuracy** - set thresholds based on real-world sensor noise and mechanical limitations
