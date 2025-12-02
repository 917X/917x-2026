# Autonomous Motion Algorithms Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- Implementation of three fundamental autonomous motions: linear drives, point turns, and swing turns
- Heading correction for straight-line driving
- Angle targeting behaviors and wrapping logic
- Motion chaining system for fluid autonomous routines
- Background task architecture and control loops
- Integration with slew rate limiters (detailed implementation documented separately)

## ▲ Executive Summary

### Purpose & Need

- The 15-second autonomous period requires precise, repeatable movements to score points and position the robot advantageously for driver control.
- Three fundamental motion primitives—linear drives, point turns, and swing turns—combine to execute complex autonomous routines.
- Slew rate limiting (documented separately) prevents wheel slip and tipping, while heading correction compensates for drivetrain asymmetry.

### Architecture Overview

Autonomous motions use a **background task architecture**:

1. **User code** calls motion setup function (e.g., `pid_drive_set`)
2. Setup function configures PIDs and sets drive mode
3. **Background task** runs continuously at 100Hz, executing the appropriate motion control loop
4. **Wait function** blocks user code until exit condition is met
5. Motors stop, drive mode resets to DISABLED

This separation allows user code to remain simple while complex control happens automatically in the background.

## ▲ Architecture

### External Dependencies

|  **Component**           |  **Type**        |  **Purpose**                              |
|:-------------------------|:-----------------|:------------------------------------------|
| `pros::Motor`            | Kernel Library   | Motor control and encoder feedback        |
| `pros::Imu`              | Kernel Library   | Angular position and acceleration         |
| `PID` (See PID Entry)    | Internal Class   | Feedback controllers                      |
| `slew`                   | Internal Class   | Acceleration rate limiting                |
| `okapi::QLength`         | Measurement Unit | Distance with units (in, cm, m)           |
| `okapi::QAngle`          | Measurement Unit | Angles with units (deg, rad)              |
| `okapi::QTime`           | Measurement Unit | Time with units (ms, sec)                 |

### Internal Dependencies

|  **Component**                |  **Type**    |  **Purpose**                                  |
|:------------------------------|:-------------|:----------------------------------------------|
| `drive_mode_set()`            | Function     | Sets current autonomous mode                  |
| `drive_sensor_left/right()`   | Function     | Reads motor encoder positions                 |
| `drive_imu_get()`             | Function     | Reads IMU heading                             |
| `new_turn_target_compute()`   | Function     | Angle wrapping and behavior logic             |
| `util::sgn()`                 | Utility      | Sign function                                 |
| `util::clamp()`               | Utility      | Value clamping                                |

### Interfaces

|  **Type**        |  **Name**                                                      |
|:-----------------|:---------------------------------------------------------------|
| Component Input  | Target distance/angle, max speed, configuration flags          |
| Component Output | Left/right motor voltages (-127 to 127)                        |

## ▲ Design Spec

### Drive Mode State Machine

Autonomous motions operate via a finite state machine:

|  **State**       |  **Control Task**                                                     |  **Active PIDs**                    |
|:-----------------|:----------------------------------------------------------------------|:------------------------------------|
| `DISABLE`        | No autonomous control                                                 | None                                |
| `DRIVE`          | Linear forward/backward drive                                         | Left, Right, Heading (optional)     |
| `TURN`           | Point turn in place                                                   | Turn                                |
| `SWING`          | Arc turn with one side pivoting                                       | Swing, Left or Right (hold)         |

The background task (`ez_auto_task`) continuously checks the current mode and executes the corresponding control loop.

### PID Controller Assignment

Different motion types use different PID instances:

#### Linear Drive
```cpp
leftPID           // Controls left motor group to target encoder position
rightPID          // Controls right motor group to target encoder position
headingPID        // Optional: corrects angular drift to maintain straight line
```

#### Point Turn
```cpp
turnPID           // Controls rotation to target angle (IMU feedback)
```

#### Swing Turn
```cpp
swingPID          // Controls rotation to target angle
leftPID / rightPID  // Holds stationary side in place via position lock
```

### Sensor-to-Target Calculation

#### Linear Drive
```cpp
// Target is absolute encoder position
l_target_encoder = l_start + distance;
r_target_encoder = r_start + distance;

// PID calculates error each iteration
leftPID.compute(drive_sensor_left());   // error = l_target_encoder - current
rightPID.compute(drive_sensor_right()); // error = r_target_encoder - current
```

#### Turn/Swing
```cpp
// Target is absolute IMU heading (degrees)
target_angle = new_turn_target_compute(raw_target, current_heading, behavior);

// PID calculates error each iteration
turnPID.compute(drive_imu_get());  // error = target_angle - current_heading
```

## ▲ Implementation Overview

### File Structure

```txt
src/
└── lib/
    └── drive/
        ├── set_pid/
        │   ├── set_drive_pid.cpp       # Linear drive setup
        │   ├── set_turn_pid.cpp        # Turn setup
        │   └── set_swing_pid.cpp       # Swing setup
        ├── pid_tasks.cpp               # Background control loops
        └── exit_conditions.cpp         # Wait functions
```

## ▲ Data Structures

### Motion State Variables

```cpp
// Current drive mode
e_mode mode;  // DISABLE, DRIVE, TURN, SWING

// Direction tracking
drive_directions current_drive_direction;  // FWD or REV
e_swing current_swing;                     // LEFT_SWING or RIGHT_SWING

// Angle behavior
e_angle_behavior current_angle_behavior;   // raw, shortest, left_turn, right_turn

// Motion parameters
int max_speed;               // Speed limit for current motion
bool heading_on;             // Is heading correction enabled?
int swing_opposite_speed;    // Opposite side speed for swing turns

// Start positions (for relative calculations)
double l_start;              // Left encoder at motion start
double r_start;              // Right encoder at motion start
```

### Slew Rate Limiter State

```cpp
class slew {
    double min_speed;            // Starting speed
    double distance_to_travel;   // Distance to ramp over
    double max_speed;            // Target maximum speed
    double x_intercept;          // Position where ramping completes
    double y_intercept;          // Max speed value
    double slope;                // Ramp slope (linear function)
    bool is_enabled;             // Is slew active?
};
```

## ▲ Implementation Explainer

### Section I. Linear Drive Implementation

#### Setup Function

```cpp
void Drive::pid_drive_set(double target, int speed, bool slew_on, bool toggle_heading) {
    // Reset PID timers
    leftPID.timers_reset();
    rightPID.timers_reset();
    
    // Configure motion parameters
    pid_speed_max_set(speed);
    heading_on = toggle_heading;
    
    // Record start positions
    l_start = drive_sensor_left();
    r_start = drive_sensor_right();
    
    // Calculate absolute target positions
    double l_target_encoder = l_start + target;
    double r_target_encoder = r_start + target;
    
    // Select PID constants based on direction
    PID *active_pid;
    slew::Constants slew_consts;
    
    if (target < 0) {  // Backward motion
        active_pid = &backward_drivePID;
        slew_consts = slew_backward.constants_get();
    } else {  // Forward motion
        active_pid = &forward_drivePID;
        slew_consts = slew_forward.constants_get();
    }
    
    // Apply constants to active PIDs
    PID::Constants pid_consts = active_pid->constants_get();
    leftPID.constants_set(pid_consts.kp, pid_consts.ki, pid_consts.kd, pid_consts.start_i);
    rightPID.constants_set(pid_consts.kp, pid_consts.ki, pid_consts.kd, pid_consts.start_i);
    
    // Set PID targets
    leftPID.target_set(l_target_encoder);
    rightPID.target_set(r_target_encoder);
    
    // Initialize slew rate limiters
    slew_left.initialize(slew_on, max_speed, l_target_encoder, drive_sensor_left());
    slew_right.initialize(slew_on, max_speed, r_target_encoder, drive_sensor_right());
    
    // Activate drive mode (triggers background task)
    drive_mode_set(DRIVE);
}
```

**Key Design Decisions:**

1. **Separate forward/backward constants:** Mechanical differences (weight distribution, friction) often require different tuning
2. **Absolute encoder targets:** Simplifies error calculation and prevents accumulation errors
3. **Dual slew limiters:** Left and right can ramp independently if needed

#### Control Loop (Background Task)

```cpp
void Drive::drive_pid_task() {
    // Compute PID outputs
    leftPID.compute(drive_sensor_left());
    rightPID.compute(drive_sensor_right());
    
    // Compute heading correction (if enabled)
    headingPID.compute(drive_imu_get());
    
    // Update slew rate limiters
    slew_left.iterate(drive_sensor_left());
    slew_right.iterate(drive_sensor_right());
    
    // Get raw PID outputs
    double l_drive_out = leftPID.output;
    double r_drive_out = rightPID.output;
    
    // Apply slew limiting via vector scaling
    double max_slew_out = fmax(slew_left.output(), slew_right.output());
    double faster_side = fmax(fabs(l_drive_out), fabs(r_drive_out));
    if (faster_side > max_slew_out) {
        l_drive_out *= (max_slew_out / faster_side);
        r_drive_out *= (max_slew_out / faster_side);
    }
    
    // Add heading correction
    double imu_out = heading_on ? headingPID.output : 0;
    double l_out = l_drive_out + imu_out;
    double r_out = r_drive_out - imu_out;
    
    // Scale combined output to max speed (vector scaling again)
    faster_side = fmax(fabs(l_out), fabs(r_out));
    if (faster_side > max_slew_out) {
        l_out *= (max_slew_out / faster_side);
        r_out *= (max_slew_out / faster_side);
    }
    
    // Set motors
    private_drive_set(l_out, r_out);
}
```

**Vector Scaling Explanation:**

Instead of clamping outputs (which loses proportionality), vector scaling preserves the ratio between left and right outputs:

$$
\text{scale\_factor} = \frac{\text{max\_allowed}}{\max(|L|, |R|)}
$$

$$
L_{\text{scaled}} = L \times \text{scale\_factor}, \quad R_{\text{scaled}} = R \times \text{scale\_factor}
$$

This ensures the robot still drives in the intended direction even when one side saturates.

### Section II. Point Turn Implementation

#### Setup Function

```cpp
void Drive::pid_turn_set(double target, int speed, e_angle_behavior behavior, bool slew_on) {
    turnPID.timers_reset();
    
    // Set angle behavior
    current_angle_behavior = behavior;
    
    // Compute wrapped target based on behavior
    target = new_turn_target_compute(target, drive_imu_get(), current_angle_behavior);
    
    // Set PID target
    turnPID.target_set(target);
    headingPID.target_set(target);  // Update for next drive motion
    pid_speed_max_set(speed);
    
    // Initialize slew
    slew_turn.initialize(slew_on, max_speed, target, drive_imu_get());
    
    // Activate turn mode
    drive_mode_set(TURN);
}
```

#### Angle Wrapping Logic

```cpp
double Drive::new_turn_target_compute(double target, double current, e_angle_behavior behavior) {
    double shortest_turn = util::wrap_angle(target - current);  // -180 to 180
    
    switch (behavior) {
        case shortest:
            return current + shortest_turn;
        
        case raw:
            return target;  // Use raw value (can exceed ±180)
        
        case left_turn:
            if (shortest_turn > 0)  // Already left turn
                return current + shortest_turn;
            else  // Force left by adding 360
                return current + shortest_turn + 360;
        
        case right_turn:
            if (shortest_turn < 0)  // Already right turn
                return current + shortest_turn;
            else  // Force right by subtracting 360
                return current + shortest_turn - 360;
    }
}
```

**Example:**
```
Current: 170°
Target: -170° (or 190°)

shortest:   170 + (-340 wrapped to 20) = 190°     (20° left turn)
raw:        -170°                                 (340° right turn around)
left_turn:  170 + 20 = 190°                       (20° left turn)
right_turn: 170 + (20 - 360) = -170°              (340° right turn)
```

#### Control Loop

```cpp
void Drive::turn_pid_task() {
    // Compute PID
    turnPID.compute(drive_imu_get());
    
    // Update slew
    slew_turn.iterate(drive_imu_get());
    
    // Apply slew limiting
    double gyro_out = util::clamp(turnPID.output, slew_turn.output(), -slew_turn.output());
    
    // Optional speed limiting when near target with integral active
    if (turnPID.constants.ki != 0 && 
        fabs(turnPID.target_get()) > turnPID.constants.start_i && 
        fabs(turnPID.error) < turnPID.constants.start_i) {
        if (pid_turn_min_get() != 0)
            gyro_out = util::clamp(gyro_out, pid_turn_min_get(), -pid_turn_min_get());
    }
    
    // Set motors: left forward, right backward
    private_drive_set(gyro_out, -gyro_out);
}
```

**Minimum Turn Speed:**

The optional minimum speed when integral is active prevents the robot from creeping slowly at the end of large turns. If the turn is large enough to use integral ($|target| > start_i$) and the robot enters the integral zone ($|error| < start_i$), the output is clamped to at least `turn_min` to maintain momentum through the final approach.

### Section III. Swing Turn Implementation

#### Setup Function

```cpp
void Drive::pid_swing_set(e_swing type, double target, int speed, int opposite_speed, 
                          e_angle_behavior behavior, bool slew_on) {
    swingPID.timers_reset();
    
    // Set behavior and compute target
    current_angle_behavior = behavior;
    target = new_turn_target_compute(target, drive_imu_get(), current_angle_behavior);
    
    // Determine swing direction
    current_swing = type;
    int side = type == LEFT_SWING ? 1 : -1;
    int direction = util::sgn((target - drive_imu_get()) * side);
    
    // Select constants based on direction
    PID *drive_pid, *swing_pid;
    slew::Constants slew_consts;
    
    if (direction == -1) {  // Backward swing
        drive_pid = &backward_drivePID;
        swing_pid = &backward_swingPID;
        slew_consts = slew_swing_backward.constants_get();
    } else {  // Forward swing
        drive_pid = &forward_drivePID;
        swing_pid = &forward_swingPID;
        slew_consts = slew_swing_forward.constants_get();
    }
    
    // Apply constants
    swingPID.constants_set(swing_pid->constants);
    leftPID.constants_set(drive_pid->constants);
    rightPID.constants_set(drive_pid->constants);
    
    // Lock stationary side to current position
    leftPID.target_set(drive_sensor_left());
    rightPID.target_set(drive_sensor_right());
    
    // Set swing target
    swingPID.target_set(target);
    headingPID.target_set(target);
    pid_speed_max_set(speed);
    swing_opposite_speed = opposite_speed;
    
    // Initialize slew (can use angle or distance)
    double current = slew_swing_using_angle ? drive_imu_get() : 
                     (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
    slew_swing.initialize(slew_on, max_speed, target, current);
    
    // Activate swing mode
    drive_mode_set(SWING);
}
```

**Swing Direction Logic:**

For a left swing (left side stationary):
- If turning clockwise (right): `side × direction = 1 × 1 = 1` (forward swing)
- If turning counter-clockwise (left): `side × direction = 1 × -1 = -1` (backward swing)

This determines whether to use forward or backward PID/slew constants.

#### Control Loop

```cpp
void Drive::swing_pid_task() {
    // Compute PIDs
    swingPID.compute(drive_imu_get());
    leftPID.compute(drive_sensor_left());
    rightPID.compute(drive_sensor_right());
    
    // Update slew
    double current = slew_swing_using_angle ? drive_imu_get() : 
                     (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
    slew_swing.iterate(current);
    
    // Apply slew limiting
    double swing_out = util::clamp(swingPID.output, slew_swing.output(), -slew_swing.output());
    
    // Optional minimum speed near target
    if (swingPID.constants.ki != 0 && 
        fabs(swingPID.target_get()) > swingPID.constants.start_i && 
        fabs(swingPID.error) < swingPID.constants.start_i) {
        if (pid_swing_min_get() != 0)
            swing_out = util::clamp(swing_out, pid_swing_min_get(), -pid_swing_min_get());
    }
    
    // Calculate opposite side output
    double opposite_output = 0;
    double scale = swing_out / max_speed;
    
    // Set motors based on swing type
    if (current_swing == LEFT_SWING) {
        opposite_output = swing_opposite_speed == 0 ? rightPID.output : (swing_opposite_speed * scale);
        private_drive_set(swing_out, opposite_output);
    } else {  // RIGHT_SWING
        opposite_output = swing_opposite_speed == 0 ? leftPID.output : -(swing_opposite_speed * scale);
        private_drive_set(opposite_output, -swing_out);
    }
}
```

**Opposite Speed Behavior:**

- `opposite_speed = 0`: Stationary side held by PID (tightest arc)
- `opposite_speed > 0`: Stationary side driven forward (wider arc)
- `opposite_speed < 0`: Stationary side driven backward (tighter than PID hold)

The scaling ensures opposite side speed tracks with main side speed during slew ramp.

**Note on Slew Rate Limiting:** The slew rate limiter is initialized and updated within these control loops, but its detailed implementation is documented separately. See the Slew Rate Limiter documentation for the mathematical foundation and internal algorithms.

### Section IV. Exit Conditions and Wait Functions

#### Standard Wait

```cpp
void Drive::pid_wait() {
    // Allow one iteration
    pros::delay(util::DELAY_TIME);
    
    if (mode == DRIVE) {
        exit_output left_exit = RUNNING;
        exit_output right_exit = RUNNING;
        
        while (left_exit == RUNNING || right_exit == RUNNING) {
            // Update secondary sensor (IMU accel for stall detection)
            leftPID.velocity_sensor_secondary_set(drive_imu_accel_get());
            rightPID.velocity_sensor_secondary_set(drive_imu_accel_get());
            
            // Check exit conditions
            left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
            right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
            
            pros::delay(util::DELAY_TIME);
        }
        
        // Mark as interfered if velocity or current exit
        if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || 
            right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
            interfered = true;
        }
    }
    // Similar logic for TURN, SWING modes...
}
```

**Exit Condition Checking:**

Each PID's `exit_condition()` method returns:
- `RUNNING`: Still active
- `SMALL_EXIT`: Within tight tolerance
- `BIG_EXIT`: Within loose tolerance (timeout)
- `VELOCITY_EXIT`: Stalled (no movement)
- `mA_EXIT`: Motor overcurrent (jam)

The wait function blocks until both sides exit (for drives) or the single controller exits (for turns/swings).

#### Wait Until (Early Exit)

```cpp
void Drive::wait_until_drive(double target) {
    // Calculate target as intermediate position
    double l_tar = l_start + target;
    double r_tar = r_start + target;
    
    // Calculate initial error sign
    double l_error = l_tar - drive_sensor_left();
    double r_error = r_tar - drive_sensor_right();
    int l_sgn = util::sgn(l_error);
    int r_sgn = util::sgn(r_error);
    
    while (true) {
        l_error = l_tar - drive_sensor_left();
        r_error = r_tar - drive_sensor_right();
        
        // If we haven't crossed target yet, check exit conditions as failsafe
        if (util::sgn(l_error) == l_sgn || util::sgn(r_error) == r_sgn) {
            // Check PID exit conditions
            // If PID exits before target, return early (failsafe)
        }
        // If we've crossed target, return success
        else {
            leftPID.timers_reset();
            rightPID.timers_reset();
            return;
        }
        
        pros::delay(util::DELAY_TIME);
    }
}
```

**Use Case:**

```cpp
chassis.pid_drive_set(48_in, 110);

// Wait until 24 inches (start intake midway through drive)
chassis.pid_wait_until(24_in);
intake.set(Intake::INTAKING);

// Continue waiting for full drive to complete
chassis.pid_wait();
```

#### Motion Chaining

```cpp
void Drive::pid_wait_quick_chain() {
    // Calculate distance to chain exit threshold
    double chain_target = chain_target_start - (pid_drive_chain_constant_get() * util::sgn(chain_target_start - chain_sensor_start));
    
    // Wait until that threshold is crossed
    pid_wait_until(chain_target);
    
    // Scale down PID targets for smooth transition
    used_motion_chain_scale = 0.0;  // Signals to next motion to adjust
}
```

**How Chaining Works:**

1. Motion A starts: `chassis.pid_drive_set(24_in, 110)`
2. Chain wait exits early: `chassis.pid_wait_quick_chain()` returns when within 3 inches of target
3. Motion B starts while A still completing: `chassis.pid_turn_set(90_deg, 90)`
4. Robot smoothly transitions from drive to turn with minimal dead time

## ▲ Usage Examples

### Example 1: Basic Drive and Turn Sequence

```cpp
void autonomous() {
    // Initialize
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    
    // Drive forward 24 inches
    chassis.pid_drive_set(24_in, 110, true);  // With slew
    chassis.pid_wait();
    
    // Turn 90 degrees
    chassis.pid_turn_set(90_deg, 90);
    chassis.pid_wait();
    
    // Drive backward 12 inches
    chassis.pid_drive_set(-12_in, 110, true);
    chassis.pid_wait();
}
```

### Example 2: Swing Turn with Opposite Speed

```cpp
// Swing 45 degrees with left side as pivot
// Opposite side driven at 30% of main speed for wider arc
chassis.pid_swing_set(LEFT_SWING, 45_deg, 110, 30);
chassis.pid_wait();
```

### Example 3: Heading Correction Drive

```cpp
// Drive 48 inches with heading correction enabled
// Compensates for drivetrain asymmetry
chassis.pid_drive_set(48_in, 110, true, true);  // slew_on=true, heading_on=true
chassis.pid_wait();
```

### Example 4: Relative Turns

```cpp
// Turn 45 degrees relative to current heading
chassis.pid_turn_relative_set(45_deg, 90);
chassis.pid_wait();

// Turn another 45 degrees (now 90° from original)
chassis.pid_turn_relative_set(45_deg, 90);
chassis.pid_wait();
```

### Example 5: Motion Chaining for Speed

```cpp
// Fast autonomous sequence using chaining
chassis.pid_drive_set(24_in, 110, true);
chassis.pid_wait_quick_chain();  // Exit early

chassis.pid_turn_set(90_deg, 90);
chassis.pid_wait_quick_chain();  // Exit early

chassis.pid_drive_set(18_in, 110, true);
chassis.pid_wait();  // Final motion waits fully
```

### Example 6: Wait Until for Timed Actions

```cpp
// Start driving
chassis.pid_drive_set(48_in, 110, true);

// Deploy intake at 12 inches
chassis.pid_wait_until(12_in);
intake.set(Intake::INTAKING);

// Start scoring at 36 inches
chassis.pid_wait_until(36_in);
intake.set(Intake::SCORING);

// Finish drive
chassis.pid_wait();
```

### Example 7: Angle Behavior Selection

```cpp
// Shortest path turn (default)
chassis.pid_turn_set(170_deg, 90, shortest);  // Turns 20° left from -170°

// Force right turn (goes the long way)
chassis.pid_turn_set(170_deg, 90, right_turn);  // Turns 340° right

// Raw target (useful for multi-rotation movements)
chassis.pid_turn_set(450_deg, 90, raw);  // Spins 450° (1.25 rotations)
```

### Example 8: Configuration from Real Codebase

```cpp
void default_constants() {
    // Linear drive constants
    chassis.pid_drive_constants_forward_set(11.7, 0, 56);
    chassis.pid_drive_constants_backward_set(5.7, 0.0, 9);
    
    // Turn constants
    chassis.pid_turn_constants_set(3.2, 0, 25, 16.0);
    
    // Swing constants
    chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
    
    // Heading correction
    chassis.pid_heading_constants_set(11, 0, 50.0);
    
    // Exit conditions
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
    chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    
    // Slew constants
    chassis.slew_drive_constants_forward_set(3.5_in, 40);
    chassis.slew_drive_constants_backward_set(0.5_in, 110);
    chassis.slew_turn_constants_set(3_deg, 70);
    chassis.slew_swing_constants_set(3_in, 80);
    
    // Chain thresholds
    chassis.pid_drive_chain_constant_set(3_in);
    chassis.pid_turn_chain_constant_set(3_deg);
    chassis.pid_swing_chain_constant_set(5_deg);
}
```

## ▲ Background Task Architecture Diagram

**Diagram Instructions:**
Create a sequence diagram showing interaction between user code and background task:

**Vertical Lanes:**
1. User Code (Autonomous Function)
2. Motion Setup Function
3. Background Task (ez_auto_task)
4. PID Controllers
5. Motors

**Sequence:**
1. User Code → Motion Setup: `chassis.pid_drive_set(24_in, 110)`
2. Motion Setup → PID: Configure constants, set targets
3. Motion Setup → Background Task: `drive_mode_set(DRIVE)`
4. Motion Setup → User Code: Return (non-blocking)
5. **Loop Start** (Background Task runs continuously):
   6. Background Task → PID: Read sensors, compute outputs
   7. PID → Background Task: Return motor voltages
   8. Background Task → Motors: Set voltages
   9. Background Task: Check exit conditions
   10. If not done, delay 10ms and repeat loop
11. User Code → Wait Function: `chassis.pid_wait()`
12. Wait Function: Block until exit condition met
13. **Loop End** (Exit condition triggers)
14. Background Task → Motors: Stop (voltage = 0)
15. Background Task: Set mode to DISABLE
16. Wait Function → User Code: Return (unblock)

Use different colors for each lane and arrows to show message flow.

## ▲ Control Loop Flowchart

**Flowchart Instructions:**
Create a detailed flowchart for one iteration of the drive control loop:

1. **Start:** "Background Task Iteration (10ms)"
2. **Decision:** "Current Mode?"
   - DRIVE → "Drive Control"
   - TURN → "Turn Control"
   - SWING → "Swing Control"
   - DISABLE → End
3. **Drive Control Path:**
   - "Read Left/Right Encoders"
   - "Compute Left/Right PID"
   - "Read IMU"
   - "Compute Heading PID (if enabled)"
   - "Update Slew Limiters"
   - "Apply Vector Scaling"
   - "Combine Drive + Heading"
   - "Apply Vector Scaling Again"
   - "Set Motor Voltages"
4. **Turn Control Path:**
   - "Read IMU Angle"
   - "Compute Turn PID"
   - "Update Slew Limiter"
   - "Apply Slew Limiting"
   - "Apply Minimum Speed (if near target)"
   - "Set Motors: Left=Output, Right=-Output"
5. **Swing Control Path:**
   - "Read IMU Angle"
   - "Read Pivot Side Encoder"
   - "Compute Swing PID"
   - "Compute Pivot Hold PID"
   - "Update Slew Limiter"
   - "Calculate Opposite Speed"
   - "Set Motors Based on Swing Type"
6. **End:** "Delay 10ms, Next Iteration"

Use color coding:
- Blue for sensor reading
- Green for PID computation
- Yellow for slew/scaling
- Red for motor output

## ▲ Tuning Methodology

### Step 1: Tune PID Constants

Follow the PID tuning guide (see PID Controller documentation) for each motion type:
- Forward drive PID
- Backward drive PID  
- Turn PID
- Swing PID
- Heading correction PID

### Step 2: Configure Exit Conditions

Set appropriate thresholds for each motion type:

```cpp
// Drives: tight tolerance, moderate timeouts
chassis.pid_drive_exit_condition_set(
    90_ms,   // small_exit_time
    1_in,    // small_error (±1 inch is good)
    250_ms,  // big_exit_time
    3_in,    // big_error (failsafe)
    500_ms,  // velocity_exit (stall detection)
    500_ms   // mA_timeout (jam detection)
);

// Turns: looser tolerance due to IMU noise
chassis.pid_turn_exit_condition_set(
    90_ms,   // small_exit_time
    3_deg,   // small_error (±3° accounts for IMU drift)
    250_ms,  // big_exit_time
    7_deg,   // big_error
    500_ms,  // velocity_exit
    500_ms   // mA_timeout
);
```

### Step 3: Tune Slew Constants

Start conservative and adjust based on robot behavior:

```cpp
// Forward drives: longer ramp for safety
chassis.slew_drive_constants_forward_set(
    3.5_in,  // Ramp over first 3.5 inches
    40       // Start at 40% speed
);

// Backward drives: can be more aggressive
chassis.slew_drive_constants_backward_set(
    0.5_in,  // Short ramp
    110      // Start high (backward is more stable)
);

// Turns: moderate ramp
chassis.slew_turn_constants_set(
    3_deg,   // Ramp over 3 degrees
    70       // Start at 70% speed
);
```

**Adjustment Guidelines:**
- Robot tips forward on starts → Increase ramp distance, decrease min speed
- Wheel slip during acceleration → Increase ramp distance
- Sluggish starts → Decrease ramp distance, increase min speed
- Oscillation during ramp → Check PID tuning first

### Step 4: Set Chain Constants

Configure how early to exit for motion chaining:

```cpp
// Conservative (smooth but slower transitions)
chassis.pid_drive_chain_constant_set(5_in);
chassis.pid_turn_chain_constant_set(5_deg);

// Aggressive (faster but may be jerky)
chassis.pid_drive_chain_constant_set(2_in);
chassis.pid_turn_chain_constant_set(2_deg);
```

## ▲ Troubleshooting

### Robot Doesn't Drive Straight

**Symptoms:** Curves left or right during straight drives

**Causes:**
- Motor output imbalance (one side faster)
- Drivetrain friction asymmetry
- Weight distribution off-center

**Solutions:**
```cpp
// Enable heading correction
chassis.pid_drive_set(24_in, 110, true, true);  // heading_on = true

// Tune heading PID (start with P-only)
chassis.pid_heading_constants_set(11, 0, 50);

// If still drifting, check physical drivetrain for issues
```

### Turn Overshoots Badly

**Symptoms:** Robot oscillates around target angle

**Causes:**
- Turn PID kP too high or kD too low
- Slew ramp too short (hits full speed too quickly)
- Exit conditions too loose

**Solutions:**
```cpp
// Reduce proportional gain
chassis.pid_turn_constants_set(2.5, 0, 25, 16);  // Reduced from 3.2

// Increase derivative damping
chassis.pid_turn_constants_set(3.2, 0, 35, 16);  // Increased from 25

// Lengthen slew ramp
chassis.slew_turn_constants_set(5_deg, 70);  // Increased from 3_deg
```

### Swing Turn Doesn't Hold Pivot Side

**Symptoms:** "Stationary" side drifts during swing

**Causes:**
- Pivot side PID not tuned (using wrong constants)
- Opposite speed set too high
- Mechanical slop in drivetrain

**Solutions:**
```cpp
// Ensure drive PID constants are set (used for pivot hold)
chassis.pid_drive_constants_forward_set(11.7, 0, 56);

// Use zero opposite speed for tightest hold
chassis.pid_swing_set(LEFT_SWING, 45_deg, 110, 0);  // opposite_speed = 0

// Increase PID gains for stiffer hold (test carefully)
chassis.pid_drive_constants_forward_set(15, 0, 70);
```

### Movement Never Exits

**Symptoms:** Robot reaches target but code hangs

**Causes:**
- Exit conditions not configured
- Thresholds impossible to achieve (too tight)
- PID oscillating around target

**Solutions:**
```cpp
// Verify exit conditions are set
chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

// Loosen error thresholds if needed
chassis.pid_drive_exit_condition_set(100_ms, 2_in, 300_ms, 5_in, 500_ms, 500_ms);

// Reduce PID gains to eliminate oscillation
```

### Slew Makes Movement Too Slow

**Symptoms:** Robot takes too long to reach full speed

**Causes:**
- Ramp distance too long
- Minimum speed too low

**Solutions:**
```cpp
// Shorten ramp distance
chassis.slew_drive_constants_forward_set(2_in, 40);  // Reduced from 3.5_in

// Increase minimum speed
chassis.slew_drive_constants_forward_set(3.5_in, 60);  // Increased from 40

// Disable slew if not needed
chassis.pid_drive_set(24_in, 110, false);  // slew_on = false
```

### Motion Chaining Too Jerky

**Symptoms:** Harsh transitions between movements

**Causes:**
- Chain constant too small (exits too late)
- PID constants different between movements
- Slew disabled on second movement

**Solutions:**
```cpp
// Increase chain constant for earlier exit
chassis.pid_drive_chain_constant_set(4_in);  // Increased from 3_in

// Ensure slew is enabled on chained movements
chassis.pid_turn_set(90_deg, 90, shortest, true);  // slew_on = true

// Use quick_chain only between compatible movements (drive→turn works, turn→turn may not)
```

## ▲ Key Takeaways

1. **Background task architecture** separates user code from control loops, simplifying autonomous programming while maintaining precise real-time control

2. **Separate PID instances** for different motion types allow independent tuning optimized for each movement's characteristics

3. **Vector scaling** preserves output ratios when limiting speed, ensuring robot direction remains accurate

4. **Slew rate limiting** prevents mechanical stress and improves consistency by controlling acceleration, not just max speed

5. **Heading correction** compensates for real-world imperfections in drivetrain symmetry

6. **Angle behaviors** provide strategic flexibility for different game situations (shortest path vs forced direction)

7. **Motion chaining** reduces autonomous cycle time by overlapping movements intelligently

8. **Exit conditions must match achievable accuracy** - tighter than PID can deliver causes hangs, too loose wastes time
