# Point-to-Point Navigation PRD

|                       |            |
|:----------------------|:-----------|
| **Document status**   | PROD       |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- Point-to-point (PTP) navigation enables the robot to move directly to field coordinates rather than executing sequences of linear drives and turns.
- By leveraging the odometry system's real-time position tracking, this algorithm continuously computes motor outputs to guide the robot toward a target $(x, y)$ coordinate while simultaneously controlling heading.
- This algorithm significantly simplifies autonomous programming, improves path consistency, and enables more sophisticated movement patterns impossible with basic linear/angular motions alone.

## ▲ Success metrics

|  **Goal**                                                                                          |  **Metric**                                                                                                                          |
|:---------------------------------------------------------------------------------------------------|:-------------------------------------------------------------------------------------------------------------------------------------|
| Robot navigates directly to target coordinates without pre-planned turn sequences                  | Single function call moves robot from current position to target $(x, y)$ coordinate                                                 |
| Movement paths are smooth and efficient, avoiding unnecessary stops and sharp turns                | Robot maintains continuous motion toward target with gradual heading adjustments                                                     |
| Heading control during movement is precise and configurable                                        | Robot automatically faces the movement direction, adjusting continuously as path curves                                              |
| Algorithm adapts gracefully when robot starts off-course or is pushed during movement              | Real-time position feedback allows continuous course correction without restarting movement                                          |
| Movements terminate reliably when target is reached                                                | PID exit conditions detect when robot is within acceptable tolerance of target position                                              |
| Path smoothness and curvature are configurable                                                     | Lookahead distance parameter controls how aggressively robot turns toward target                                                     |
| Integration with existing motion systems is seamless                                               | Algorithm uses the same PID controllers, exit conditions, and slew limiters as basic motions                                         |

## ▲ Implementation Assumptions

- The odometry system is functional and continuously updating the robot's position at high frequency
- The robot's starting position is set correctly before autonomous begins via `odom_xyt_set()`
- PID constants for odometry motions (`xyPID`, `odom_angularPID`) are properly tuned
- Field coordinates follow a consistent origin and axis convention throughout autonomous
- The Drive class state machine properly manages transitions between motion types

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**                           |  **Importance**    |  **Notes**                                                                                                      |
|:-------------------------------------------|:-------------------|:----------------------------------------------------------------------------------------------------------------|
| Odometry Position Tracking                 | HIGH               | Real-time $(x, y, \theta)$ coordinates are fundamental input to the algorithm                                   |
| XY Distance PID Controller                 | HIGH               | Controls linear speed toward target based on remaining distance                                                 |
| Angular PID Controller                     | HIGH               | Controls heading to face movement direction                                                                     |
| Lookahead Distance Calculation             | HIGH               | Determines point to face during movement, affecting path curvature                                              |
| Turn Bias / Heading Prioritization         | MEDIUM             | Scales linear speed based on heading error to prevent overshoot on sharp turns                                  |
| Target Detection Logic                     | HIGH               | Determines when robot has reached target coordinate to terminate movement                                       |
| Direction Control (Forward/Reverse)        | MEDIUM             | Enables robot to drive backward to targets when strategically advantageous                                      |
| Integration with Slew & Exit Conditions    | HIGH               | Reuses existing acceleration limiting and movement termination logic                                            |

## ▲ Design Overview Diagram

**Diagram Instructions:**
Create a high-level block diagram showing:

1. **Input Block** at top:
   - User specifies target coordinate: `{{x, y}, direction, speed}`

2. **Control Loop Block** (center) - runs at 100 Hz:
   - **Read Odometry:** Current $(x, y, \theta)$ from tracking system
   - **Lookahead Calculation:** Find point to face based on lookahead distance and target position
   - **Compute XY PID:** Calculate linear speed from distance to target
   - **Compute Angular PID:** Calculate turning power from heading error
   - **Turn Bias Scaling:** Scale linear speed down when heading error is large
   - **Vector Scaling:** Combine linear and angular outputs while respecting max speed
   - **Output:** Left and right motor voltages

3. **Exit Condition Check Block** (right):
   - "Target Detection" → Check if within tolerance of target
   - "Exit Condition Check" → Terminate when target reached

4. **Output Block** (bottom):
   - Motor commands to differential drive

Use arrows to show flow: Input → Control Loop (with feedback from odometry) → Exit Check → Outputs
Use a feedback arrow from "Read Odometry" back to "Control Loop" to emphasize closed-loop nature.

## ▲ Algorithm Description

### Point-to-Point Navigation

**Purpose:** Navigate directly from current position to a single target coordinate.

**How It Works:**
- Continuously calculates a lookahead point perpendicular to the approach vector (not on the path)
- The lookahead point is offset from the target by LOOK_AHEAD distance, perpendicular to the current→target line
- Robot always faces this perpendicular point, creating natural arc trajectories
- XY PID controller outputs linear speed based on remaining distance to target
- Angular PID controller outputs turning power to face the lookahead point
- Outputs are combined using vector scaling to drive left/right motors
- Movement terminates when robot reaches target within exit condition tolerances

**Key Parameters:**
- **Target coordinates:** $(x, y)$ in field coordinate system
- **Speed:** Maximum linear speed (0-127)
- **Direction:** Forward (`fwd`) or reverse (`rev`)
- **Slew enable:** Optional acceleration limiting

**Function Signature:**
```cpp
chassis.pid_odom_ptp_set({{x, y}, direction, speed});
```

**Example:**
```cpp
// Navigate to coordinate (36, 24) driving forward at speed 110
chassis.pid_odom_ptp_set({{36_in, 24_in}, fwd, 110});
chassis.pid_wait();
```

**Advantages:**
- Simple, direct path to target
- Minimal computation overhead
- Smooth curved trajectories via lookahead strategy
- Real-time course correction via odometry feedback

**Limitations:**
- Always creates arc trajectories due to perpendicular lookahead strategy, even on straight approaches
- May take inefficient curved paths if target is directly behind robot
- Single target only - no multi-waypoint support
- Can oscillate if PID tuning is too aggressive
- Not ideal for situations requiring perfectly straight motion (use `pid_drive_set()` instead)

## ▲ Key Parameters & Configuration

### Lookahead Distance
**Purpose:** Controls the perpendicular offset from target that the robot aims toward, affecting path curvature.

**Configuration:**
```cpp
chassis.odom_look_ahead_set(8_in);  // Set lookahead to 8 inches
double current = chassis.odom_look_ahead_get();  // Read current value
```

**How It Works:**
- Creates a point perpendicular to the current→target vector, offset by LOOK_AHEAD distance
- Robot continuously faces this offset point rather than the target directly
- This causes natural arc trajectories even on straight approaches
- Larger offset = wider arcs, smaller offset = tighter curves

**Effect:**
- **Small values (5-7 inches):** Tighter curves, more direct paths, may oscillate
- **Medium values (8-10 inches):** Balanced arcs, smooth motion (recommended)
- **Large values (11-15 inches):** Very wide arcs, may approach target from the side

**Visual Representation:**
```
Small lookahead (6"):          Medium lookahead (9"):        Large lookahead (12"):
Start → ╰───→ Target           Start → ╰────→ Target         Start → ╰─────→ Target
        tight curve                     smooth arc                    wide arc
```

---

### Turn Bias Amount
**Purpose:** Scales down linear speed when heading error is large, preventing overshoot on sharp turns.

**Configuration:**
```cpp
chassis.odom_turn_bias_set(2.5);  // Higher values = less speed reduction
chassis.odom_turn_bias_enable(true);
```

**How It Works:**
$$\text{linear\_speed\_scaled} = \text{linear\_speed} \times \left(1 - \frac{1 - \cos(\theta_{\text{error}})}{\text{turn\_bias}}\right)$$

When heading error is 90°:
- `turn_bias = 1.0` → linear speed reduced to ~0% (prioritize turning)
- `turn_bias = 2.5` → linear speed reduced to ~60% (balanced, recommended)
- `turn_bias = 5.0` → linear speed reduced to ~80% (prioritize forward motion)

**Tuning Guidance:**
- Increase if robot turns too aggressively and slows down excessively
- Decrease if robot overshoots target on sharp angle approaches

---

### Angular PID Constants
**Purpose:** Control responsiveness of heading correction during movement.

**Configuration:**
```cpp
// For point-to-point navigation
chassis.pid_odom_angular_constants_set(5.5, 0, 35);
```

**Tuning:**
- **P-term:** Increase for more aggressive heading correction, decrease if robot oscillates
- **D-term:** Increase to reduce oscillation/wobbling during movement
- **I-term:** Keep at 0 or very small to avoid heading drift accumulation

**Relationship to Turn PID:**
- Should be less aggressive than standard turn PID (typically 50-70% of turn P-value)
- Prevents oscillation during movement while maintaining heading accuracy

---

### XY PID Constants
**Purpose:** Control how aggressively robot accelerates toward target.

**Configuration:**
```cpp
// Uses existing forward/backward drive PID constants
chassis.pid_drive_constants_forward_set(12, 0, 80);
chassis.pid_drive_constants_backward_set(12, 0, 80);
```

**Note:** Point-to-point navigation automatically selects forward or backward constants based on movement direction.

---

### Exit Conditions
**Purpose:** Determine when movement is complete and robot has reached target.

**Configuration:**
```cpp
chassis.pid_odom_exit_condition_set(
    3_in,   // small_error: Position tolerance (3 inches)
    150,    // small_exit_time: Time within tolerance before exit (150ms)
    500,    // big_exit_time: Time for looser tolerance
    750,    // timeout: Maximum movement duration (750ms)
    0,      // velocity: Robot velocity threshold (0 = disabled)
    0       // mA: Motor current threshold (0 = disabled)
);
```

**Tuning:**
- **Tighter tolerances (1-2 inches):** More precise final position, may timeout more often
- **Looser tolerances (4-5 inches):** Faster completion, less precise
- **Balance:** Default 3 inches works well for most applications

## ▲ Coordinate System & Direction Control

### Field Coordinate Convention
- **Origin (0, 0):** Center of field
- **X-axis:** Positive toward right side of field when viewing from driver station
- **Y-axis:** Positive toward opponent side of field
- **Theta:** 0° = facing positive X direction, increases counter-clockwise

### Setting Starting Position
```cpp
void autonomous() {
    // Set robot's starting position before first movement
    chassis.odom_xyt_set(12_in, -6_in, 0_deg);  // x=12, y=-6, facing 0°
    
    // Now robot knows where it is, ready for navigation
    chassis.pid_odom_ptp_set({{36_in, 24_in}, fwd, 110});
    chassis.pid_wait();
}
```

### Forward vs Reverse Movement
**Forward (`fwd`):**
- Robot drives "front-first" toward target
- Used for most movements

**Reverse (`rev`):**
- Robot drives "back-first" toward target
- Useful when target is behind robot, avoiding full 180° turn
- Intake mechanisms often benefit from approaching targets backward

**Example:**
```cpp
// Drive forward to scoring position
chassis.pid_odom_ptp_set({{48_in, 48_in}, fwd, 110});
chassis.pid_wait();

// Drive backward to pickup (more efficient than turning around)
chassis.pid_odom_ptp_set({{12_in, 12_in}, rev, 90});
chassis.pid_wait();
```

## ▲ Integration with Existing Systems

### PID Controllers
Point-to-point navigation reuses existing Drive class PID infrastructure:
- **xyPID:** Uses `forward_drivePID` or `backward_drivePID` constants
- **odom_angularPID:** Dedicated angular controller with separate tuning
- **leftPID / rightPID:** Used internally for `pid_wait_until()` functionality

### Slew Rate Limiting
- Slew constants configured for forward/backward apply automatically
- Can be disabled per-movement: `chassis.pid_odom_ptp_set(target, false);`
- Prevents wheel slip and robot tipping during acceleration

### Exit Conditions
- Same exit condition structure as basic motions
- Based on xyPID error, velocity, and timeout
- Configured via existing functions

### Wait Functions
All standard wait functions work with point-to-point navigation:
```cpp
chassis.pid_wait();              // Block until movement complete
chassis.pid_wait_quick();        // Exit slightly early
chassis.pid_wait_quick_chain();  // Exit very early for chaining
chassis.pid_wait_until(24_in);   // Wait until 24" traveled (encoder-based)
```

## ▲ Typical Usage Patterns

### Example 1: Simple Navigation
```cpp
void autonomous() {
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    
    // Navigate to scoring zone
    chassis.pid_odom_ptp_set({{48_in, 48_in}, fwd, 110});
    chassis.pid_wait();
}
```

---

### Example 2: Sequential Movements
```cpp
void autonomous() {
    chassis.odom_xyt_set(-48_in, -24_in, 90_deg);
    
    // Navigate to first target
    chassis.pid_odom_ptp_set({{-24_in, 0_in}, fwd, 110});
    chassis.pid_wait();
    
    // Navigate to second target
    chassis.pid_odom_ptp_set({{0_in, 24_in}, fwd, 110});
    chassis.pid_wait();
    
    // Navigate to final target
    chassis.pid_odom_ptp_set({{24_in, 48_in}, fwd, 90});
    chassis.pid_wait();
}
```

---

### Example 3: Movement with Subsystem Integration
```cpp
void autonomous() {
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    
    // Navigate to pickup zone
    chassis.pid_odom_ptp_set({{24_in, 24_in}, fwd, 110});
    chassis.pid_wait();
    
    // Activate intake
    intake.set(Intake::INTAKING);
    pros::delay(500);
    
    // Navigate to scoring zone
    chassis.pid_odom_ptp_set({{48_in, 48_in}, fwd, 110});
    chassis.pid_wait();
    
    // Score
    intake.set(Intake::SCORING);
    pros::delay(500);
}
```

---

### Example 4: Reverse Movement
```cpp
void autonomous() {
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    
    // Drive forward to scoring
    chassis.pid_odom_ptp_set({{48_in, 48_in}, fwd, 110});
    chassis.pid_wait();
    
    // Drive backward to next target (faster than turning around)
    chassis.pid_odom_ptp_set({{24_in, 12_in}, rev, 90});
    chassis.pid_wait();
}
```

---

### Example 5: Mixed Basic and Point-to-Point Motions
```cpp
void autonomous() {
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    
    // Use point-to-point for long-distance movement
    chassis.pid_odom_ptp_set({{36_in, 36_in}, fwd, 110});
    chassis.pid_wait();
    
    // Use basic turn for precise final heading adjustment
    chassis.pid_turn_set(180_deg, 90);
    chassis.pid_wait();
    
    // Use basic linear drive for precise final positioning
    chassis.pid_drive_set(6_in, 70);
    chassis.pid_wait();
}
```

---

### Example 6: Configuration Tuning
```cpp
void autonomous() {
    // Configure point-to-point parameters
    chassis.odom_look_ahead_set(9_in);
    chassis.odom_turn_bias_set(2.5);
    chassis.odom_turn_bias_enable(true);
    
    // Set PID constants
    chassis.pid_odom_angular_constants_set(5.5, 0, 35);
    chassis.pid_drive_constants_forward_set(12, 0, 80);
    
    // Set exit conditions
    chassis.pid_odom_exit_condition_set(3_in, 150, 500, 750, 0, 0);
    
    // Set starting position
    chassis.odom_xyt_set(-48_in, -24_in, 90_deg);
    
    // Execute movement
    chassis.pid_odom_ptp_set({{48_in, 48_in}, fwd, 110});
    chassis.pid_wait();
}
```

## ▲ Troubleshooting

### Problem: Robot oscillates or wobbles during movement

**Likely Causes:**
- Angular PID too aggressive
- Lookahead distance too small

**Solutions:**
1. Reduce angular PID P-term
2. Increase angular PID D-term
3. Increase lookahead distance to 10-12 inches

---

### Problem: Robot takes wide, inefficient paths

**Likely Causes:**
- Lookahead distance too large
- Turn bias too high (not turning aggressively enough)

**Solutions:**
1. Decrease lookahead distance to 7-8 inches
2. Decrease turn bias amount to 2.0
3. Increase angular PID P-term slightly

---

### Problem: Robot overshoots target

**Likely Causes:**
- XY PID too aggressive
- Exit conditions too loose

**Solutions:**
1. Reduce XY PID P-term
2. Increase XY PID D-term
3. Tighten exit condition thresholds (2 inches instead of 3)
4. Reduce maximum speed

---

### Problem: Robot stops short of target

**Likely Causes:**
- Exit conditions too tight
- Minimum speed too high
- Odometry drift

**Solutions:**
1. Loosen exit condition thresholds (4-5 inches)
2. Reduce or disable minimum speed
3. Verify odometry accuracy (check tracking wheels)
4. Increase timeout value

---

### Problem: Movement never completes (timeout)

**Likely Causes:**
- Target unreachable due to obstacles
- PID constants not tuned
- Odometry not working

**Solutions:**
1. Verify target coordinates are reachable
2. Re-tune PID constants (test basic drives/turns first)
3. Verify odometry is updating (print coordinates during movement)
4. Check for mechanical issues (stuck wheels, low battery)

---

### Problem: Robot takes curved/arc path when target is directly ahead

**Diagnosis:**
- This is **expected behavior**, not a bug
- The perpendicular lookahead strategy always creates arcs
- Robot faces a point offset to the side of target, not the target itself

**Solutions:**
1. This is intentional design - arcs smooth out oscillations and create predictable motion
2. **If perfectly straight motion is required**, use `pid_drive_set()` + `pid_turn_set()` sequence instead
3. Reducing lookahead distance will make the arc tighter but won't eliminate it
4. Accept the arc trajectory as a tradeoff for smoother, more robust navigation

## ◎ Out of Scope

- **Pure Pursuit path following:** Multi-waypoint navigation with automatic waypoint advancement (uses `pid_odom_set()` function, documented separately)
- **Boomerang controller:** Specifying final heading during movement (documented separately)
- **Dynamic obstacle avoidance:** Real-time path replanning
- **Velocity-based navigation:** Using velocity profiles instead of position control
- **Turn-to-point movements:** Rotating to face a coordinate without driving
- **Adaptive lookahead distance:** Dynamic lookahead based on curvature or velocity
