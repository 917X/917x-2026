# Point-to-Point Navigation Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | PROD       |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- Mathematical foundations for point-to-point navigation
- Lookahead point calculation algorithm
- Target detection and "past target" logic
- Control loop architecture and motor output calculations
- Complete implementation walkthrough with code
- Configuration parameters and tuning guidance

## ▲ Executive Summary

### Purpose & Need

Traditional autonomous programming requires decomposing every movement into sequences of linear drives and turns. For example, to move from point A to point B:
1. Calculate angle to turn
2. Execute turn
3. Calculate distance to drive
4. Execute drive
5. Handle any drift or position errors

This approach is:
- **Time-consuming:** Each movement requires manual angle/distance calculations
- **Error-prone:** Small measurement errors compound across movements
- **Inflexible:** Cannot adapt mid-execution to disturbances

Point-to-point (PTP) navigation solves this by accepting target coordinates and automatically computing optimal motor outputs. The robot continuously recalculates its path based on real-time odometry, enabling:
- **Simplified programming:** One function call replaces multi-step sequences
- **Robustness:** Automatic correction for wheel slip, collisions, or position drift  
- **Smooth paths:** Gradual curved trajectories instead of stop-turn-drive sequences

### Design Philosophy

1. **Closed-Loop Control:** Continuous odometry feedback ensures target is reached regardless of disturbances
2. **Lookahead Strategy:** Robot faces a point ahead on the path rather than directly at target, enabling smooth curved trajectories
3. **PID-Based Architecture:** Reuses existing, well-tuned PID controllers for consistency with basic motions
4. **State Machine Integration:** Clean transitions between motion types via Drive class state enumeration

## ▲ Architecture

### External Dependencies

|  **Component**        |  **Type**               |  **Purpose**                                    |
|:----------------------|:------------------------|:------------------------------------------------|
| `Pros::Motor`         | Kernel Library          | Motor control interface                         |
| `Pros::Imu`           | Kernel Library          | Inertial sensor for heading                     |
| `Pros::Rotation`      | Kernel Library          | Tracking wheel encoders                         |
| `okapi::QAngle`       | Measurement Unit Helper | Angles with units (deg, rad)                    |
| `okapi::QLength`      | Measurement Unit Helper | Distances with units (in, m, cm)                |

### Internal Dependencies

|  **Component**                          |  **Type**    |  **Purpose**                                                 |
|:----------------------------------------|:-------------|:-------------------------------------------------------------|
| `PID` (Drive, Turn, Odom controllers)   | Class        | PID control for linear and angular motions                   |
| `slew`                                  | Class        | Acceleration limiting                                        |
| `tracking_wheel`                        | Class        | Odometry encoder abstraction                                 |
| `util`                                  | Namespace    | Math utilities (angle wrapping, distance, etc.)              |
| `pose`                                  | Struct       | Position structure `{x, y, theta}`                           |
| `odom`                                  | Struct       | Movement command `{pose, direction, speed, turn_behavior}`   |
| `e_mode`                                | Enum         | Drive state (POINT_TO_POINT, etc.)                           |
| `drive_directions`                      | Enum         | Forward (`fwd`) or reverse (`rev`)                           |

### Data Structures

#### Core Position Structures

```cpp
// Pose: Represents a 2D position with heading
struct pose {
    double x;      // X coordinate (inches)
    double y;      // Y coordinate (inches)  
    double theta;  // Heading angle (degrees), or ANGLE_NOT_SET
};

// Odom: Complete movement specification
struct odom {
    pose target;                           // Target position
    drive_directions drive_direction;      // fwd or rev
    int max_xy_speed;                      // Max linear speed (0-127)
    int min_speed = 0;                     // Min speed (default 0)
    e_angle_behavior turn_behavior = raw;  // Angle targeting mode
};
```

#### Instance Variables for Point-to-Point Motion

```cpp
// Current target tracking
pose odom_target = {0.0, 0.0, 0.0};        // Current target position
pose odom_current = {0.0, 0.0, 0.0};       // Current robot position
pose odom_start = {0.0, 0.0, 0.0};         // Position at movement start
pose odom_target_start = {0.0, 0.0, 0.0};  // Final target for movement

// Lookahead point calculation
pose point_to_face[2];                     // Two candidate points perpendicular to target
bool ptf1_running;                         // Which candidate point is active

// Movement state
drive_directions current_drive_direction;  // fwd or rev
int past_target;                           // Sign of initial "past target" value
double new_current_fake = 0.0;             // Fake sensor value for XY PID
double xy_delta_fake = 1.0;                // Increment for fake sensor

// Configuration parameters
double LOOK_AHEAD = 7.0;                   // Lookahead distance (inches)
double odom_turn_bias_amount = 2.5;        // Turn bias scaling factor
bool is_odom_turn_bias_enabled = true;     // Enable/disable turn bias
int odom_min_speed = 0;                    // Minimum speed during movement
double odom_drive_boost_threshold = 0.0;   // Drive boost activation threshold
double odom_drive_boost_multiplier = 1.0;  // Drive boost multiplier
```

### Interfaces

|  **Type**        |  **Name**                                                                              |
|:-----------------|:---------------------------------------------------------------------------------------|
| Component Input  | Odometry position, target coordinates, PID constants, user configuration parameters    |
| Component Output | Left/right motor voltages, drive mode state, current target information for debugging  |

## ▲ Mathematical Foundations

### Lookahead Point Calculation

The lookahead point determines where the robot should face during movement. **Critically, this point is perpendicular to the current→target vector, not ahead on the path.** This creates a "side-facing carrot" strategy that produces natural arc trajectories.

**Given:**
- Current position: $(x_c, y_c)$
- Target position: $(x_t, y_t)$
- Lookahead distance: $d_l$

**Calculation:**

1. Find slope of line from current to target:
   $$m = \frac{y_t - y_c}{x_t - x_c}$$

2. Find angle of that line, then **rotate 90° to get perpendicular**:
   $$\alpha = 90° - \arctan(m)$$

3. Two candidate points exist on the perpendicular line through target, at distance $d_l$:
   $$\text{ptf}_1 = (x_t + d_l \cos(\alpha), \quad y_t + d_l \sin(\alpha))$$
   $$\text{ptf}_2 = (x_t - d_l \cos(\alpha), \quad y_t - d_l \sin(\alpha))$$

4. Select the candidate farther from current position:
   $$\text{point\_to\_face} = \begin{cases}
   \text{ptf}_1 & \text{if } \|\text{ptf}_1 - (x_c, y_c)\| > \|\text{ptf}_2 - (x_c, y_c)\| \\
   \text{ptf}_2 & \text{otherwise}
   \end{cases}$$

**Key Insight:** This perpendicular strategy means the robot is **always** trying to face a point offset to the side of the target, regardless of heading error. This creates arc trajectories even when the robot starts perfectly aligned with the target.

**Visual Representation:**
```
                      ptf_1 (farther from current)
                         *
                         |
                         | d_l (lookahead distance)
                         |
Current ────────────→ Target ──── perpendicular line
  (x_c, y_c)          (x_t, y_t)
                         |
                         | d_l
                         |
                         *
                      ptf_2 (closer, not selected)

Robot faces ptf_1, creating an arc trajectory even if 
already pointing at target.
```

**Important Behavior Note:**

This perpendicular lookahead strategy is applied **continuously throughout the movement**, regardless of current heading error. This means:

- Even if the robot starts perfectly aligned with the target (0° heading error)
- It will still face the perpendicular offset point
- This creates a natural arc trajectory rather than a straight line
- The arc becomes more pronounced with larger lookahead distances

This is **different from traditional Pure Pursuit**, where the lookahead point would be ahead on the desired path. Here, the perpendicular offset is a design choice that:
- Smooths out oscillations near the target
- Creates predictable arc trajectories
- May be less efficient for straight-line approaches

**When to use basic `pid_drive_set()` instead:**
- If you need perfectly straight motion
- If the target is directly ahead and you want no lateral drift
- If efficiency is critical and the approach angle is already correct

**Special Case: Reverse Movement**
When driving backward (`rev`), the target is conceptually rotated 180° around the current position before lookahead calculation. This makes the robot face away from the target while driving backward.

**Implementation:**
```cpp
std::vector<pose> Drive::find_point_to_face(pose current, pose target, 
                                             drive_directions dir, bool set_global) {
    // Flip target 180° if reversing
    if (dir == rev) {
        pose new_target = target;
        // Translate to current as origin, flip, translate back
        new_target.x = current.x - (target.x - current.x);
        new_target.y = current.y - (target.y - current.y);
        target = new_target;
    }

    // Calculate slope and angle
    double tx_cx = target.x - current.x;
    double m = 0.0;
    double angle = 0.0;
    if (tx_cx != 0) {
        m = (target.y - current.y) / tx_cx;
        angle = 90.0 - util::to_deg(atan(m));
    }

    // Two candidate points at ± lookahead distance
    pose ptf1 = util::vector_off_point(odom_look_ahead_get(), 
                                       {target.x, target.y, angle});
    pose ptf2 = util::vector_off_point(-odom_look_ahead_get(), 
                                       {target.x, target.y, angle});

    // Select farther point
    if (set_global) {
        double ptf1_dist = util::distance_to_point(ptf1, current);
        double ptf2_dist = util::distance_to_point(ptf2, current);
        ptf1_running = (ptf1_dist > ptf2_dist);
    }

    point_to_face = {ptf1, ptf2};
    return {ptf1, ptf2};
}
```

---

### Target Detection Logic

The robot must determine when it has reached the target to terminate movement.

**Naive Approach (Doesn't Work):**
Simply checking if distance to target is below threshold fails because:
- Robot may never pass exactly through target
- Creates oscillation as robot circles around target
- Doesn't account for movement direction

**Correct Approach: "Past Target" Detection**

Transform the problem into 1D by projecting position onto the axis from current position through lookahead point:

1. Translate target to origin: $(x_t', y_t') = (0, 0)$
2. Translate current position by same amount: $(x_c', y_c')$
3. Calculate angle to lookahead point (adjusted for forward/reverse)
4. Rotate coordinate system so lookahead point lies on Y-axis
5. Check sign of rotated $y_c'$: 
   - Negative → haven't reached target yet
   - Positive → have passed target

**Visual Representation:**
```
Before rotation:              After rotation to lookahead axis:
                              
   Lookahead                     Lookahead
      *                              * (on positive Y-axis)
      |                              |
      |                              |
  Current *----→ Target          Current * ← (positive Y = past target)
                                     |
                                   Target (origin)
                                     |
```

**Implementation:**
```cpp
double Drive::is_past_target(pose target, pose current) {
    // Translate around target as origin
    double fake_y = current.y - target.y;
    double fake_x = current.x - target.x;

    // Get angle to lookahead point
    pose ptf;
    ptf.y = point_to_face[!ptf1_running].y - target.y;
    ptf.x = point_to_face[!ptf1_running].x - target.x;
    int add = current_drive_direction == REV ? 180 : 0;
    double fake_angle = util::to_rad(
        util::absolute_angle_to_point(ptf, {fake_x, fake_y}) + add
    );

    // Rotate coordinate system
    double rotated_x = (fake_x * cos(fake_angle)) - (fake_y * sin(fake_angle));
    double rotated_y = (fake_y * cos(fake_angle)) + (fake_x * sin(fake_angle));

    return rotated_y;  // Positive = past target, negative = haven't reached
}
```

**Usage:**
```cpp
// At movement start, record initial sign
past_target = util::sgn(is_past_target(odom_target, odom_pose_get()));

// During movement, check if sign flipped
double current_value = is_past_target(odom_target, odom_pose_get());
int flipped = (util::sgn(current_value) != util::sgn(past_target)) ? -1 : 1;
// flipped = -1 means we've passed target
```

---

### Turn Bias Calculation

When heading error is large, the robot should prioritize turning over forward motion to avoid overshooting the target. Turn bias scales down linear speed based on heading error.

**Formula:**
$$\text{scale} = 1 - \frac{1 - \cos(\theta_{\text{error}})}{\text{bias}_{\text{amount}}}$$

where:
- $\theta_{\text{error}}$ = angle difference between current heading and lookahead point
- $\text{bias}_{\text{amount}}$ = tunable constant (typical: 2.0 - 3.0)

**Behavior Examples:**
| Heading Error | bias = 1.5 | bias = 2.5 | bias = 4.0 |
|--------------|------------|------------|------------|
| 0°           | 100%       | 100%       | 100%       |
| 45°          | 61%        | 77%        | 85%        |
| 90°          | 33%        | 60%        | 75%        |
| 135°         | 11%        | 43%        | 65%        |
| 180°         | 0%         | 20%        | 50%        |

**Implementation:**
```cpp
// In ptp_task() control loop
double xy_out = xyPID.output;  // Linear speed
double a_out = current_a_odomPID.output;  // Angular speed

// Apply turn bias scaling
if (odom_turn_bias_enabled()) {
    double scale = 1.0 - ((1.0 - cos(util::to_rad(current_a_odomPID.error))) 
                           / odom_turn_bias_amount);
    xy_out *= scale;
}
```

**Tuning Guidance:**
- **Low bias (1.5 - 2.0):** Aggressive turning, prioritizes heading accuracy, may slow down excessively
- **Medium bias (2.5 - 3.0):** Balanced, smooth paths (recommended)
- **High bias (4.0 - 5.0):** Prioritizes forward motion, may overshoot on sharp turns

## ▲ Implementation Explainer

### Section I: User Function Call

The user initiates a point-to-point movement:

```cpp
// User code
chassis.pid_odom_ptp_set({{36_in, 24_in}, fwd, 110});
```

This function has multiple overloads for convenience:

```cpp
// Base signature (no units, explicit slew)
void Drive::pid_odom_ptp_set(odom imovement, bool slew_on);

// Without slew parameter (auto-detect from direction)
void Drive::pid_odom_ptp_set(odom imovement) {
    bool slew_on = imovement.drive_direction == fwd ? 
                   slew_drive_forward_get() : slew_drive_backward_get();
    pid_odom_ptp_set(imovement, slew_on);
}

// With okapi units
void Drive::pid_odom_ptp_set(united_odom p_imovement) {
    odom imovement = util::united_odom_to_odom(p_imovement);
    pid_odom_ptp_set(imovement);
}

// With okapi units and explicit slew
void Drive::pid_odom_ptp_set(united_odom p_imovement, bool slew_on) {
    odom imovement = util::united_odom_to_odom(p_imovement);
    pid_odom_ptp_set(imovement, slew_on);
}
```

### Section II: Movement Initialization

The main initialization function sets up all necessary state:

```cpp
void Drive::pid_odom_ptp_set(odom imovement, bool slew_on) {
    imovement = set_odom_direction(imovement);  // Apply coordinate flipping if enabled

    // Store movement metadata for exit condition checking
    odom_second_to_last = odom_pose_get();
    odom_target_start = imovement.target;
    odom_start = odom_pose_get();

    // Reset PID timers
    xyPID.timers_reset();
    current_a_odomPID.timers_reset();

    // Store encoder values for wait_until() functionality
    l_start = drive_sensor_left();
    r_start = drive_sensor_right();

    // Enable turn bias and configure slew
    odom_turn_bias_enable(true);
    current_slew_on = slew_on;
    slew_min_when_it_enabled = 0;
    slew_will_enable_later = false;

    // Call raw initialization (detailed below)
    raw_pid_odom_ptp_set(imovement, slew_on);

    // Initialize slew controllers
    int dir = current_drive_direction == REV ? -1 : 1;
    double dist_to_target = util::distance_to_point(odom_target, odom_pose_get()) * dir;
    slew_left.initialize(slew_on, max_speed, dist_to_target + l_start, l_start);
    slew_right.initialize(slew_on, max_speed, dist_to_target + r_start, r_start);

    // Set drive mode to activate control loop
    drive_mode_set(POINT_TO_POINT);
}
```

### Section III: Target Setup

The `raw_pid_odom_ptp_set()` function configures the target and PID constants:

```cpp
void Drive::raw_pid_odom_ptp_set(odom imovement, bool slew_on) {
    // ===== STEP 1: Update Movement Direction =====
    current_drive_direction = imovement.drive_direction;

    // ===== STEP 2: Calculate Lookahead Point =====
    point_to_face = find_point_to_face(
        odom_pose_get(), 
        {imovement.target.x, imovement.target.y}, 
        current_drive_direction, 
        true
    );

    // Calculate angle to face lookahead point
    double target = util::absolute_angle_to_point(
        point_to_face[!ptf1_running], 
        odom_pose_get()
    );

    // Apply turn behavior (shortest, raw, left, right)
    if (imovement.turn_behavior != raw) {
        odom_imu_start = drive_imu_get();
        current_angle_behavior = imovement.turn_behavior;
    }
    target = new_turn_target_compute(target, odom_imu_start, current_angle_behavior);
    headingPID.target_set(target);

    // ===== STEP 3: Set Target Coordinates =====
    odom_target.x = imovement.target.x;
    odom_target.y = imovement.target.y;

    // ===== STEP 4: Select Forward or Backward PID Constants =====
    PID *new_drive_pid;
    slew::Constants slew_consts;
    
    if (current_drive_direction == REV) {
        new_drive_pid = &backward_drivePID;
        slew_consts = slew_backward.constants_get();
    } else {
        new_drive_pid = &forward_drivePID;
        slew_consts = slew_forward.constants_get();
    }

    // Prioritize custom fwd/rev constants if available
    if (fwd_rev_drivePID.constants_set_check() && !new_drive_pid->constants_set_check())
        new_drive_pid = &fwd_rev_drivePID;

    // ===== STEP 5: Copy Constants to XY PID =====
    PID::Constants pid_drive_consts = new_drive_pid->constants_get();
    xyPID.constants_set(pid_drive_consts.kp, pid_drive_consts.ki, 
                        pid_drive_consts.kd, pid_drive_consts.start_i);

    // ===== STEP 6: Set Speed Limits =====
    pid_speed_max_set(imovement.max_xy_speed);
    odom_min_speed = imovement.min_speed;

    // ===== STEP 7: Configure Slew if Needed =====
    int slew_min = slew_consts.min_speed;
    if (current_slew_on && slew_will_enable_later && !slew_on && slew_odom_reenabled()) {
        slew_on = true;
        slew_will_enable_later = false;
        if (slew_min_when_it_enabled > slew_consts.min_speed)
            slew_min = slew_min_when_it_enabled;

        slew_left.constants_set(slew_consts.distance_to_travel, slew_min);
        slew_right.constants_set(slew_consts.distance_to_travel, slew_min);

        // Reinitialize slew
        int dir = current_drive_direction == REV ? -1 : 1;
        double dist_to_target = 100.0 * dir;
        slew_left.initialize(slew_on, max_speed, dist_to_target + drive_sensor_left(), 
                            drive_sensor_left());
        slew_right.initialize(slew_on, max_speed, dist_to_target + drive_sensor_right(), 
                             drive_sensor_right());
    }

    if (slew_min_when_it_enabled == 0) {
        slew_left.constants_set(slew_consts.distance_to_travel, slew_min);
        slew_right.constants_set(slew_consts.distance_to_travel, slew_min);
    }

    // ===== STEP 8: Set Angular PID Constants =====
    PID::Constants angle_const = odom_angularPID.constants_get();
    current_a_odomPID.constants_set(angle_const.kp, angle_const.ki, 
                                     angle_const.kd, angle_const.start_i);

    // ===== STEP 9: Record Initial "Past Target" Sign =====
    past_target = util::sgn(is_past_target(odom_target, odom_pose_get()));

    slew_min_when_it_enabled = pid_speed_max_get();

    // ===== STEP 10: Configure Internal PIDs for wait_until() =====
    int dir = current_drive_direction == REV ? -1 : 1;
    leftPID.target_set(l_start + (odom_look_ahead_get() * dir));
    rightPID.target_set(l_start + (odom_look_ahead_get() * dir));
    leftPID.exit = xyPID.exit;  // Switch over to xy pid exits
    rightPID.exit = xyPID.exit;
}
```

### Section IV: Background Control Loop

Once initialized, the background task `ez_auto_task()` continuously calls the control loop:

```cpp
void Drive::ez_auto_task() {
    while (true) {
        // Update odometry tracking
        ez_tracking_task();

        // Execute control based on current mode
        switch (drive_mode_get()) {
            case POINT_TO_POINT:
                ptp_task();
                break;
            case DRIVE:
                drive_pid_task();
                break;
            case TURN:
                turn_pid_task();
                break;
            // ... other modes ...
        }

        pros::delay(ez::util::DELAY_TIME);  // 10ms loop rate (100 Hz)
    }
}
```

### Section V: Point-to-Point Control Loop

The `ptp_task()` function runs at 100 Hz and computes motor outputs:

```cpp
void Drive::ptp_task() {
    // ===== STEP 1: Update Slew Rate Limiters =====
    slew_left.iterate(drive_sensor_left());
    slew_right.iterate(drive_sensor_right());
    double max_slew_out = fmax(slew_left.output(), slew_right.output());

    // ===== STEP 2: Calculate XY Error (Distance to Target) =====
    // Check if we've passed target using "past target" detection
    double temp_target = is_past_target(odom_target, odom_pose_get());
    int dir = (current_drive_direction == REV ? -1 : 1);  // Direction multiplier
    int flipped = util::sgn(temp_target) != util::sgn(past_target) ? -1 : 1;  // Did we pass?

    // Create "fake sensor value" for XY PID
    // This accumulates as we move, providing a monotonic signal for PID
    new_current_fake += xy_delta_fake * ((dir * flipped));
    
    // Compute XY PID (linear speed output)
    xyPID.compute_error(fabs(temp_target) * dir * flipped, new_current_fake);

    // ===== STEP 3: Calculate Heading Error =====
    pose ptf = point_to_face[!ptf1_running];
    double a_target = util::absolute_angle_to_point(ptf, odom_pose_get());
    a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
    double wrapped_a_target = a_target - odom_theta_get();
    
    // Compute angular PID (turning power output)
    current_a_odomPID.compute_error(wrapped_a_target, odom_theta_get());

    // ===== STEP 4: Apply Speed Limits and Min Speed =====
    double xy_out = xyPID.output;
    xy_out = util::clamp(xy_out, max_slew_out);  // Respect slew limit

    // Apply minimum speed if far from target
    if (fabs(xyPID.error) > (xyPID.exit.small_error > 0 ? xyPID.exit.small_error : 1.0)) {
        if (fabs(xy_out) < odom_min_speed) 
            xy_out = util::sgn(xy_out) * odom_min_speed;
    }

    // ===== STEP 5: Apply Drive Boost (if configured) =====
    // When robot is far from target but XY output is weak, boost it
    double distance_to_target = util::distance_to_point(odom_target, odom_pose_get());
    if (odom_drive_boost_threshold > 0.0 && 
        fabs(xy_out) < odom_drive_boost_threshold && 
        distance_to_target > 20.0) {
        xy_out *= odom_drive_boost_multiplier;
        xy_out = util::clamp(xy_out, max_slew_out);  // Re-clamp after boost
    }

    // ===== STEP 6: Apply Turn Bias =====
    // Scale down linear speed when heading error is large
    if (odom_turn_bias_enabled()) {
        double scale = 1.0 - ((1.0 - cos(util::to_rad(current_a_odomPID.error))) 
                              / odom_turn_bias_amount);
        xy_out *= scale;
    }

    double a_out = current_a_odomPID.output;

    // ===== STEP 7: Vector Scaling (Linear + Angular) =====
    // Scale so neither output exceeds max speed
    double faster_side = fmax(fabs(xy_out), fabs(a_out));
    if (faster_side > max_slew_out) {
        xy_out *= (max_slew_out / faster_side);
        a_out *= (max_slew_out / faster_side);
    }

    // ===== STEP 8: Combine into Differential Drive Outputs =====
    double l_out = xy_out + a_out;  // Left = forward + counter-clockwise turn
    double r_out = xy_out - a_out;  // Right = forward - counter-clockwise turn

    // Final vector scaling to respect motor voltage limits
    faster_side = fmax(fabs(l_out), fabs(r_out));
    if (faster_side > max_slew_out) {
        l_out *= (max_slew_out / faster_side);
        r_out *= (max_slew_out / faster_side);
    }

    // ===== STEP 9: Set Motor Voltages =====
    if (drive_toggle)
        private_drive_set(l_out, r_out);

    // ===== STEP 10: Update Internal PIDs for wait_until() =====
    leftPID.compute(drive_sensor_left());
    rightPID.compute(drive_sensor_right());
}
```

**Control Loop Visualization:**
```
[Odometry] → (x, y, θ)
     ↓
[Lookahead Calc] → point to face
     ↓
[XY PID] ────→ linear_speed ──┐
                               ├→ [Vector Scaling] → [Motor Outputs]
[Angular PID] → turn_power ────┘
     ↑
[Turn Bias] (scales linear_speed based on heading error)
```

### Section VI: Exit Condition Check

Movement terminates when exit conditions are met. This is handled by the PID exit condition system (covered in PID documentation). The XY PID error is checked against:

- **Small error threshold:** Position tolerance (e.g., 3 inches)
- **Small exit time:** Time within tolerance before exiting (e.g., 150ms)
- **Big error threshold:** Large error tolerance for faster exit (e.g., 5 inches)
- **Big exit time:** Time within large tolerance (e.g., 500ms)
- **Velocity threshold:** Robot velocity near zero
- **Timeout:** Maximum time allowed (e.g., 3000ms)

When any condition is satisfied, `xyPID.exit_condition()` returns `true`, causing `pid_wait()` to unblock and the movement to complete.

## ▲ Configuration & Tuning

### Lookahead Distance

**Effect on path curvature:**

```
Small lookahead (6 inches):
    Current ─→ ╰──→ Target
               tight curve

Medium lookahead (9 inches):
    Current ─→ ╰────→ Target
               smooth arc

Large lookahead (12 inches):
    Current ─→ ╰──────→ Target
               wide arc
```

**Tuning procedure:**
1. Start with 8-9 inches
2. Test movement and observe robot behavior:
   - **Oscillates/wobbles** → increase lookahead to 10-12 inches
   - **Takes wide, inefficient paths** → decrease lookahead to 6-7 inches
   - **Smooth but slightly overshoots** → increase lookahead slightly
3. Fine-tune in 0.5 inch increments

**Configuration:**
```cpp
chassis.odom_look_ahead_set(9_in);
```

---

### Turn Bias Amount

**Effect on speed vs. turning priority:**

| Turn Bias | Behavior                                    | Use Case                              |
|-----------|---------------------------------------------|---------------------------------------|
| 1.5-2.0   | Prioritizes turning, slows significantly    | Tight spaces, precise positioning     |
| 2.5-3.0   | Balanced (recommended)                      | General use, smooth paths             |
| 4.0-5.0   | Prioritizes speed, may overshoot            | Long distances, gentle curves         |

**Tuning procedure:**
1. Start with 2.5
2. Test sharp angle approaches (e.g., target 90° off current heading):
   - **Overshoots target** → decrease to 2.0
   - **Turns too slowly, takes forever** → increase to 3.0
3. Verify smooth behavior on straight approaches (shouldn't slow down much)

**Configuration:**
```cpp
chassis.odom_turn_bias_set(2.5);
chassis.odom_turn_bias_enable(true);
```

---

### XY and Angular PID Constants

**XY PID (Linear speed control):**
- Uses existing forward/backward drive PID constants
- Tune using standard drive PID tuning procedures first
- Typical values: `P=10-15, I=0, D=60-100`

**Angular PID (Heading control):**
- Should be less aggressive than standard turn PID
- Typical relationship: Angular P = 50-70% of Turn P
- Typical values: `P=5-6, I=0, D=30-40`

**Tuning procedure:**
1. Tune basic drive PID with `pid_drive_set()` (see PID documentation)
2. Tune basic turn PID with `pid_turn_set()`  
3. Set angular PID to 60% of turn PID P-value:
   ```cpp
   // If turn PID is (10, 0, 50)
   chassis.pid_odom_angular_constants_set(6, 0, 30);
   ```
4. Test point-to-point motion
5. Adjust:
   - **Robot wobbles side-to-side** → reduce angular P, increase angular D
   - **Robot doesn't turn aggressively enough** → increase angular P
   - **Overshoot target** → reduce XY P, increase XY D

**Configuration:**
```cpp
// XY PID (uses drive constants)
chassis.pid_drive_constants_forward_set(12, 0, 80);
chassis.pid_drive_constants_backward_set(12, 0, 80);

// Angular PID (dedicated)
chassis.pid_odom_angular_constants_set(5.5, 0, 35);
```

---

### Exit Conditions

**Parameters:**
```cpp
chassis.pid_odom_exit_condition_set(
    small_error,      // Position tolerance (inches)
    small_exit_time,  // Time to hold within tolerance (ms)
    big_exit_time,    // Time for looser tolerance (ms)
    timeout,          // Max movement duration (ms)
    velocity,         // Robot velocity threshold (0 = disabled)
    mA                // Motor current threshold (0 = disabled)
);
```

**Recommended starting values:**
```cpp
chassis.pid_odom_exit_condition_set(3_in, 150, 500, 750, 0, 0);
```

**Tuning for different scenarios:**

| Scenario              | small_error | small_exit_time | timeout | Notes                          |
|-----------------------|-------------|-----------------|---------|--------------------------------|
| Precise positioning   | 2 inches    | 200 ms          | 1000 ms | May timeout more often         |
| Fast completion       | 5 inches    | 100 ms          | 500 ms  | Less accurate final position   |
| Long-distance travel  | 4 inches    | 150 ms          | 2000 ms | Allow more time to settle      |
| Short-distance travel | 2 inches    | 100 ms          | 500 ms  | Quick movements                |

## ▲ Advanced Features

### Drive Boost

When robot is far from target but XY PID output is weak (common when heading error is large and turn bias reduces speed), drive boost applies a multiplier to prevent stalling:

**Configuration:**
```cpp
// Boost speed by 1.5x when output < 20 and distance > 20 inches
chassis.odom_drive_boost_set(20, 1.5);
```

**When it activates:**
- Robot is more than 20 inches from target
- XY PID output is below threshold (20 in this example)
- Heading error is large, causing turn bias to reduce speed significantly

**Effect:**
Prevents robot from moving too slowly when it's far from target but facing the wrong direction.

---

### Coordinate Flipping

For symmetrical autonomous routines, coordinates can be flipped:

```cpp
// Enable X-axis flipping (mirror left/right)
chassis.odom_x_flip(true);

// Enable Y-axis flipping (mirror front/back)
chassis.odom_y_flip(true);

// Enable theta flipping
chassis.odom_theta_flip(true);
```

**Usage:**
```cpp
void red_autonomous() {
    chassis.odom_xyt_set(-48_in, -48_in, 45_deg);
    chassis.odom_x_flip(false);
    run_shared_path();
}

void blue_autonomous() {
    chassis.odom_xyt_set(48_in, -48_in, 135_deg);
    chassis.odom_x_flip(true);  // Mirror path
    run_shared_path();
}

void run_shared_path() {
    chassis.pid_odom_ptp_set({{24_in, 0_in}, fwd, 110});  // Auto-flipped
    chassis.pid_wait();
}
```

---

### Turn Behavior Specification

Each movement can specify angle targeting behavior:

```cpp
chassis.pid_odom_ptp_set({{{12_in, 12_in}, fwd, 110, 0, shortest}});  // Shortest path turn
chassis.pid_odom_ptp_set({{{36_in, 24_in}, fwd, 110, 0, ccw}});       // Force CCW
chassis.pid_odom_ptp_set({{{48_in, 48_in}, fwd, 90, 0, raw}});        // Raw angle
```

**Behaviors:**
- `shortest`: Default, takes shortest angular path to lookahead point
- `raw`: Uses exact angle value, may rotate >180°
- `ccw`: Forces counter-clockwise rotation
- `cw`: Forces clockwise rotation

## ▲ Troubleshooting

### Problem: Robot oscillates/wobbles side-to-side

**Diagnosis:**
- Angular PID too aggressive
- Lookahead distance too small

**Solutions:**
1. Reduce angular PID P-term: `chassis.pid_odom_angular_constants_set(4.0, 0, 35);`
2. Increase angular PID D-term: `chassis.pid_odom_angular_constants_set(5.5, 0, 50);`
3. Increase lookahead: `chassis.odom_look_ahead_set(11_in);`

---

### Problem: Robot takes wide, inefficient curved paths

**Diagnosis:**
- Lookahead distance too large
- Turn bias too high (not turning aggressively)

**Solutions:**
1. Decrease lookahead: `chassis.odom_look_ahead_set(7_in);`
2. Decrease turn bias: `chassis.odom_turn_bias_set(2.0);`
3. Increase angular PID P-term slightly

---

### Problem: Robot overshoots target

**Diagnosis:**
- XY PID too aggressive
- Exit conditions too loose
- Speed too high

**Solutions:**
1. Reduce XY PID P-term: `chassis.pid_drive_constants_forward_set(10, 0, 80);`
2. Increase XY PID D-term: `chassis.pid_drive_constants_forward_set(12, 0, 100);`
3. Tighten exit conditions: `chassis.pid_odom_exit_condition_set(2_in, 150, 500, 750, 0, 0);`
4. Reduce maximum speed to 90-100

---

### Problem: Robot stops short of target

**Diagnosis:**
- Exit conditions too tight
- Odometry drift
- Mechanical friction

**Solutions:**
1. Loosen exit conditions: `chassis.pid_odom_exit_condition_set(4_in, 150, 500, 1000, 0, 0);`
2. Verify odometry accuracy (test basic drives, compare reported distance to actual)
3. Check for mechanical issues (wheels binding, low battery)

---

### Problem: Movement never completes (timeout)

**Diagnosis:**
- Target unreachable
- PID constants way off
- Odometry not updating

**Solutions:**
1. Verify target coordinates are reachable (no obstacles)
2. Test basic `pid_drive_set()` and `pid_turn_set()` - if those don't work, PTP won't either
3. Print odometry coordinates during movement to verify tracking is working
4. Increase timeout: `chassis.pid_odom_exit_condition_set(3_in, 150, 500, 2000, 0, 0);`

---

### Problem: Robot moves slowly/hesitantly

**Diagnosis:**
- Turn bias too low (reducing speed excessively)
- Minimum speed set too low
- Slew limiting too conservative

**Solutions:**
1. Increase turn bias: `chassis.odom_turn_bias_set(3.0);`
2. Increase minimum speed: Set `min_speed` parameter in movement call
3. Reduce slew distance or disable slew: `chassis.pid_odom_ptp_set(target, false);`

## ▲ Integration with Existing Documentation

This documentation builds upon:

**Odometry PRD & Implementation:**
- Position tracking provides real-time $(x, y, \theta)$ feedback
- Coordinate system conventions established in odometry docs apply here
- Tracking wheel configuration affects point-to-point accuracy

**PID Controller PRD & Implementation:**
- XY PID and angular PID reuse PID class infrastructure
- Exit condition system from PID docs applies to point-to-point motions
- PID tuning procedures from basic motions inform PTP tuning

**Autonomous Motion Algorithms PRD & Implementation:**
- Point-to-point extends basic motions (drive, turn, swing)
- Slew rate limiting from basic motions carries over
- State machine from Drive class manages mode transitions

**Driver Control PRD & Implementation:**
- Brake modes set in driver control apply during autonomous point-to-point motions
- Current limiting and thermal protection remain active

## ▲ Tests & Performance Analysis

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Issues

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Version History

[Note to LLM: IGNORE. This section will be handled separately.]
