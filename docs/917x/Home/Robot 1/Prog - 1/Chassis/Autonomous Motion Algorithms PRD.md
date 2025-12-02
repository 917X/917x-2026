# Autonomous Motion Algorithms PRD

|                       |            |
|:----------------------|:-----------|
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- Autonomous motion algorithms enable the robot to execute precise, repeatable movements during the 15-second autonomous period without human input.
- These algorithms combine PID control with sensor feedback to perform fundamental motion primitives: linear drives, in-place turns, and swing turns.
- Motion smoothness and consistency are enhanced through slew rate limiting and configurable angle targeting behaviors.

## ▲ Success metrics

|  **Goal**                                                                                          |  **Metric**                                                                                                                                     |
|:---------------------------------------------------------------------------------------------------|:------------------------------------------------------------------------------------------------------------------------------------------------|
| The robot can drive straight forward and backward for precise distances                            | Linear drives complete within ±1 inch of target distance with minimal deviation from straight line                                             |
| The robot can turn in place to precise headings                                                    | Point turns complete within ±3 degrees of target angle                                                                                         |
| The robot can execute arc turns (swing turns) using one side as a pivot                            | Swing turns complete within ±3 degrees with the stationary side remaining within ±1 inch of starting position                                  |
| Movements start smoothly without wheel slip or tipping                                             | Slew rate limiting prevents sudden accelerations, ramping motor output gradually over configurable distance                                    |
| The robot can chain multiple movements together fluidly                                            | Motion chaining allows next movement to begin before current movement fully completes, reducing dead time between actions                      |
| Angle targeting adapts to current orientation                                                      | Shortest path, left-only, right-only, and raw angle behaviors provide flexibility for different strategic needs                                |
| Movements terminate reliably without infinite loops                                                | PID exit conditions based on position error, velocity, and motor current ensure all movements complete or fail gracefully                      |
| Heading correction maintains straight drives                                                       | Optional heading PID compensates for drift during linear movements, keeping robot aligned to initial orientation                               |

## ▲ Implementation Assumptions

- The `Drive` class is properly instantiated with motors, IMU, and encoders
- PID constants and exit conditions are configured for each motion type
- The robot is on a level surface (IMU drift on inclines may affect turns)
- Motors respond predictably and symmetrically (both sides have similar characteristics)

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**                     |  **Importance**    |  **Notes**                                                                              |
|:-------------------------------------|:-------------------|:----------------------------------------------------------------------------------------|
| Linear Drive PID Implementation      | HIGH               | Core autonomous movement - drive forward/backward to target distance                    |
| Point Turn PID Implementation        | HIGH               | Core autonomous movement - turn in place to target angle                                |
| Swing Turn PID Implementation        | HIGH               | Arc turns using one side as pivot                                                       |
| Heading Correction System            | HIGH               | Keep robot straight during linear drives                                                |
| Slew Rate Limiting                   | MEDIUM             | Smooth acceleration to prevent tipping and wheel slip                                   |
| Angle Targeting Behaviors            | MEDIUM             | Shortest path, left-only, right-only, raw angle targeting                               |
| Relative vs Absolute Targeting       | MEDIUM             | Set targets relative to current position or absolute heading                            |
| Motion Chaining                      | MEDIUM             | Start next movement before current completes for fluid autonomous routines              |
| Wait Functions                       | HIGH               | Blocking wait until movement completes via exit conditions                              |
| Wait Until Functions                 | MEDIUM             | Early exit when robot reaches intermediate waypoint                                     |

## ▲ Design Overview Diagram

**Diagram Instructions:**
Create a flowchart showing autonomous motion execution:

1. **User Code:** "Call Motion Function (e.g., pid_drive_set, pid_turn_set)"
2. **Motion Setup:**
   - "Configure PID Constants"
   - "Configure Exit Conditions"
   - "Initialize Slew Rate Limiter"
   - "Set Drive Mode (DRIVE, TURN, or SWING)"
3. **Background Task Loop** (runs continuously at 10ms):
   - "Read Sensors (Encoders, IMU)"
   - "Compute PID Output"
   - "Apply Slew Limiting"
   - "Heading Correction (if enabled)"
   - "Set Motor Voltages"
   - "Check Exit Conditions"
4. **Wait Function:** "chassis.pid_wait()"
   - Blocks until exit condition met
5. **End:** "Movement Complete, Motors Stop"

Show the background task as a separate parallel process that runs independently from user code.

## ▲ Motion Types

### 1. Linear Drive (Forward/Backward)
**Function:** `chassis.pid_drive_set(distance, speed, slew_on, heading_on)`

**Purpose:** Drive straight forward or backward for a specified distance.

**Sensors Used:**
- Left/right motor encoders for distance tracking
- IMU for heading correction (optional)

**Control Strategy:**
- Separate PID controllers for left and right motor groups
- Optional heading PID adds corrective offset to keep robot straight
- Separate constants for forward vs backward to account for mechanical differences

**Example:**
```cpp
chassis.pid_drive_set(24_in, 110, true);  // Drive 24 inches forward at speed 110
chassis.pid_wait();
```

### 2. Point Turn (In-Place Rotation)
**Function:** `chassis.pid_turn_set(angle, speed, behavior, slew_on)`

**Purpose:** Turn in place to a target absolute or relative heading.

**Sensors Used:**
- IMU for angular position

**Control Strategy:**
- Single PID controller calculates turning power
- Left motors set to output, right motors set to negative output
- Angle wrapping logic ensures shortest path (unless overridden)

**Angle Behaviors:**
- `shortest`: Turn via shortest path (default)
- `raw`: Turn to exact angle value (may go >180°)
- `left_turn`: Force left (counter-clockwise) turn only
- `right_turn`: Force right (clockwise) turn only

**Example:**
```cpp
chassis.pid_turn_set(90_deg, 90);  // Turn to 90° heading via shortest path
chassis.pid_wait();
```

### 3. Swing Turn (Arc Turn)
**Function:** `chassis.pid_swing_set(type, angle, speed, opposite_speed, behavior, slew_on)`

**Purpose:** Turn in an arc by pivoting around one side of the drivetrain.

**Sensors Used:**
- IMU for angular position
- Motor encoders for pivot side position hold

**Control Strategy:**
- Swing PID calculates turning power for moving side
- Stationary side uses position-hold PID to prevent drift
- `opposite_speed` parameter can drive stationary side for tighter/wider arcs

**Swing Types:**
- `LEFT_SWING`: Left side stationary, right side moves
- `RIGHT_SWING`: Right side stationary, left side moves

**Example:**
```cpp
chassis.pid_swing_set(LEFT_SWING, 45_deg, 110);  // Swing 45° with left side as pivot
chassis.pid_wait();
```

## ▲ Advanced Features

### Slew Rate Limiting
**Purpose:** Gradually ramp motor output from minimum to maximum speed to prevent:
- Wheel slip during acceleration
- Robot tipping on sudden starts
- Mechanical stress on drivetrain

**Configuration:**
```cpp
// Ramp from 40 to max speed over first 3.5 inches of forward drives
chassis.slew_drive_constants_forward_set(3.5_in, 40);

// Use slew on this movement
chassis.pid_drive_set(48_in, 110, true);  // slew_on = true
```

**Note:** Detailed implementation of the slew rate limiter is documented separately. See the Slew Rate Limiter documentation for mathematical derivation and internal algorithms.

### Heading Correction
**Purpose:** Compensate for drivetrain asymmetry and friction to keep robot straight during linear drives.

**How It Works:**
- Heading PID tracks IMU angle during drive
- Corrective offset added to one side, subtracted from other
- Automatically disabled during turns and swings

**Configuration:**
```cpp
// Set heading correction constants
chassis.pid_heading_constants_set(11, 0, 50);

// Enable heading correction for this drive
chassis.pid_drive_set(48_in, 110, true, true);  // heading_on = true
```

### Motion Chaining
**Purpose:** Reduce dead time between sequential movements by starting the next movement before the previous fully completes.

**How It Works:**
- `pid_wait_quick()` exits when robot is "close enough" to target
- `pid_wait_quick_chain()` exits even earlier based on chain threshold
- Robot smoothly transitions into next movement while still approaching target

**Example:**
```cpp
chassis.pid_drive_set(24_in, 110, true);
chassis.pid_wait_quick_chain();  // Exit early to chain into next motion

chassis.pid_turn_set(90_deg, 90);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(12_in, 110, true);
chassis.pid_wait();  // Final movement waits fully
```

**Chain Thresholds:**
```cpp
// Set distance/angle at which chain exits trigger
chassis.pid_drive_chain_constant_set(3_in);   // Exit when within 3 inches
chassis.pid_turn_chain_constant_set(3_deg);   // Exit when within 3 degrees
chassis.pid_swing_chain_constant_set(5_deg);  // Exit when within 5 degrees
```

### Wait Until (Intermediate Waypoints)
**Purpose:** Exit movement early when robot crosses an intermediate position, useful for timing subsystem actions during movement.

**Example:**
```cpp
chassis.pid_drive_set(48_in, 110, true);

// Wait until 24 inches traveled (halfway)
chassis.pid_wait_until(24_in);
intake.set(Intake::INTAKING);  // Start intake at midpoint

chassis.pid_wait();  // Continue waiting until drive completes
```

## ▲ Typical Autonomous Routine Structure

```cpp
void autonomous() {
    // Reset all sensors and positions
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    
    // Movement 1: Drive forward
    chassis.pid_drive_set(24_in, 110, true);
    chassis.pid_wait();
    
    // Movement 2: Turn to face goal
    chassis.pid_turn_set(90_deg, 90);
    chassis.pid_wait();
    
    // Movement 3: Drive to goal
    chassis.pid_drive_set(18_in, 110, true);
    chassis.pid_wait_until(12_in);  // Start scoring early
    intake.set(Intake::SCORING);
    chassis.pid_wait();
    
    pros::delay(500);  // Wait for scoring to complete
    
    // Movement 4: Swing turn to exit
    chassis.pid_swing_set(LEFT_SWING, 45_deg, 110);
    chassis.pid_wait();
    
    // Movement 5: Drive away
    chassis.pid_drive_set(-30_in, 110, true);
    chassis.pid_wait();
}
```

## ◎ Out of Scope

- **Odometry-based motions:** Absolute position estimation, point-to-point navigation, pure pursuit path following (covered in separate Odometry documentation)
- **Curved paths:** Non-arc curved trajectories (requires odometry/pure pursuit)
- **Collision avoidance:** Reactive replanning based on obstacles
- **Vision-based targeting:** Camera-guided movements
- **Feedforward control:** Model-based acceleration compensation
- **Path planning algorithms:** A*, RRT, or other pathfinding methods
