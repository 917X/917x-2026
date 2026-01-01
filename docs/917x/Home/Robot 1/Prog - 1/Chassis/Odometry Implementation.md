# Odometry Implementation (Absolute Position Tracking)

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | PROD       |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- The mathematical foundations of differential drive odometry
- How tracking wheels measure robot displacement
- The coordinate transformation algorithms used to calculate position
- Integration with the Drive class for autonomous navigation
- Configuration and usage examples

## ▲ Executive Summary

### Purpose & Need

- In competitive robotics, achieving consistent autonomous performance requires the robot to know where it is on the field at all times. Without this knowledge, the robot can only execute "blind" movements based on encoder counts and gyroscope angles, which accumulate error over time.
- Odometry solves this problem by continuously computing the robot's absolute position $(x, y, \theta)$ relative to a known starting point on the field. This enables sophisticated autonomous behaviors like point-to-point navigation, dynamic path following, and recovery from disturbances.
- The odometry system acts as the robot's "sense of location," allowing it to make intelligent navigation decisions during the autonomous period.

## ▲ Architecture

### External Dependencies

|  **Component**     |  **Type**               |  **Purpose**                                           |
|:-------------------|:------------------------|:-------------------------------------------------------|
| `pros::Imu`        | Kernel Library          | Provides heading (orientation angle) measurements     |
| `pros::Rotation`   | Kernel Library          | Interfaces with rotation sensors on tracking wheels   |
| `pros::adi::Encoder` | Kernel Library        | Interfaces with quadrature encoders on tracking wheels |
| `pros::Task`       | Kernel Library          | Enables background task for continuous position update |
| `okapi::QLength`   | Measurement Unit Helper | Length units with automatic conversions (in, cm, m)    |
| `okapi::QAngle`    | Measurement Unit Helper | Angle units with automatic conversions (deg, rad)      |

### Internal Dependencies

|  **Component**      |  **Type**    |  **Purpose**                                          |
|:--------------------|:-------------|:------------------------------------------------------|
| `tracking_wheel`    | Class        | Abstracts encoder/rotation sensor hardware            |
| `pose`              | Struct       | Data structure for $(x, y, \theta)$ coordinates       |
| `util`              | Class        | Utility functions for coordinate transformations      |

### Interfaces

|  **Type**        |  **Name**                                                                                                               |
|:-----------------|:------------------------------------------------------------------------------------------------------------------------|
| Component Input  | Tracking wheel encoder values, IMU heading, Drive class configuration parameters                                        |
| Component Output | Current robot pose $(x, y, \theta)$, accessible via getter functions (`odom_x_get()`, `odom_y_get()`, `odom_theta_get()`) |

## ▲ Design Spec

### Coordinate System Convention

The odometry system uses a field-centric coordinate system:
- **Origin $(0, 0)$**: Center of the field
- **X-axis**: Horizontal direction (positive to the right)
- **Y-axis**: Vertical direction (positive forward)
- **Theta $(\theta)$**: Heading angle in degrees (0° = facing positive Y direction)

This coordinate system matches standard competition field layouts where the field is typically 12 feet × 12 feet (144 inches × 144 inches), with positions measured from the center.

### Sensor Configuration Options

The odometry system supports multiple tracking wheel configurations:

|  **Configuration**                |  **Description**                                                                                  |  **Accuracy**  |
|:----------------------------------|:--------------------------------------------------------------------------------------------------|:---------------|
| Two Vertical Trackers             | Left and right tracking wheels parallel to forward direction                                      | Good           |
| Two Vertical + One Horizontal     | Left/right vertical trackers plus front or back horizontal tracker (perpendicular to motion)      | Excellent      |
| Integrated Motor Encoders (IMEs)  | Uses drivetrain motor encoders (no dedicated tracking wheels)                                     | Fair           |
| Single Vertical + Horizontal      | One vertical tracker (left or right) with one horizontal tracker                                  | Good           |

Our implementation uses **two vertical tracking wheels** (left and right sides) for reliable position tracking.

### Mathematical Foundation

#### Step 1: Understanding Local Displacement

At any moment in time, the robot can measure:
- $\Delta s_L$: Distance traveled by the left tracking wheel
- $\Delta s_R$: Distance traveled by the right tracking wheel
- $\Delta \theta$: Change in heading angle from the IMU

From these measurements, we calculate the robot's local displacement—how far it moved in its own reference frame:

$$
\Delta s_{avg} = \frac{\Delta s_L + \Delta s_R}{2}
$$

This represents the forward distance the robot's center traveled. For a perfect straight line, $\Delta s_L = \Delta s_R$ and $\Delta \theta = 0$.

#### Step 2: Arc Motion and Local Coordinates

When the robot turns while moving, it follows a circular arc. To handle this, we need to account for the curvature of the path.

If the robot turned by $\Delta \theta$ during the movement, the actual straight-line displacement (chord length) is:

$$
\Delta s_{local} = \Delta s_{avg} \quad \text{if } \Delta \theta = 0
$$

But when turning ($\Delta \theta \neq 0$), we use:

$$
\Delta s_{local} = 2 \cdot \sin\left(\frac{\Delta \theta}{2}\right) \cdot \frac{\Delta s_{avg}}{\Delta \theta}
$$

**Why this formula?** When a robot drives in a circular arc, the chord (straight-line distance between start and end points) is shorter than the arc length measured by the wheels. The sine function captures this geometric relationship.

For very small turns, $\sin(x) \approx x$, so this simplifies back to $\Delta s_{avg}$.

#### Step 3: Tracking Wheel Offset Correction

Each tracking wheel is mounted at a distance from the robot's center of rotation. This offset affects the measurement:

- Let $d_{track}$ = distance from the tracking wheel to the robot's center (can be positive or negative)

When the robot rotates, the tracking wheel moves in a larger or smaller arc than the robot's center. We correct for this:

$$
\Delta s_{corrected} = \Delta s_{measured} - d_{track} \cdot \Delta \theta
$$

**Intuition:** If a tracking wheel is 5 inches to the left of center, and the robot turns right by 90°, that wheel travels a longer path than the center point. We subtract this extra distance to find the true center displacement.

#### Step 4: Coordinate Transformation to Global Frame

Now we have the robot's local displacement $\Delta s_{local}$ in its own reference frame. We need to transform this into the global field coordinates.

The robot's orientation at the start of this time step is $\theta_{prev}$, and at the end is $\theta_{current}$. We use the **midpoint angle**:

$$
\theta_{mid} = \theta_{prev} + \frac{\Delta \theta}{2}
$$

**Why the midpoint?** The robot is continuously rotating during motion. Using the midpoint angle gives a more accurate average orientation during the movement.

Now we decompose the local displacement into global X and Y components:

$$
\Delta x_{global} = \Delta s_{local} \cdot \cos(\theta_{mid})
$$

$$
\Delta y_{global} = \Delta s_{local} \cdot \sin(\theta_{mid})
$$

**Wait—there's a coordinate system quirk!** The mathematical convention has 0° pointing along the positive X-axis, but our robot convention has 0° pointing along positive Y (forward). To handle this, the actual implementation swaps and negates coordinates:

$$
\begin{aligned}
\Delta x_{field} &= -\Delta y_{global} \\
\Delta y_{field} &= \Delta x_{global}
\end{aligned}
$$

#### Step 5: Position Integration

Finally, we accumulate these incremental changes to track the robot's absolute position:

$$
\begin{aligned}
x_{current} &= x_{previous} + \Delta x_{field} \\
y_{current} &= y_{previous} + \Delta y_{field} \\
\theta_{current} &= \theta_{previous} + \Delta \theta
\end{aligned}
$$

This integration happens continuously in a background task running at approximately 10ms intervals.

### Complete Odometry Algorithm Flow

```
1. Store previous sensor values
2. Read current sensor values (encoders, IMU)
3. Calculate deltas: Δs_L, Δs_R, Δθ
4. Apply tracking wheel offset corrections
5. Calculate local displacement considering arc motion
6. Determine midpoint orientation angle
7. Transform local displacement to global coordinates
8. Integrate to update absolute position (x, y, θ)
9. Repeat continuously in background task
```

### Handling Different Tracking Configurations

The implementation includes logic to choose the best position estimate based on available sensors:

**Priority hierarchy:**
1. If only one vertical tracker is enabled (left or right), use that tracker's position
2. If both vertical trackers enabled:
   - For IME (motor encoders), use central pose (average of left and right)
   - For dedicated trackers, use configured preference (defaults to left)
3. Add horizontal tracker contribution (if present) to improve X-axis accuracy

This flexibility allows the system to work with various hardware setups.

## ▲ Implementation Overview

### File structure

```txt
.
├── include/
│   └── EZ-Template/
│       ├── tracking_wheel.hpp
│       ├── util.hpp (contains pose struct)
│       └── drive/
│           └── drive.hpp (contains odometry functions)
└── src/
    └── EZ-Template/
        ├── tracking_wheel.cpp
        └── drive/
            └── tracking.cpp (core odometry calculations)
```

### Data Structures

#### The `pose` Structure

The fundamental data structure for representing position:

```cpp
struct pose {
  double x;      // X coordinate in inches
  double y;      // Y coordinate in inches  
  double theta;  // Heading angle in degrees
};
```

#### The `tracking_wheel` Class

Abstraction layer for encoder hardware:

```cpp
class tracking_wheel {
  // Hardware interfaces
  pros::adi::Encoder adi_encoder;      // ADI quadrature encoder
  pros::Rotation smart_encoder;         // V5 rotation sensor
  
  // Configuration
  double wheel_diameter;                // Diameter in inches
  double distance_to_center;            // Offset from robot center
  double ratio;                         // Gear ratio
  
  // Key methods
  double get();                         // Returns distance traveled in inches
  void reset();                         // Resets encoder to zero
  double distance_to_center_get();      // Returns offset distance
};
```

#### Position Storage Variables

```cpp
// Current absolute position (continuously updated)
pose odom_current = {0.0, 0.0, 0.0};

// Individual poses for different tracking methods
pose l_pose{0.0, 0.0, 0.0};           // Left tracker pose
pose r_pose{0.0, 0.0, 0.0};           // Right tracker pose  
pose central_pose{0.0, 0.0, 0.0};     // Central IME pose

// Previous sensor readings (for delta calculation)
float l_last = 0.0;                    // Last left encoder value
float r_last = 0.0;                    // Last right encoder value
float h_last = 0.0;                    // Last horizontal encoder value
float t_last = 0.0;                    // Last theta (heading) value
```

## ▲ Implementation Explainer

### Section I. Tracking Wheel Setup

#### 1. Tracking Wheel Construction

Tracking wheels are initialized with their hardware configuration:

```cpp
// Example: Create a tracking wheel using V5 Rotation sensor
tracking_wheel leftTracker(
    -15,        // Port 15 (negative = reversed)
    2.75,       // 2.75" wheel diameter
    -5.625,     // 5.625" left of center (negative = left side)
    1.0         // 1:1 gear ratio (direct drive)
);
```

#### 2. Attaching Trackers to Drive Chassis

The tracking wheels are registered with the Drive class:

```cpp
// Set left tracking wheel
chassis.odom_tracker_left = &leftTracker;

// Set right tracking wheel (if using two vertical trackers)
chassis.odom_tracker_right = &rightTracker;

// Set horizontal tracker (optional, for improved accuracy)
chassis.odom_tracker_front = &horizontalTracker;  // or odom_tracker_back
```

#### 3. Configuration Constants

Set the chassis track width (distance between left/right wheels):

```cpp
// 11.25 inches between wheel centers
chassis.drive_width_set(11.25);
```

This value is critical for accurate rotation calculations.

### Section II. Core Odometry Calculations

#### The Main Tracking Task

The odometry system runs continuously in a background task called by the Drive class. Here's the simplified flow:

```cpp
void Drive::ez_tracking_task() {
  // Exit if odometry disabled or IMU not ready
  if (!imu_calibration_complete || !odometry_enabled) {
    // Reset all "last" values to zero
    return;
  }
  
  // 1. Get current sensor readings
  float l_current = /* left tracker position */;
  float r_current = /* right tracker position */;
  float t_current = /* IMU heading in radians */;
  
  // 2. Calculate deltas (change since last update)
  float l_delta = l_current - l_last;
  float r_delta = r_current - r_last;
  float t_delta = t_current - t_last;
  
  // 3. Update "last" values for next iteration
  l_last = l_current;
  r_last = r_current;  
  t_last = t_current;
  
  // 4. Solve for position change
  pose l_pose_delta = solve_xy_vert(
      l_track_width,   // Offset from center
      t_current,       // Current angle
      l_delta,         // Distance traveled
      t_delta          // Angle change
  );
  
  // 5. Accumulate position changes
  l_pose.x += l_pose_delta.x;
  l_pose.y += l_pose_delta.y;
  
  // 6. Update current odometry position
  odom_current.x = l_pose.x;  // (or r_pose/central_pose based on config)
  odom_current.y = l_pose.y;
  odom_current.theta = /* IMU angle in degrees */;
}
```

#### The Position Solver: `solve_xy_vert()`

This function implements the core mathematical transformations explained earlier:

```cpp
pose Drive::solve_xy_vert(
    float p_track_width,    // Distance from wheel to center
    float current_t,        // Current heading angle (radians)
    float delta_vert,       // Distance traveled by tracker
    float delta_t           // Change in heading (radians)
) {
  pose output = {0.0, 0.0, 0.0};
  
  // Calculate local displacement considering arc motion
  float local_x = delta_vert;  // Default for straight motion
  float half_delta_t = 0.0;
  
  if (delta_t != 0) {
    half_delta_t = delta_t / 2.0;
    
    // Arc correction: chord length calculation
    float i = sin(half_delta_t) * 2.0;
    
    // Apply tracking wheel offset correction
    local_x = (delta_vert / delta_t - p_track_width) * i;
  }
  
  // Calculate midpoint orientation
  float alpha = current_t - half_delta_t;
  
  // Transform to global coordinates (standard math frame)
  float x = cos(alpha) * local_x;
  float y = sin(alpha) * local_x;
  
  // Convert from math standard to robot field coordinates
  // (swap axes and negate to match robot forward = +Y convention)
  output.x = -y;
  output.y = x;
  
  return output;
}
```

**Key points:**
- The `p_track_width` offset is subtracted from the displacement before arc correction
- The `sin(half_delta_t) * 2.0` term implements the chord length formula
- The coordinate swap `(-y, x)` converts from mathematical convention to field convention

#### Horizontal Tracker Integration

If a horizontal (perpendicular) tracker is present, it measures sideways motion:

```cpp
pose Drive::solve_xy_horiz(
    float p_track_width,
    float current_t,
    float delta_horiz,  // Horizontal displacement
    float delta_t
) {
  // Similar logic to solve_xy_vert, but for perpendicular motion
  // Uses -sin() and cos() for 90-degree rotated transformation
  
  float local_y = delta_horiz;
  
  if (delta_t != 0) {
    float half_delta_t = delta_t / 2.0;
    float i = sin(half_delta_t) * 2.0;
    // Note: + instead of - for horizontal offset
    local_y = (delta_horiz / delta_t + p_track_width) * i;
  }
  
  float alpha = current_t - half_delta_t;
  
  // 90-degree rotated transformation
  float x = -sin(alpha) * local_y;
  float y = cos(alpha) * local_y;
  
  output.x = -y;
  output.y = x;
  
  return output;
}
```

The horizontal tracker's contribution is added to the vertical tracker positions to improve X-axis accuracy.

### Section III. Configuration and Control Functions

#### Setting Robot Position

Before autonomous begins, set the starting position:

```cpp
// Set position to (45 inches, -6 inches, -90 degrees)
chassis.odom_xyt_set(45_in, -6_in, -90_deg);

// Alternative: set only X and Y, keep current heading
chassis.odom_xy_set(45_in, -6_in);

// Reset to origin
chassis.odom_reset();  // Sets position to (0, 0, 0)
```

#### Reading Current Position

Access the robot's current location:

```cpp
// Get individual components
double x = chassis.odom_x_get();        // X coordinate in inches
double y = chassis.odom_y_get();        // Y coordinate in inches  
double heading = chassis.odom_theta_get();  // Heading in degrees

// Get complete pose structure
pose current_position = chassis.odom_pose_get();
// Access: current_position.x, current_position.y, current_position.theta
```

#### Enabling/Disabling Odometry

Control when the odometry system runs:

```cpp
// Enable odometry tracking
chassis.odom_enable(true);

// Disable odometry (stops position updates)
chassis.odom_enable(false);

// Check if odometry is enabled
bool is_tracking = chassis.odom_enabled();
```

#### Sensor Selection Configuration

Choose which sensors to prioritize:

```cpp
// Prefer left tracking wheel when both left and right are available
chassis.odom_use_left = true;

// Prefer right tracking wheel
chassis.odom_use_left = false;
```

### Section IV. Integration with Autonomous Navigation

#### Point-to-Point Movement

Once odometry is tracking position, the Drive class can perform coordinate-based navigation:

```cpp
// Move to absolute field coordinate (24", 36")
chassis.pid_odom_set(
    {{24_in, 36_in, 0_deg}, fwd, 90},  // Target pose, direction, max speed
    true  // Wait for completion
);

// Turn to face a specific point
chassis.turn_to_point(48_in, 48_in);

// Chain multiple waypoints
chassis.pid_wait_quick_chain();
chassis.pid_odom_set({{30_in, 30_in}, fwd, 80}, true);
```

The PID controllers use the odometry position as feedback:
- **xyPID**: Controls linear distance to target
- **odom_angularPID**: Controls heading while driving to target
- **boomerangPID**: Implements smooth "boomerang" curves to targets

#### Example Autonomous Routine

```cpp
void autonomous() {
  // 1. Set starting position
  chassis.odom_xyt_set(45_in, -6_in, -90_deg);
  
  // 2. Navigate to scoring position  
  chassis.pid_odom_set({{22_in, -21_in, -160_deg}, fwd, 90}, true);
  
  // 3. Turn to face game element
  chassis.turn_to_point(36_in, -36_in);
  chassis.pid_wait();
  
  // 4. Drive to pickup location
  chassis.pid_odom_set({{36_in, -36_in}, fwd, 60}, true);
  
  // 5. Return to starting area using odometry coordinates
  chassis.pid_odom_set({{45_in, -6_in, -90_deg}, rev, 90}, true);
}
```

Throughout this routine, the odometry system continuously updates the robot's position, allowing it to navigate precisely even if bumped or if wheels slip slightly.

### Section V. Debugging and Monitoring

#### Console Output

During development, you can print odometry data to debug:

```cpp
void telemetry() {
  while (true) {
    pose current = chassis.odom_pose_get();
    
    printf("Odometry: X=%.2f  Y=%.2f  Theta=%.2f\n", 
           current.x, current.y, current.theta);
    
    pros::delay(100);  // Print every 100ms
  }
}

// Start telemetry task
pros::Task telemetryTask(telemetry);
```

#### Brain Screen Display

Display position on the V5 brain LCD:

```cpp
void screen_task() {
  while (true) {
    ez::screen_print(
        "X: " + std::to_string(chassis.odom_x_get()) + " in\n" +
        "Y: " + std::to_string(chassis.odom_y_get()) + " in\n" +
        "Heading: " + std::to_string(chassis.odom_theta_get()) + " deg",
        0  // Start at line 0
    );
    pros::delay(50);
  }
}
```

## ▲ Practical Considerations

### Measurement Accuracy

The accuracy of odometry depends on:
1. **Tracking wheel precision**: Larger diameter wheels provide finer resolution
2. **Mounting rigidity**: Any flex or play in tracking wheel mounts introduces error
3. **Surface friction**: Slippery tiles can cause tracking wheels to skip
4. **Update rate**: Higher task frequency (shorter delays) improves accuracy
5. **IMU calibration**: The inertial sensor must be properly calibrated before each match

### Common Issues and Solutions

**Problem:** Position drifts over time even on straight drives
- **Cause:** Tracking wheel slip or IMU calibration drift
- **Solution:** Add weight to tracking wheels for better traction; recalibrate IMU

**Problem:** Position jumps or exhibits large discontinuities
- **Cause:** Encoder wiring issues or sensor disconnections
- **Solution:** Check sensor connections; verify encoders are properly initialized

**Problem:** Turns show incorrect position changes
- **Cause:** Incorrect track width measurement or tracking wheel offset
- **Solution:** Physically remeasure distance between wheels; verify offset signs

**Problem:** Robot drives past target in autonomous
- **Cause:** PID tuning with odometry differs from basic PID
- **Solution:** Retune odometry-specific PID constants (see `pid_odom_drive_constants_set()`)

### Performance Optimization

The odometry task runs continuously in the background. To minimize computational overhead:

- Odometry calculations use simple arithmetic (no expensive trigonometry in tight loops)
- The task runs at 10ms intervals, balancing responsiveness with CPU usage
- Coordinate transformations are precomputed where possible

### Field-Relative vs Robot-Relative

It's important to understand the difference:

**Robot-relative** (without odometry):
```cpp
// "Drive forward 24 inches from wherever you are now"
chassis.pid_drive_set(24_in, 110);
```

**Field-relative** (with odometry):
```cpp
// "Go to the absolute field coordinate (24, 36)"  
chassis.pid_odom_set({{24_in, 36_in}, fwd, 90}, true);
```

Odometry enables field-relative commands, which are much more powerful for autonomous routines.

## ▲ Alignment with PRD Goals

Let's revisit the success metrics from the PRD:

| **Goal**                                                                                          | **Implementation**                                                                                                   | **Status** |
|:--------------------------------------------------------------------------------------------------|:---------------------------------------------------------------------------------------------------------------------|:-----------|
| The robot can accurately track its position on the field in real-time                             | Background task updates position every 10ms using encoder and IMU data                                               | ✅ Met      |
| Position tracking remains accurate despite wheel slip and external disturbances                   | Dedicated tracking wheels isolated from drivetrain traction issues                                                   | ✅ Met      |
| The odometry data integrates seamlessly with the Drive class                                      | Position accessible via `odom_x_get()`, `odom_y_get()`, `odom_pose_get()` functions                                 | ✅ Met      |
| Position coordinates are easily configurable for different starting positions                     | `odom_xyt_set()` function allows setting any starting position before autonomous                                     | ✅ Met      |
| The odometry implementation is mathematically sound and follows established best practices        | Implementation based on differential drive odometry equations from Purdue SIGBots and competitive robotics community | ✅ Met      |

## ▲ References and Further Reading

- [Purdue SIGBots Wiki - Odometry](https://wiki.purduesigbots.com/software/odometry)
- [5225A - Pilons Odometry Document](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)
- Differential drive kinematics in robotics literature
- VEX Forum discussions on tracking wheel configurations

## ▲ Conclusion

The odometry system transforms the robot from a simple motor-encoder-based machine into an intelligent agent that knows where it is and can navigate precisely to any point on the field. By continuously integrating local motion measurements into a global coordinate frame, odometry enables sophisticated autonomous behaviors that would be impossible with dead reckoning alone.

The mathematical foundation—though involving trigonometry and coordinate transformations—can be understood intuitively: the robot measures how far its wheels moved and which direction it's facing, then uses geometry to figure out where it ended up on the field. This simple concept, executed continuously and accurately, is the key to world-class autonomous performance in VEX robotics.
