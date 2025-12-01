# Driver Control Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- The driver control subsystem within the `Drive` class
- Controller input processing and mapping to motor outputs
- Exponential curve functions for enhanced low-speed precision
- Control modes: arcade (split/single) and tank drive
- Active brake implementation for holding position
- Real-time curve adjustment and SD card persistence

## ▲ Executive Summary

### Purpose & Need

- During the 1:45 driver control period, the robot must respond accurately and intuitively to human controller inputs.
- The driver control subsystem transforms raw joystick positions into motor voltages, applying curve functions for improved precision and filtering noise through deadzone thresholds.
- To prevent unintended drift when the driver releases the joysticks, an active brake PID controller maintains the robot's position.

## ▲ Architecture

### External Dependencies

|  **Component**           |  **Type**        |  **Purpose**                              |
|:-------------------------|:-----------------|:------------------------------------------|
| `pros::Controller`       | Kernel Library   | Controller input interface                |
| `pros::Motor`            | Kernel Library   | Motor control interface                   |
| `PID` (See PID Entry)    | Internal Class   | Active brake position holding             |

### Internal Dependencies

|  **Component**    |  **Type**    |  **Purpose**                                  |
|:------------------|:-------------|:----------------------------------------------|
| `drive_set()`     | Function     | Low-level motor voltage setter                |
| `drive_sensor_left()` / `drive_sensor_right()` | Function | Motor encoder position getters |
| `util::DELAY_TIME` | Constant    | Standard loop delay (10ms)                    |

### Interfaces

|  **Type**        |  **Name**                                                      |
|:-----------------|:---------------------------------------------------------------|
| Component Input  | Controller joystick analog values, button states              |
| Component Output | Motor voltage commands (-127 to 127) for left/right drivetrains |

## ▲ Design Spec

### Control Mode Architecture

The driver control system supports three primary control modes:

#### 1. Tank Drive
- **Left Stick Y-axis** → Left motor group voltage
- **Right Stick Y-axis** → Right motor group voltage
- Each side of the drivetrain is controlled independently
- Turning is accomplished by moving sticks in opposite directions

#### 2. Arcade Split (Default)
- **Left Stick Y-axis** → Forward/Reverse motion
- **Right Stick X-axis** → Turning motion
- Motor outputs calculated as:
  - Left motors: `forward + turn`
  - Right motors: `forward - turn`

#### 3. Arcade Single
- **Left Stick Y-axis** → Forward/Reverse motion
- **Left Stick X-axis** → Turning motion
- Same output calculation as arcade split, but single-stick control

### Signal Processing Pipeline

Each controller input passes through the following processing stages:

```
Raw Joystick Value
       ↓
Joystick Threshold Filter (deadzone)
       ↓
Exponential Curve Function
       ↓
Control Mode Calculation (tank vs arcade)
       ↓
Active Brake Check (if joysticks at zero)
       ↓
Speed Limit Application
       ↓
Motor Voltage Output (-127 to 127)
```

### Exponential Curve Function

The curve function provides enhanced low-speed control while maintaining full power at high speeds:

$$
f(x) = \left( e^{-\frac{c}{10}} + e^{\frac{|x| - 127}{10}} \left(1 - e^{-\frac{c}{10}}\right) \right) \cdot x
$$

Where:
- $x$ = joystick input (-127 to 127)
- $c$ = curve scaling factor (adjustable 0.0 to 10.0+)
- Higher $c$ values = more aggressive curve = finer low-speed control

**Curve Behavior:**
- When $c = 0.0$: Linear response (no curve applied)
- When $c = 3.0$: Moderate curve, good balance
- When $c = 7.0$: Aggressive curve, very fine low-speed control

### Active Brake System

When joystick inputs return to zero, an active brake PID controller engages:

- **Target:** Motor encoder position when joysticks were released
- **Output:** Corrective motor voltage to maintain position
- **Constants:** Proportional-only controller (default $k_p = 0.0$, disabled by default)

This prevents the robot from drifting due to external forces or imperfect motor braking.

## ▲ Implementation Overview

### File Structure

```txt
src/
└── lib/
    └── drive/
        └── user_input.cpp         # All driver control logic

include/
└── lib/
    └── drive/
        └── drive.hpp              # Function declarations
```

### Core Driver Control Functions

|  **Function**                                |  **Purpose**                                                |
|:---------------------------------------------|:------------------------------------------------------------|
| `opcontrol_tank()`                           | Tank drive control loop                                     |
| `opcontrol_arcade_standard(e_type)`          | Standard arcade control (left stick forward)                |
| `opcontrol_arcade_flipped(e_type)`           | Flipped arcade control (right stick forward)                |
| `opcontrol_joystick_threshold_iterate()`     | Core processing: threshold, curve, active brake, output     |
| `opcontrol_curve_left()` / `opcontrol_curve_right()` | Exponential curve functions                     |
| `clipped_joystick()`                         | Apply deadzone threshold to raw joystick value              |

### Configuration Functions

|  **Function**                                |  **Purpose**                                                |
|:---------------------------------------------|:------------------------------------------------------------|
| `opcontrol_joystick_threshold_set(int)`      | Set deadzone threshold (default: 5)                         |
| `opcontrol_curve_default_set(double, double)`| Set default curve values for left/right sticks              |
| `opcontrol_drive_activebrake_set()`          | Configure active brake PID constants                        |
| `opcontrol_speed_max_set(int)`               | Set maximum output speed (default: 127)                     |
| `opcontrol_drive_reverse_set(bool)`          | Toggle reversed driving mode                                |
| `opcontrol_arcade_scaling(bool)`             | Enable vector scaling to prevent output > 127               |

### Real-Time Curve Adjustment Functions

|  **Function**                                |  **Purpose**                                                |
|:---------------------------------------------|:------------------------------------------------------------|
| `opcontrol_curve_buttons_toggle(bool)`       | Enable/disable curve adjustment mode                        |
| `opcontrol_curve_buttons_left_set()`         | Set buttons for adjusting left curve                        |
| `opcontrol_curve_buttons_right_set()`        | Set buttons for adjusting right curve                       |
| `opcontrol_curve_buttons_iterate()`          | Process button presses, update curves, save to SD           |
| `opcontrol_curve_sd_initialize()`            | Load saved curve values from SD card at startup             |

## ▲ Data Structures

### Driver Control State Variables

```cpp
// Joystick filtering
int JOYSTICK_THRESHOLD;                    // Deadzone (default: 5)
double opcontrol_speed_max;                // Max output speed (default: 127.0)

// Curve scaling factors
double left_curve_scale;                   // Left stick curve (0.0 = linear)
double right_curve_scale;                  // Right stick curve (0.0 = linear)

// Control mode flags
bool is_tank;                              // True if using tank drive
bool is_reversed;                          // True if driving reversed
bool disable_controller;                   // True enables curve button adjustment
bool practice_mode_is_on;                  // True limits max speed to 120
bool arcade_vector_scaling;                // True enables vector normalization
```

### PID Controllers for Active Brake

```cpp
PID left_activebrakePID;                   // Left side position hold
PID right_activebrakePID;                  // Right side position hold
```

### Button State Tracking for Curve Adjustment

```cpp
struct button_ {
    pros::controller_digital_e_t button;   // Which button
    bool lock;                              // Debounce lock
    bool release_reset;                     // Release detection flag
    double hold_timer;                      // Hold duration tracking
    double release_timer;                   // Release duration tracking
    double increase_timer;                  // Auto-repeat timing
};

button_ l_increase_;                        // Left curve increase button
button_ l_decrease_;                        // Left curve decrease button
button_ r_increase_;                        // Right curve increase button
button_ r_decrease_;                        // Right curve decrease button
```

## ▲ Implementation Explainer

### Section I. Initialization & Setup

1. **Initialize the `Drive` object** with motors and sensors (see Drive Implementation doc)

2. **Configure driver control parameters** in `initialize()`:

   ```cpp
   void initialize() {
       // Disable curve adjustment buttons (default)
       chassis.opcontrol_curve_buttons_toggle(false);
       
       // Set active brake to 0 (disabled by default)
       chassis.opcontrol_drive_activebrake_set(0.0);
       
       // Set default curve values (0.0 = linear, no curve)
       chassis.opcontrol_curve_default_set(0.0, 0.0);
       
       // Load saved curves from SD card (if available)
       chassis.opcontrol_curve_sd_initialize();
       
       // Set joystick deadzone threshold (default is 5)
       chassis.opcontrol_joystick_threshold_set(5);
   }
   ```

3. **Set brake mode for driver control** at the start of `opcontrol()`:

   ```cpp
   void opcontrol() {
       chassis.drive_brake_set(MOTOR_BRAKE_COAST);
       
       while (true) {
           // Driver control loop
       }
   }
   ```

### Section II. Core Control Loop

The primary driver control loop runs continuously during the driver period:

```cpp
void opcontrol() {
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    
    while (true) {
        // Call appropriate control function
        chassis.opcontrol_arcade_standard(ez::SPLIT);
        
        // ... other robot control code ...
        
        pros::delay(ez::util::DELAY_TIME);  // 10ms delay
    }
}
```

### Section III. Arcade Split Control Implementation

The `opcontrol_arcade_standard()` function implements split-stick arcade control:

```cpp
void Drive::opcontrol_arcade_standard(e_type stick_type) {
    is_tank = false;
    opcontrol_drive_sensors_reset();
    
    // Process curve adjustment buttons (if enabled)
    opcontrol_curve_buttons_iterate();
    
    int fwd_stick, turn_stick;
    
    // Get joystick values based on control type
    if (stick_type == SPLIT) {
        // Left Y = forward, Right X = turn
        fwd_stick = opcontrol_curve_left(
            clipped_joystick(master.get_analog(ANALOG_LEFT_Y))
        );
        turn_stick = opcontrol_curve_right(
            clipped_joystick(master.get_analog(ANALOG_RIGHT_X))
        );
    }
    // SINGLE mode would use both axes from left stick
    
    // Calculate left/right motor outputs and apply
    opcontrol_joystick_threshold_iterate(
        fwd_stick + turn_stick,    // Left motors
        fwd_stick - turn_stick     // Right motors
    );
}
```

### Section IV. Tank Drive Implementation

Tank drive provides independent control of each drivetrain side:

```cpp
void Drive::opcontrol_tank() {
    is_tank = true;
    opcontrol_drive_sensors_reset();
    
    opcontrol_curve_buttons_iterate();
    
    // Apply curve to both sticks (left curve only in tank mode)
    int l_stick = opcontrol_curve_left(
        clipped_joystick(master.get_analog(ANALOG_LEFT_Y))
    );
    int r_stick = opcontrol_curve_left(
        clipped_joystick(master.get_analog(ANALOG_RIGHT_Y))
    );
    
    opcontrol_joystick_threshold_iterate(l_stick, r_stick);
}
```

### Section V. Joystick Threshold & Deadzone

The `clipped_joystick()` function eliminates controller drift:

```cpp
int Drive::clipped_joystick(int joystick) {
    // If joystick is within threshold, return 0
    return abs(joystick) < JOYSTICK_THRESHOLD ? 0 : joystick;
}
```

**Example:** With `JOYSTICK_THRESHOLD = 5`:
- Input: `-3` → Output: `0` (within deadzone)
- Input: `42` → Output: `42` (outside deadzone)
- Input: `127` → Output: `127` (max value)

### Section VI. Exponential Curve Function

The curve function applies exponential scaling for enhanced precision:

```cpp
double Drive::opcontrol_curve_left(double x) {
    if (left_curve_scale != 0) {
        return (
            powf(2.718, -(left_curve_scale / 10)) + 
            powf(2.718, (fabs(x) - 127) / 10) * 
            (1 - powf(2.718, -(left_curve_scale / 10)))
        ) * x;
    }
    return x;  // Linear if curve scale is 0
}
```

**Curve Response Comparison:**

| Joystick Input | Linear ($c=0$) | Moderate ($c=3$) | Aggressive ($c=7$) |
|:--------------:|:--------------:|:----------------:|:------------------:|
| 10             | 10             | ~4               | ~2                 |
| 30             | 30             | ~15              | ~6                 |
| 60             | 60             | ~38              | ~20                |
| 90             | 90             | ~68              | ~48                |
| 127            | 127            | 127              | 127                |

The aggressive curve provides much finer control at low speeds while still allowing full power at high inputs.

### Section VII. Final Output Processing

The `opcontrol_joystick_threshold_iterate()` function finalizes motor outputs:

```cpp
void Drive::opcontrol_joystick_threshold_iterate(int l_stick, int r_stick) {
    double l_out = 0.0, r_out = 0.0;
    
    // Check if joysticks are active
    if (abs(l_stick) > 0 || abs(r_stick) > 0) {
        // Update active brake targets
        if (left_activebrakePID.constants_set_check()) {
            opcontrol_drive_activebrake_targets_set();
        }
        
        // Apply reversal if enabled
        if (!is_reversed) {
            l_out = l_stick;
            r_out = r_stick;
        } else {
            l_out = -r_stick;
            r_out = -l_stick;
        }
    }
    // Joysticks at zero - engage active brake
    else {
        l_out = left_activebrakePID.compute(drive_sensor_left());
        r_out = right_activebrakePID.compute(drive_sensor_right());
    }
    
    // Apply vector scaling if enabled (prevents > 127)
    if (arcade_vector_scaling) {
        double faster_side = fmax(fabs(l_out), fabs(r_out));
        if (faster_side > 127.0) {
            l_out *= (127.0 / faster_side);
            r_out *= (127.0 / faster_side);
        }
    }
    
    // Apply speed limit
    l_out *= (opcontrol_speed_max / 127.0);
    r_out *= (opcontrol_speed_max / 127.0);
    
    // Clamp to max speed
    l_out = l_out > opcontrol_speed_max ? opcontrol_speed_max : l_out;
    r_out = r_out > opcontrol_speed_max ? opcontrol_speed_max : r_out;
    
    // Output to motors
    drive_set(l_out, r_out);
}
```

### Section VIII. Active Brake Implementation

When joysticks return to zero, active brake holds position:

```cpp
// Configure active brake (P-only controller)
chassis.opcontrol_drive_activebrake_set(2.0);  // kp = 2.0

// When joysticks are active, update brake targets
void Drive::opcontrol_drive_activebrake_targets_set() {
    left_activebrakePID.target_set(drive_sensor_left());
    right_activebrakePID.target_set(drive_sensor_right());
}

// When joysticks at zero, compute brake output
l_out = left_activebrakePID.compute(drive_sensor_left());
```

**How It Works:**
1. When driver releases joysticks, current encoder positions are saved as PID targets
2. PID continuously compares current position to target
3. If robot drifts (e.g., from collision), PID applies corrective voltage
4. When driver moves joysticks again, targets update immediately

### Section IX. Real-Time Curve Adjustment

Drivers can modify curve values on-the-fly using controller buttons:

```cpp
// Enable curve adjustment mode
chassis.opcontrol_curve_buttons_toggle(true);

// Set buttons for curve adjustment
chassis.opcontrol_curve_buttons_left_set(
    pros::E_CONTROLLER_DIGITAL_LEFT,   // Decrease
    pros::E_CONTROLLER_DIGITAL_UP      // Increase
);
chassis.opcontrol_curve_buttons_right_set(
    pros::E_CONTROLLER_DIGITAL_RIGHT,  // Decrease
    pros::E_CONTROLLER_DIGITAL_DOWN    // Increase
);
```

**Button Press Logic:**
- Single press: Increase/decrease curve by 0.1
- Hold for 500ms: Auto-repeat every 100ms
- Release for 250ms: Save new value to SD card (`/usd/left_curve.txt`)

The current curve values are displayed on the controller screen:

```
3.2         4.5
```
(Left curve: 3.2, Right curve: 4.5)

### Section X. SD Card Persistence

Curve values can be saved and loaded from the SD card:

```cpp
// Load saved curves at startup
void Drive::opcontrol_curve_sd_initialize() {
    FILE* l_usd_file_read;
    if ((l_usd_file_read = fopen("/usd/left_curve.txt", "r"))) {
        char l_buf[5];
        fread(l_buf, 1, 5, l_usd_file_read);
        left_curve_scale = std::stof(l_buf);
        fclose(l_usd_file_read);
    } else {
        save_l_curve_sd();  // Create file with default value
    }
    // Similar for right curve...
}

// Save curve value to SD card
void Drive::save_l_curve_sd() {
    FILE* usd_file_write = fopen("/usd/left_curve.txt", "w");
    std::string in_str = std::to_string(left_curve_scale);
    fputs(in_str.c_str(), usd_file_write);
    fclose(usd_file_write);
}
```

This allows drivers to maintain their preferred curve settings across matches.

## ▲ Usage Examples

### Basic Arcade Split Control

```cpp
void opcontrol() {
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    
    while (true) {
        chassis.opcontrol_arcade_standard(ez::SPLIT);
        pros::delay(ez::util::DELAY_TIME);
    }
}
```

### Tank Drive with Active Brake

```cpp
void initialize() {
    chassis.opcontrol_drive_activebrake_set(2.5);  // Enable active brake
}

void opcontrol() {
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    
    while (true) {
        chassis.opcontrol_tank();
        pros::delay(ez::util::DELAY_TIME);
    }
}
```

### Custom Curve Configuration

```cpp
void initialize() {
    // Set aggressive curve for precise control
    chassis.opcontrol_curve_default_set(5.0, 3.0);
    
    // Load saved curves from SD (overrides defaults if available)
    chassis.opcontrol_curve_sd_initialize();
    
    // Enable real-time adjustment
    chassis.opcontrol_curve_buttons_toggle(true);
}
```

### Practice Mode with Speed Limit

```cpp
void initialize() {
    chassis.opcontrol_joystick_practicemode_toggle(true);
    chassis.opcontrol_speed_max_set(90);  // Limit to 70% speed
}
```

## ▲ Behavioral Flowchart

**Flowchart Instructions:**
Create a detailed flowchart showing the complete signal processing pipeline:

1. **Start:** "Driver Moves Joystick"
2. **Input:** "Read Raw Analog Value (-127 to 127)"
3. **Process:** "Apply Deadzone Threshold" (decision diamond)
   - If |value| < threshold → Output 0
   - Else → Continue to next step
4. **Process:** "Apply Exponential Curve Function"
5. **Decision:** "Control Mode?"
   - Tank → "Left Stick → Left Motors, Right Stick → Right Motors"
   - Arcade → "Calculate: Left = Fwd + Turn, Right = Fwd - Turn"
6. **Decision:** "Joysticks Active?"
   - Yes → "Update Active Brake Targets"
   - No → "Compute Active Brake PID Output"
7. **Process:** "Apply Vector Scaling (if enabled)"
8. **Process:** "Apply Speed Limit"
9. **Process:** "Clamp to [-127, 127]"
10. **Output:** "Set Motor Voltages"
11. **End:** "Motors Respond"

Use color coding:
- Blue for input/output
- Yellow for processing
- Green for decision points
- Red for the active brake path

## ▲ Key Design Decisions

### 1. Arcade Split as Default

We chose arcade split over tank drive as the primary control mode because:
- **Intuitive:** Separation of translation (left stick) and rotation (right stick) matches mental models
- **Precision:** Easier to drive straight lines and execute precise positioning
- **Accessibility:** New drivers can achieve competency faster

### 2. Exponential Curve Function

The exponential curve formula was selected over linear or polynomial alternatives because:
- **Smooth response:** No discontinuities or sudden changes in sensitivity
- **Adjustable:** Single parameter ($c$) allows tuning from linear to very aggressive
- **Battle-tested:** Proven effective by team 5225A in VEX competitions

### 3. P-Only Active Brake

Active brake uses only proportional control (no integral or derivative) because:
- **Sufficient:** Position holding doesn't require zero steady-state error
- **Stable:** Simpler controller is less prone to oscillation or instability
- **Fast:** No integral windup delays or derivative noise issues

### 4. Real-Time Curve Adjustment

Optional curve button adjustment was included because:
- **Driver preference:** Different drivers have different sensitivity preferences
- **Adaptability:** Allows quick adjustment between matches without reflashing code
- **SD persistence:** Settings are retained across power cycles

## ▲ Troubleshooting

### Robot Drifts When Joysticks Released

**Cause:** Active brake disabled or improperly tuned

**Solution:** Enable active brake with appropriate $k_p$ value:
```cpp
chassis.opcontrol_drive_activebrake_set(2.0);  // Start with kp = 2.0
```

If still drifting, increase $k_p$ incrementally. If oscillating, decrease $k_p$.

### Robot Unresponsive to Small Joystick Inputs

**Cause:** Deadzone threshold too large or curve too aggressive

**Solutions:**
- Reduce threshold: `chassis.opcontrol_joystick_threshold_set(3);`
- Reduce curve scale: `chassis.opcontrol_curve_default_set(2.0, 2.0);`

### Cannot Drive Straight in Arcade Mode

**Cause:** Joystick drift or physical drivetrain asymmetry

**Solutions:**
- Increase deadzone: `chassis.opcontrol_joystick_threshold_set(8);`
- Use active brake to correct drift
- Calibrate physical drivetrain motors for equal output

### Robot Too Sensitive at High Speeds

**Cause:** Curve scale too low (too linear)

**Solution:** Increase curve scale for more gradual ramp:
```cpp
chassis.opcontrol_curve_default_set(4.0, 4.0);
```

### Curve Values Not Persisting

**Cause:** No SD card or SD card not properly formatted

**Solution:** Ensure SD card is FAT32 formatted and inserted before robot powers on. Check PROS terminal for SD card initialization messages.
