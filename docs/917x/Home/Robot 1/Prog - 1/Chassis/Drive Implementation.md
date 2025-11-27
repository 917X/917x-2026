# Drive Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2025-11-01 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- The overarching structural implementation of the `Drive` class
- The components needed to construct the `Drive` class
- Components and functions that the drive class holds. **Details regarding their implementations are explained in their respective individual documentation entries.**

## ▲ Executive Summary

### Purpose & Need

- The robot needs to be programmed to be able to move, both in driver control and in autonomous.
- The Drive class accomplishes this by providing a high level, abstract system to interface with the chassis to perform actions necessary for moving the robot around.
- To ensure movement consistency during autonomous, the Drive class should also implement features (e.g. sensors and localization) that minimizes the effect of unexpected external disturbances such as friction and wheel slip.

## ▲ Architecture

### External Dependencies

|  **Component**    |  **Type**               |  **Purpose**                     |
|:------------------|:------------------------|:---------------------------------|
| `Pros::Motor`     | Kernel Library          | Basic motor functions interface  |
| `Pros::Imu`       | Kernel Library          | Inertial sensor interface        |
| `Pros::Rotation`  | Kernel Library          | Rotation sensor interface        |
| `Pros::Task`      | Kernel Library          | Multitasking/threading interface |
| `okapi::QAngle`   | Measurement Unit Helper | Angles with units (deg, rad)     |
| `okapi::QLength`  | Measurement Unit Helper | Length with units (in, m, cm)    |
| `okapi::QTime`    | Measurement Unit Helper | Time with units (msec, sec)      |

### Internal Dependencies

|  **Component**                       |  **Type**    |  **Purpose**                     |
|:-------------------------------------|:-------------|:---------------------------------|
| `PID` (See PID Entry)                | Class        | PID Controller class             |
| `slew` (See Slew Entry)              | Class        | Slew rate limiter class          |
| `tracking_wheel` (See Odom Entry)    | Class        | Tracking wheel abstraction       |
| `util`                               | Class        | Various utility functions        |
| `pose` (See Odom Entry)              | Struct       | 2d planar position (x, y) struct |
| `e_swing` (See Angular Motion Entry) | Enum         | Enum for swing turns             |
| `e_mode`                             | Enum         | Current drive mode Enum          |

### Interfaces

|  **Type**        |  **Name**                                                                                                               |
|:-----------------|:------------------------------------------------------------------------------------------------------------------------|
| Component Input  | `Motor`, `tracking_wheel`, `PID` components as well as user inputs for various functions implemented within this class. |
| Component Output | Physical drivetrain motor outputs                                                                                       |

## ▲ Design Spec

### Stateful System

- We try to make all of our systems stateful. This is good practice for readability, modularity, and makes it easier to debug.
- This means that operations for the `Drive` class and the physical drivetrain, including motions and movements, are classified and confined into finite states.
- The drivetrain will regress into its disabled state if it was **interrupted** or interfered during active movements.

### Finite Drive States

|  **State**       |  **Purpose**                                            |
|:-----------------|:--------------------------------------------------------|
| `DISABLE`        | All drive functions are completely disabled             |
| `DRIVE`          | Autonomous straight-line motion (fwd/rev)               |
| `TURN`           | Autonomous in-place turning motion                      |
| `SWING`          | Autonomous arc turn using only one drivetrain side      |
| `TURN_TO_POINT`  | Autonomously turn to face a distant 2d coordinate point |
| `POINT_TO_POINT` | Autonomously navigate to a distant 2d coordinate point  |
| `PURE_PURSUIT`   | Autonomously Follow a defined path through waypoints    |
| `INTERFERED`     | Status when the motions are unexpectedly interrupted    |

### Finite State Diagram

[Note to LLM: do not create the actual image. This will be handled separately. Instead, simply put instructions on how to create the diagram.]

### Features Overview

|  **Feature**               |  **Need / Purpose**                                                                                                                                                                                                                                            |
|:---------------------------|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Constructor                | The basis of the `Drive` chassis class. Constructs an instance of the `Drive` object with the relevant components: motors, hardware constants, and sensors, including tracking wheels.                                                                         |
| Hardware Component Storage | Stores the hardware components passed through the constructor. Motors, sensors, etc.                                                                                                                                                                           |
| PID Controllers            | PID (proportional / integral / derivative) controllers are used in all autonomous motions. More details could be found in the PID Controller entry.                                                                                                            |
| Odometry & Localization    | Odometry, or absolute position tracking, is needed in order to consistently estimate the position of the robot relative to the field. This would allow for more added precision and point-to-point motions. More details could be found in the Odometry entry. |
| Slew Rate limiters         | Slew rate limiting is the technique to limit the maximum acceleration of drivetrain motors per unit time. This prevents sudden, extreme changes in the drivetrain’s velocity that would otherwise cause undesirable outcomes such as tipping.                  |
| Core movement functions    | This covers the range of motions that the robot is able to perform during the autonomous period, including straight forward/reverse driving, in-place turns, arc swing turns, point-to-point coordinate motions, and pure pursuit.                             |
| Configuration Functions    | These functions configure certain constants and alter the behavior of every other function within the `Drive` class. Examples include functions that change PID controller constant values.                                                                    |
| Driver Control Functions   | These are responsible for controlling the drivetrain via driver input during the driver controller period.                                                                                                                                                     |

### Constructor Parameters

|  **Parameter**    |  **Units**         |  **Description**                                                               |
|:------------------|:-------------------|:-------------------------------------------------------------------------------|
| `left_ports`      | `std::vector<int>` | Port numbers for the left side drivetrain motors                               |
| `right_ports`     | `std::vector<int>` | Port numbers for the right-side drivetrain motors                              |
| `imu_port`        | `int`              | Port number for the inertial sensor                                            |
| `wheel_dia`       | `double`           | Wheel diameter of the drivetrain motors. Used for distance/speed calculations. |
| `ticks`           | `double`           | Motor cartridge rpm. Used for distance/speed calculations.                     |
| `ratio`           | `double`           | Gear ratio of the drivetrain. Used for distance/speed calculations.            |

### Feature UML Diagram
    
[Note to LLM: do not create the actual image. This will be handled separately. Instead, simply put instructions on how to create the diagram.]

## ▲ Implementation Overview
### File structure

```txt
.
├── include/
│   ├── lib/
│   │   ├── drive/
│   │   │   └── drive.hpp
│   │   ├── PID.hpp
│   │   ├── api.hpp
│   │   ├── sdcard.hpp
│   │   ├── slew.hpp
│   │   ├── tracking_wheel.hpp
│   │   └── util.hpp
│   ├── okapi/
│   │   └── ...
│   └── pros/
│       └── ...
├── sd-card/
│   └── pursuit.txt
└── src/
    └── lib/
        ├── drive/
        │   ├── set_pid/
        │   │   ├── set_drive_pid.cpp
        │   │   ├── set_odom_pid.cpp
        │   │   ├── set_pid.cpp
        │   │   ├── set_swing_pid.cpp
        │   │   └── set_turn_pid.cpp
        │   ├── drive.cpp
        │   ├── exit_conditions.cpp
        │   ├── pid_tasks.cpp
        │   ├── purepursuit_math.cpp
        │   ├── tracking.cpp
        │   └── user_input.cpp
        ├── PID.cpp
        ├── sdcard.cpp
        ├── slew.cpp
        ├── tracking_wheel.cpp
        └── util.cpp
```

### Interface Level Setup Functions

- Includes only surface level functions that sets up the `Drive` instance.
- Further documentation found within respective independent entries

|  **Name**                                                        |  **Type**    |  **Purpose**                                                                                           |
|:-----------------------------------------------------------------|:-------------|:-------------------------------------------------------------------------------------------------------|
| `Drive(…)`                                                       | Constructor  | Constructs the chassis with relevant components                                                        |
| `void initialize()`                                              | Function     | Initializes the `Drive` instance before a match                                                        |
| `void pid_constants_set(...)`                                    | Function     | Setup PID constants and behaviors for the `Drive` class                                                |
| `void slew_constants_set(…)`                                     | Function     | Setup slew limited constants and behaviors for the `Drive` class                                       |
| `void drive_mode_set(e_mode mode, bool stop_drive = true)`       | Function     | Switches the drive mode by `mode` enumeration. Stops the drivetrain by default if in `DISABLED` state. |
| `void drive_brake_set` `(pros::motor_brake_node_e_t brake_type)` | Function     | Sets the physical motor braking mode for all drivetrain motors. Usually between `HOLD` and `COAST`.    |
| `void drive_sensor_reset()`                                      | Function     | Resets all of the drivetrain sensors                                                                   |
| `void drive_imu_reset` `(double new_heading)`                    | Function     | Sets the absolute reference point of the drive inertial sensor                                         |
| `void drive_imu_calibrate`                                       | Function     | Calibrates the inertial sensor before a match.                                                         |

## ▲ Data Structures

### Instance Component Data Structures

```cpp
// * Motor Collections
std::vector<pros::Motor> left_motors;
std::vector<pros::Motor> right_motors;

// * Sensor Objects
pros::Imu imu;  // Inertial measurement unit

// Tracking wheel pointers
tracking_wheel* odom_tracker_left;
tracking_wheel* odom_tracker_right;
tracking_wheel* odom_tracker_front;
tracking_wheel* odom_tracker_back;

// * Slew Rate Controllers
slew slew_left;
slew slew_right;
slew slew_forward;
slew slew_backward;
slew slew_turn;
slew slew_swing_forward;
slew slew_swing_backward;
slew slew_swing;
```

### PID Controllers

```cpp
// * PID Controllers
// Basic motion PIDs
PID headingPID;           // Heading correction during drives
PID turnPID;              // Point turns
PID swingPID;             // Swing turns (unified)
PID forward_swingPID;     // Forward swing
PID backward_swingPID;    // Backward swing
PID fwd_rev_swingPID;     // Combined forward/reverse swing

// Drive PIDs
PID leftPID;              // Left side
PID rightPID;             // Right side
PID forward_drivePID;     // Forward drive
PID backward_drivePID;    // Backward drive
PID fwd_rev_drivePID;     // Combined forward/reverse drive

// Odometry PIDs
PID xyPID;                // Linear odometry movement
PID current_a_odomPID;    // Current odometry angular
PID boomerangPID;         // Boomerang movements
PID odom_angularPID;      // Odometry angular control

// Internal/utility PIDs
PID internal_leftPID;
PID internal_rightPID;
PID left_activebrakePID;  // Active brake for left
PID right_activebrakePID; // Active brake for right

```

### Instance Position Tracking Data Structures

```cpp
// Current positions 
pose odom_current = {0.0, 0.0, 0.0};        // {x, y, theta}
pose odom_target = {0.0, 0.0, 0.0};
pose odom_second_to_last = {0.0, 0.0, 0.0};
pose odom_start = {0.0, 0.0, 0.0};
pose odom_target_start = {0.0, 0.0, 0.0};
pose turn_to_point_target = {0.0, 0.0, 0.0};

// Helper poses 
pose l_pose{0.0, 0.0, 0.0};
pose r_pose{0.0, 0.0, 0.0};
pose central_pose{0.0, 0.0, 0.0};
```

### Instance State Value Enums

```cpp
// Primary state
e_mode mode;                    // DISABLE, DRIVE, TURN, SWING, etc.
e_swing current_swing;          // LEFT_SWING or RIGHT_SWING

// Motion direction
drive_directions current_drive_direction = fwd;

// Angle behaviors
e_angle_behavior current_angle_behavior = raw;
e_angle_behavior default_swing_type = raw;
e_angle_behavior default_turn_type = raw;
e_angle_behavior default_odom_type = shortest;
```

### Configuration Primitive Values

```cpp
double TICK_PER_REV;
double TICK_PER_INCH;
double CIRCUMFERENCE;
double CARTRIDGE;
double RATIO;
double WHEEL_DIAMETER;
double global_track_width = 0.0;
double IMU_SCALER = 1.0;
```

### Driver Control Parameters

```cpp
int JOYSTICK_THRESHOLD;
pros::motor_brake_mode_e_t CURRENT_BRAKE = pros::E_MOTOR_BRAKE_COAST;
int CURRENT_MA = 2500;
int max_speed;
double opcontrol_speed_max = 127.0;

// Curve Scaling 
double left_curve_scale;
double right_curve_scale;
```

## ▲ Implementation Explainer

### Section I. Setup & Declarations

1. Begin by importing Dependencies into `lib/drive/drive.hpp`

   ```cpp
   #pragma once

   // STL deps
   #include <functional>
   #include <iostream>
   #include <tuple>

   // lib deps
   #include "lib/PID.hpp"
   #include "lib/slew.hpp"
   #include "lib/tracking_wheel.hpp"
   #include "lib/util.hpp"

   // okapiLib (PROS builtin utility library) deps
   #include "okapi/api/units/QAngle.hpp"
   #include "okapi/api/units/QLength.hpp"
   #include "okapi/api/units/QTime.hpp"

   // kernel deps
   #include "pros/motor_group.hpp"
   #include "pros/motors.h"
   ```
2. Set up the `Drive` class with its constructor. In its base form, the constructor should be able to use internal motor encoders for Odometry tracking. This is to ensure compatibility across all possible drivetrain configurations that we could potentially design, with or without tracking wheels. If independent trackers are installed, we can manually pass their pointers after initialization. More information about this can be found within the Odometry entry,

   ```cpp
   class Drive {
    public:
       /**
   	 * Creates a Drive Controller using internal encoders.
   	 *
   	 * \param left_motor_ports
   	 * \param right_motor_ports
   	 * \param imu_port
   	 * \param wheel_diameter
   	 * \param ticks
   	 * \param ratio
   	 */
   	Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
   		  int imu_port, double wheel_diameter, double ticks,
   		  double ratio = 1.0);

   ```
3. Instantiate state variables and hardware components.

   ```cpp
       // Joystick deadzone threshold
   	int JOYSTICK_THRESHOLD;

   	// Global current brake mode
   	pros::motor_brake_mode_e_t CURRENT_BRAKE = pros::E_MOTOR_BRAKE_COAST;

   	// Current swing type 
   	e_swing current_swing;

   	// Left & right drivetrain motor storage vectors 
   	std::vector<pros::Motor> left_motors;
   	std::vector<pros::Motor> right_motors;

   	// Inertial sensor 
   	pros::Imu imu;

   	// Left, right, front, and back odometry tracking wheels
   	tracking_wheel *odom_tracker_left;
   	tracking_wheel *odom_tracker_right;
   	tracking_wheel *odom_tracker_front;
   	tracking_wheel *odom_tracker_back;
   ```
4. Instantiate PID and slew rate controllers

   ```cpp
    // PID objects
   	PID headingPID;
   	PID turnPID;
   	PID leftPID;
   	PID rightPID;
   	PID forward_drivePID;
   	PID backward_drivePID;
   	PID fwd_rev_drivePID;
   	PID swingPID;
   	PID forward_swingPID;
   	PID backward_swingPID;
   	PID fwd_rev_swingPID;
   	PID xyPID;
   	PID current_a_odomPID;
   	PID boomerangPID;
   	PID odom_angularPID;
   	PID internal_leftPID;
   	PID internal_rightPID;
   	PID left_activebrakePID;
   	PID right_activebrakePID;

   	// Slew rate controllers
   	slew slew_left;
   	slew slew_right;
   	slew slew_forward;
   	slew slew_backward;
   	slew slew_turn;
   	slew slew_swing_forward;
   	slew slew_swing_backward;
   	slew slew_swing;
   ```
5. Declare PID/slew controller setters and getters. This documentation will only show the general form of their declarations. As outlined above, there are a LOT of individual PID and slew rate controllers. Each PID & slew controller constant getter/setter for each PID/slew type is implemented using the **exact same** code and format and then carbon copied to have different names. Therefore only a few will be shown here for conciseness.

   ```cpp
       /**
   	 * Set the turn pid constants object.
   	 *
   	 * \param p
   	 *        proportional term
   	 * \param i
   	 *        integral term
   	 * \param d
   	 *        derivative term
   	 * \param p_start_i
   	 *        error threshold to start integral gain
   	 */
   	void pid_turn_constants_set(double p, double i = 0.0, double d = 0.0,
   								double p_start_i = 0.0);

   	/**
   	 * Returns PID constants with PID::Constants.
   	 */
   	PID::Constants pid_turn_constants_get();

   	/**
   	 * Set the forward pid constants object.
   	 *
   	 * \param p
   	 *        proportional term
   	 * \param i
   	 *        integral term
   	 * \param d
   	 *        derivative term
   	 * \param p_start_i
   	 *        error threshold to start integral gain
   	 */
   	void pid_drive_constants_forward_set(double p, double i = 0.0,
   										 double d = 0.0,
   										 double p_start_i = 0.0);

   	/**
   	 * Returns PID constants with PID::Constants.
   	 */
   	PID::Constants pid_drive_constants_forward_get();

       /**
   	 * Sets constants for slew for driving forward.
   	 *
   	 * Slew ramps up the speed of the robot until the set distance is traveled.
   	 *
   	 * \param distance
   	 *        the distance the robot travels before reaching max speed, an okapi
   	 * distance unit
   	 * \param min_speed
   	 *        the starting speed for the movement, 0 - 127
   	 */
   	void slew_drive_constants_forward_set(okapi::QLength distance,
   										  int min_speed);
   ```
6. Declare drive state setters/getters

   ```cpp
       // Current mode of the drive
   	e_mode mode;

   	/**
   	 * Sets current mode of drive.
   	 *
   	 * \param p_mode
   	 *        the new drive mode
   	 * \param stop_drive
   	 *        if the drive will stop when p_mode is DISABLED
   	 */
   	void drive_mode_set(e_mode p_mode, bool stop_drive = true);

       /**
   	 * Returns current mode of drive.
   	 */
   	e_mode drive_mode_get();

       /**
   	 * Changes the way the drive behaves when it is not under active user
   	 * control.
   	 *
   	 * \param brake_type
   	 *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST'
   	 * 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
   	 */
   	void drive_brake_set(pros::motor_brake_mode_e_t brake_type);

   	/**
   	 * Returns the brake mode of the drive in pros_brake_mode_e_t_.
   	 */
   	pros::motor_brake_mode_e_t drive_brake_get();
   ```
7. Declare drive initialization functions

   ```cpp
       /**
   	 * Calibrates imu and runs other tasks to initializes drivetrain for auton
   	 */
   	void initialize();

       /**
   	 * Calibrates the IMU
        * Returns false if calibration fails 
   	 */
   	bool drive_imu_calibrate();
       
   	/**
   	 * Reset all the chassis motors and tracking wheels
   	 * the start of your autonomous routine.
   	 */
   	void drive_sensor_reset();

       /**
   	 * Resets the current imu value.
   	 *
   	 * \param new_heading
   	 *        new heading value
   	 */
   	void drive_imu_reset(double new_heading = 0);
   ```

### II. Specific Function Implementations

1. `Drive` Constructor. This instantiates the drivetrain motors, imu, and class constants outlined above.

   ```cpp
   // Constructor for integrated encoders
   Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
                int imu_port, double wheel_diameter, double ticks, double ratio)
       : imu(imu_port),
         left_tracker(-1, -1, false),   // Default value
         right_tracker(-1, -1, false),  // Default value
         left_rotation(-1),
         right_rotation(-1) {

     // Turn off external trackers by default
     is_tracker = DRIVE_INTEGRATED;

     // Set ports to a global vector
     for (auto i : left_motor_ports) {
       pros::Motor temp((std::int8_t)abs(i));
       temp.set_reversed(util::reversed_active(i));
       left_motors.push_back(temp);
     }
     for (auto i : right_motor_ports) {
       pros::Motor temp((std::int8_t)abs(i));
       temp.set_reversed(util::reversed_active(i));
       right_motors.push_back(temp);
     }

     // Set constants for distance/speed calculation
     WHEEL_DIAMETER = wheel_diameter;
     RATIO = ratio;
     CARTRIDGE = ticks;
     TICK_PER_INCH = drive_tick_per_inch();

     // Helper function to set predefined default PID/Slew constants
     drive_defaults_set();
   }
   ```
2. PID controller setters. As mentioned above, only the function to modify one PID controller will be documented here. The rest contain the same exact implementation, only differing by name.

   ```cpp
   // Sets forward lateral driving PID constants
   void Drive::pid_drive_constants_forward_set(double p, double i, double d, double p_start_i) {
     forward_drivePID.constants_set(p, i, d, p_start_i);
   }

   // General PID constants setter
   void PID::constants_set(double p, double i, double d, double p_start_i) {
     constants.kp = p;
     constants.ki = i;
     constants.kd = d;
     constants.start_i = p_start_i;
   }

   // Get existing PID constants 
   PID::Constants PID::constants_get() { return constants; }

   // Set forward lateral driving slew Constants
   void Drive::slew_drive_constants_forward_set(okapi::QLength distance, int min_speed) {
     double dist = distance.convert(okapi::inch);
     slew_forward.constants_set(dist, min_speed);
   }

   // General slew constants setter
   void slew::constants_set(double distance, int minimum_speed) {
     constants.min_speed = minimum_speed;
     constants.distance_to_travel = distance;
   }
   ```
3. Drive State setters/getters

   ```cpp
   // Sets drive state. Stops drivetrain if in DISABLED state
   void Drive::drive_mode_set(e_mode p_mode, bool stop_drive) {
     mode = p_mode;
     if (mode == DISABLE && stop_drive)
       private_drive_set(0, 0); // direct motor power output setter 
   }

   // Gets current active drive state
   e_mode Drive::drive_mode_get() { return mode; }
   ```
4. Drive brake mode setters/getters

   ```cpp
   // Set brake modes
   void Drive::drive_brake_set(pros::motor_brake_mode_e_t brake_type) {
     CURRENT_BRAKE = brake_type;
     for (auto i : left_motors) {
       i.set_brake_mode(brake_type);  
     }
     for (auto i : right_motors) {
       i.set_brake_mode(brake_type);  
     }
   }

   // Get current brake mode
   pros::motor_brake_mode_e_t Drive::drive_brake_get() {
     return CURRENT_BRAKE;
   }
   ```
5. Drive initializations functions

   ```cpp
   // Drive init helper
   void Drive::initialize() {
     opcontrol_curve_sd_initialize();
     drive_imu_calibrate();
     drive_sensor_reset();
   }

   // Calibrates imu. Returns false if calibration fails. 
   bool Drive::drive_imu_calibrate() {
     imu_calibration_complete = false;
     imu.reset();
     int iter = 0;
     bool current_status = imu.is_calibrating();
     bool last_status = current_status;
     bool successful = false;
     
     // Loops until calibration ends
     // Fail calibration if it takes too long
     while (true) {
       iter += util::DELAY_TIME;

       // Check for changes in calibration state 
       if (!successful) {
         last_status = current_status;
         current_status = imu.is_calibrating();
         successful = !current_status && last_status ? true : false;
       }

       if (iter >= 2000) {
         if (successful) {
           break;
         }
         if (iter >= 3000) {
           // timed out. calibration fails
           return false;
         }
       }
       pros::delay(util::DELAY_TIME);
     }
     
     // Calibration complete
     return true;
   }
   ```
6. Drive sensor reset functions

   ```cpp
   // Zeroes ALL drivetrain sensors and motors 
   void Drive::drive_sensor_reset() {
     // Zero active brake constants
     left_activebrakePID.target_set(0.0);
     right_activebrakePID.target_set(0.0);

     // Reset odometry tracking 
     h_last = 0.0;
     l_last = 0.0;
     r_last = 0.0;
     t_last = 0.0;

     // Reset sensors
     left_motors.front().tare_position();
     right_motors.front().tare_position();

     // Reset all odometry encoders if they are enabled 
     if (odom_tracker_left_enabled) odom_tracker_left->reset();
     if (odom_tracker_right_enabled) odom_tracker_right->reset();
     if (odom_tracker_front_enabled) odom_tracker_front->reset();
     if (odom_tracker_back_enabled) odom_tracker_back->reset();
   }

   // Resets the inertial sensor position 
   void Drive::drive_imu_reset(double new_heading) {
     imu.set_rotation(new_heading);
     angle_rad = util::to_rad(new_heading);
     t_last = angle_rad; // updates prev heading field
   }
   ```

## ▲ Tests & Performance Analysis

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Issues

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Version History

[Note to LLM: IGNORE. This section will be handled separately.]
