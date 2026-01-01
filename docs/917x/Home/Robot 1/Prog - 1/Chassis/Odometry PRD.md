# Odometry PRD (Absolute Position Tracking)

|                       |            |
|:----------------------|:-----------|
| **Document status**   | PROD       |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- Odometry, also known as absolute position tracking, is a system that continuously estimates the robot's position on the field during autonomous operation.
- Without odometry, the robot can only move relative to its starting position using motor encoders and gyroscope data. This approach suffers from cumulative drift errors caused by wheel slip, momentum-induced overshooting, and other mechanical inconsistencies.
- By implementing odometry, the robot maintains a persistent awareness of its exact coordinates $(x, y, \theta)$ on the field plane, enabling precise point-to-point navigation and significantly improved autonomous consistency.

## ▲ Success metrics

|  **Goal**                                                                                                    |  **Metric**                                                                                                                                 |
|:-------------------------------------------------------------------------------------------------------------|:--------------------------------------------------------------------------------------------------------------------------------------------|
| The robot can accurately track its position on the field in real-time                                        | The odometry system continuously updates the robot's $(x, y, \theta)$ coordinates at a high refresh rate (minimum 50 Hz)                   |
| Position tracking remains accurate throughout autonomous despite wheel slip and external disturbances        | The system uses dedicated tracking wheels that are not affected by traction issues on the drivetrain motors                                |
| The odometry data integrates seamlessly with the existing Drive class for autonomous navigation              | The Drive class can access current position via getter functions and set target positions for point-to-point navigation                    |
| Position coordinates are easily configurable for different starting positions and field orientations         | Starting position can be set programmatically before autonomous begins using a simple function call                                         |
| The odometry implementation is mathematically sound and follows established best practices                   | Position calculations are based on proven differential drive odometry equations from robotics literature and competitive robotics resources |

## ▲ Implementation Assumptions

- The robot has a differential drive chassis (left and right motor sides operating independently)
- An inertial measurement unit (IMU) is installed and calibrated to provide accurate heading measurements
- Tracking wheels (either dedicated encoders or rotation sensors) are properly mounted with minimal play/backlash
- The tracking wheel positions and offsets from the robot's center are precisely measured and documented
- The robot operates on a flat, level surface (standard VEX competition field)

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**                   |  **Importance**    |  **Notes**                                                                                                               |
|:-----------------------------------|:-------------------|:-------------------------------------------------------------------------------------------------------------------------|
| Tracking Wheel Hardware            | HIGH               | Dedicated unpowered wheels with encoders/rotation sensors to measure displacement without drivetrain interference        |
| Inertial Measurement Unit (IMU)    | HIGH               | Provides accurate heading (theta) measurements essential for coordinate transformations                                  |
| Real-time Position Calculation     | HIGH               | Odometry calculations must run continuously in a background task to maintain up-to-date position data                    |
| Position Data Structure            | MEDIUM             | A structured data type (pose struct) to store and access x, y, and theta coordinates cleanly                             |
| Coordinate System Configuration    | MEDIUM             | Ability to set starting position and orientation for different autonomous routines                                       |
| Integration with PID Controllers   | HIGH               | Odometry data must interface with existing PID systems to enable point-to-point autonomous navigation                    |

## ▲ Design Overview Diagram

[Note to LLM: do not create the actual image. This will be handled separately. Instead, simply put instructions on how to create the diagram.]

**Diagram: Odometry System Block Diagram**

Create a block diagram showing:
1. **Inputs block** containing:
   - Left Tracking Wheel Encoder
   - Right Tracking Wheel Encoder  
   - Horizontal Tracking Wheel Encoder (optional)
   - IMU (Inertial Measurement Unit)

2. **Processing block** (center) containing:
   - "Odometry Calculation Task" (running continuously)
   - Mathematical formulas showing: $\Delta s_L$, $\Delta s_R$, $\Delta \theta$ → $(x, y, \theta)$

3. **Outputs block** containing:
   - Current Position $(x, y, \theta)$
   - Position Data Structure (pose)

4. **Consumer block** containing:
   - PID Controllers
   - Point-to-Point Navigation
   - Autonomous Movement Functions

Use arrows to show data flow from inputs → processing → outputs → consumers.
Use different colors for hardware components (blue), software components (green), and data structures (orange).

## ◎ Out of Scope

- This document focuses on **encoder-based odometry** using tracking wheels and the inertial sensor. Alternative localization methods such as vision-based positioning, GPS-like systems, or distance sensor triangulation are not covered here.
- Advanced filtering techniques (Kalman filters, particle filters) for sensor fusion are not implemented in this basic odometry system.
- Three-dimensional (3D) position tracking. This system only tracks the robot's planar position on a 2D field surface.
