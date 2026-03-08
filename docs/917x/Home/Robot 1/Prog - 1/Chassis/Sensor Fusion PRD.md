# Sensor Fusion State Estimation PRD

|                       |            |
|:----------------------|:-----------|
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- The encoder odometry system (documented separately) is the robot's baseline localization mechanism. It integrates tracking wheel displacement and IMU heading to maintain a continuous $(x, y, \theta)$ estimate in real time.
- However, encoder odometry is fundamentally **accumulative**: every small measurement error is permanently integrated into the position. Events like wheel slip, carpet drag, or contact from other robots silently corrupt the estimate. The system has no mechanism for self-correction — it cannot know that it is wrong.
- This document covers two supplementary localization systems that extend the encoder odometry with additional sensing. Together with the existing encoder odometry, they form a **three-layer state estimation stack** — the first multi-sensor position fusion of this kind implemented in competitive VEX V5 robotics:

|  **Layer**  |  **System**  |  **Sensor Input**  |  **What It Addresses**  |
|:------------|:-------------|:-------------------|:------------------------|
| 1 | Encoder Odometry (existing) | Tracking wheels, IMU heading | Baseline continuous pose tracking |
| 2 | IMU Acceleration Fusion | IMU accelerometer | Wheel slip and transient dynamics |
| 3 | Ray-Casting Localization (RCL) | Four distance sensors | Long-term absolute position drift |

- Each layer addresses failure modes that the others cannot cover. The two new systems are purely additive: neither modifies the encoder odometry, and both are completely inert if not instantiated or not started.

## ▲ Success Metrics

|  **Goal**  |  **Metric**  |
|:-----------|:-------------|
| Absolute position error remains bounded over a full 15-second autonomous run | RCL-corrected position error stays below 2 inches regardless of drift accumulated by the wheel encoders |
| The system detects and responds to wheel slip events that encoder odometry cannot observe | IMU fusion produces an updated fused position within one loop period (~20 ms) of a slip event |
| Absolute corrections do not interfere with active PID chassis motions | The RCL sync rate cap (3 in/s default) is transparent to active motion controllers and causes no step discontinuities |
| All three systems run without blocking autonomous or opcontrol | Each system runs in its own dedicated PROS background task; zero CPU time is taken from the main program loop |
| Invalid sensor readings are silently rejected | Corrupted distance readings (low confidence, obstructed path, off-cardinal angle) are discarded without affecting the position estimate |
| The combined system requires no ongoing manual calibration during a match | All systems initialize from the encoder odometry state and self-maintain throughout autonomous and driver control |

## ▲ Implementation Assumptions

- Four VEX V5 Distance Sensors are installed on the robot, one facing each cardinal direction (forward, right, backward, left). Complete directional coverage ensures the RCL system can always anchor at least one field axis regardless of robot orientation.
- The VEX V5 IMU is installed and calibrated. Its accelerometer output is an additional data channel on the same physical device already required by the chassis library for heading.
- The robot operates on a standard VRC competition field with interior dimensions of ±70.5 inches from center on both axes.
- Encoder odometry is fully initialized and the IMU has completed calibration before either supplementary system is started.

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**  |  **Importance**  |  **Notes**  |
|:------------------|:-----------------|:------------|
| Four VEX V5 Distance Sensors | HIGH | One per side; enables wall distance measurement on both field axes from any robot position and orientation |
| VEX V5 IMU (existing) | HIGH | Accelerometer (IMU Fusion) and gyroscope heading (both systems) are outputs of the same existing sensor; no new hardware needed |
| IMU Fusion background task at 50 Hz | HIGH | `fusion_loop()` must run continuously to track dynamic acceleration; read-on-demand polling is insufficient |
| RCL main update task at 25 Hz | HIGH | `main_update()` queries all sensors and updates the absolute position fix on a continuous cycle |
| RCL sync task at 25 Hz | HIGH | `sync_update()` applies the RCL correction into the chassis odometry at a controlled, rate-limited pace |
| Differential dead-reckoning state | HIGH | `latest_precise` + `pose_at_latest` pair enables continuous position tracking between absolute sensor readings |
| Cardinal direction heading gate | MEDIUM | Sensor readings are accepted only when the ray is within ±15° of a cardinal direction; prevents heading-error-amplified noise |
| Distance confidence threshold | MEDIUM | Readings above 200 mm are rejected if sensor confidence is below 40/63; filters far-field noise |
| Rate-limited auto-sync | MEDIUM | Maximum sync rate of 3 in/s prevents sudden corrections from disrupting active PID chassis controllers |
| Accumulation mode | LOW | Optional multi-cycle sensor averaging API enables higher-precision position resets when the robot is stationary at the start of autonomous |

## ▲ System Architecture Diagram

[Note: Do not create the actual image. Use the instructions below to create it separately.]

**Diagram: Three-Layer Localization Architecture**

Create a top-to-bottom layered layout with three horizontal bands and a shared feedback path:

**Layer 1 — Encoder Odometry (grey background, labeled "Existing System"):**
- Inputs on the left: "Left Tracking Wheel (ΔL)" and "Right Tracking Wheel (ΔR)" feeding into a process box labeled "Pose Integration (~100 Hz)"
- Input from right: "IMU Heading (θ)" also feeding into the process box
- Output arrow to the right labeled "`odom_pose (x, y, θ)`"

**Layer 2 — IMU Acceleration Fusion (blue background, labeled "New — ImuFusion"):**
- Input on the left: "IMU Accelerometer (ax, ay)" — note it comes from the same IMU as Layer 1 with a shared hardware icon
- A horizontal chain of process boxes connected by arrows: "Body→Field Rotation" → "High-Pass Filter" → "Velocity Decay Integration" → "Complementary Blend"
- Dotted arrow from Layer 1's `odom_pose` flowing down into the "Complementary Blend" box, labeled "odom anchor (1−α)"
- Output arrow on the right labeled "`fused_x, fused_y` (read-only)"

**Layer 3 — Ray-Casting Localization (green background, labeled "New — RclTracking"):**
- Inputs on the left: four small sensor icons labeled "Front Dist", "Right Dist", "Back Dist", "Left Dist"
- A horizontal chain: "Ray Validity Check" → "Ray–Wall Intersection" → "Coordinate Recovery" → "Differential Bridge"
- Dotted arrow from Layer 1's `odom_pose` flowing down into the "Differential Bridge" box, labeled "odom delta baseline"
- Output arrow on the right labeled "`rcl_pose (x, y)`"
- A curved feedback arrow from the `rcl_pose` output looping back and connecting into Layer 1's pose input, labeled "Gradual sync ≤ 3 in/s"

**Layout notes:**
- Use consistent hardware icons (rounded rectangles) for sensors and distinct box styles (sharp rectangles) for software processes
- The curved sync feedback arrow should clearly show that RCL gradually pushes corrections into the encoder odometry
- Color code: grey = existing, blue = IMU Fusion, green = RCL

## ◎ Out of Scope

- **Heading ($\theta$) fusion**: Both new systems operate exclusively on position $(x, y)$. Heading is handled with high accuracy by the dedicated gyroscope already inside the VEX V5 IMU, and no additional sensor input is available to improve it further.
- **Kalman filtering**: A formal Kalman filter is not implemented. The complementary filter used in IMU Fusion is a standard engineering approximation that achieves similar behavior to an optimal single-state Kalman filter under certain noise assumptions, while being far simpler to implement and tune on embedded hardware.
- **Obstacle detection or dynamic field awareness**: The RCL system includes sensor quality filtering but does not model or track dynamic field objects.
- **Three-dimensional position tracking**: All systems operate exclusively in the 2D field plane.
