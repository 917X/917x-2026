# Driver Control PRD

|                       |            |
|:----------------------|:-----------|
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- Driver control functionality translates human controller inputs into precise drivetrain outputs, enabling the driver to maneuver the robot effectively during the 1:45 driver control period.
- This system must provide intuitive, responsive control that allows drivers to execute complex maneuvers while maintaining fine control at low speeds and adequate power at high speeds.

## ▲ Success metrics

|  **Goal**                                                                                                          |  **Metric**                                                                                                                                     |
|:-------------------------------------------------------------------------------------------------------------------|:------------------------------------------------------------------------------------------------------------------------------------------------|
| The drivetrain responds accurately to controller joystick inputs                                                   | The robot moves forward/backward and turns in direct correspondence to joystick positions with minimal latency                                  |
| The driver can precisely control the robot at both low and high speeds                                             | A customizable exponential curve function allows fine control at low joystick values while maintaining full power at high values               |
| The control scheme is intuitive and matches driver preferences                                                     | Multiple control modes (arcade split, arcade single, tank) are available, with arcade split as the default                                     |
| The robot maintains position when joysticks are released                                                           | An active brake PID controller holds the chassis in place when no input is detected, preventing drift                                           |
| Joystick drift and deadzone are properly handled                                                                   | A configurable joystick threshold eliminates unintended movement from controller drift                                                          |
| Drivers can tune control responsiveness without code changes                                                       | Real-time curve adjustment via controller buttons allows drivers to modify sensitivity on-the-fly, with values persisted to SD card            |

## ▲ Implementation Assumptions

- The `Drive` chassis class has been successfully instantiated with motors and sensors
- A PROS controller object (`master`) is properly initialized and connected
- The robot's physical drivetrain operates as a differential drive (left/right motor groups)
- An SD card is available for storing persistent curve values (optional but recommended)

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**                     |  **Importance**    |  **Notes**                                                                              |
|:-------------------------------------|:-------------------|:----------------------------------------------------------------------------------------|
| Joystick Input Mapping               | HIGH               | Convert controller analog inputs to motor voltages                                      |
| Curve Function Implementation        | HIGH               | Apply exponential scaling for improved low-speed control                                |
| Control Mode Selection               | MEDIUM             | Support arcade (split/single) and tank drive modes                                      |
| Active Brake System                  | HIGH               | Prevent drift when joysticks are released                                               |
| Deadzone Handling                    | HIGH               | Filter out joystick noise and controller drift                                          |
| Real-time Curve Adjustment           | LOW                | Optional feature for on-the-fly sensitivity tuning                                      |
| SD Card Persistence                  | LOW                | Save custom curve values between power cycles                                           |

## ▲ Design Overview Diagram

**Diagram Instructions:**
Create a flowchart showing the driver control signal flow:
1. Start with "Controller Joystick Input" at the top
2. Arrow down to "Joystick Threshold Filter (Deadzone)"
3. Arrow down to "Curve Function Application (Exponential Scaling)"
4. Decision diamond: "Joysticks Active?"
   - Yes path: Arrow to "Drive Mode Calculation" (tank vs arcade)
   - No path: Arrow to "Active Brake PID"
5. Both paths converge at "Motor Output (Left/Right Voltages)"
6. End at "Physical Drivetrain Motors"

Use different colors to distinguish the two paths (active control vs active brake).

## ▲ Control Mode Decision Matrix

|  **Control Mode**    |  **Forward/Reverse Control**  |  **Turn Control**         |  **Advantages**                                          |  **Disadvantages**                                      |
|:---------------------|:------------------------------|:--------------------------|:---------------------------------------------------------|:--------------------------------------------------------|
| Tank Drive           | Left stick → Left motors      | Left stick forward        | Highly precise control of each side independently        | Unintuitive for new drivers, requires practice          |
|                      | Right stick → Right motors    | Right stick backward      | Natural for experienced drivers                          | Difficult to drive straight without practice            |
| Arcade (Split)       | Left stick Y → Forward/Rev    | Right stick X → Turning   | **Intuitive separation of translation and rotation**     | Requires two hands for most movements                   |
|                      |                               |                           | **Easy to drive straight**                               |                                                         |
|                      |                               |                           | **Best for precise positioning**                         |                                                         |
| Arcade (Single)      | Left stick Y → Forward/Rev    | Left stick X → Turning    | One-handed operation possible                            | Less precise control, harder to execute complex paths   |
|                      |                               |                           | Comfortable for casual use                               | Forward and turn inputs can interfere                   |

**Our Choice: Arcade (Split)**

We selected arcade split control because it provides the most intuitive separation between forward/backward motion (left stick) and turning (right stick). This makes it significantly easier for drivers to execute precise positioning maneuvers, drive in straight lines, and perform smooth arcs. The two-handed requirement is acceptable given that most competition driving requires both hands anyway for managing other robot systems on the controller's shoulder buttons.

## ◎ Out of Scope

- Autonomous control functionality (covered in separate autonomous documentation)
- PID controller implementation details (covered in PID Controller documentation)
- Field-centric or heading-locked control modes
- Ramping/slew rate limiting during driver control (only used in autonomous)
- Controller vibration feedback for driver alerts
