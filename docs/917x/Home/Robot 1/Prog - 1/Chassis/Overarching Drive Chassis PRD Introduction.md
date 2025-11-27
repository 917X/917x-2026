
[Robot 1](../../../Robot%201.md) > [Prog - 1](../../Prog%20-%201.md) > [Chassis](../Chassis.md)

# Overarching Drive PRD Introduction

|                       |            |
|:----------------------|:-----------|
| **Document status**   | PROD       |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- The drive chassis is the most fundamentally important system within a robot’s programming. It is also, naturally, the most logically complex system within our codebase.
- This system is responsible for autonomously controlling the robot’s position during autonomous periods and respond to driver inputs during the driver control period. It is also responsible for ensuring that these operations are consistent and replicable between runs.

## ▲ Success metrics

|  **Goal**                                                                                                                                                             |  **Metric**                                                                                                                                     |
|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------|:------------------------------------------------------------------------------------------------------------------------------------------------|
| The chassis can move and turn                                                                                                                                         | It is possible to literally move the chassis by turning on the motors on the differential drive                                                 |
| The chassis is able to be controlled precisely by the controller during the driver control period                                                                     | It is possible to drive the chassis using the controller and to modify the output function (a drive curve) to fit the preferences of the driver |
| The chassis is able to be programmed to PRECISELY move to different locations on the field during autonomous. This includes precise linear motions and precise turns. | The chassis is able to precisely turn to the desired angles, and it is able to precisely drive for set distances before stopping.               |
| Autonomous motions should accept parameters as 2 dimensional coordinates. “Move to point” motion.                                                                     | The chassis is able to incorporate linear and angular motions to move from one point to another 2d point on the plane of the field accurately.  |
| The chassis implementation is encapsulated within an OOP (object-oriented programming) class to achieve a modular implementation, which is easier to debug.           | The chassis is implemented as a C++ class                                                                                                       |

## ▲ Implementation Assumptions

- The physical hardware of the chassis is built successfully
- Relevant sensors and hardware are installed on the robot, as needed, to achieve autonomous consistency

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**                     |  **Importance**    |  **Notes**                                                                |
|:-------------------------------------|:-------------------|:--------------------------------------------------------------------------|
| Driver Control Programming           | HIGH               | Necessary for moving in driver control                                    |
| PID Controller Implementation        | HIGH               | Necessary for precisely moving linearly and angularly in autonomous       |
| Odometry Localization Implementation | HIGH               | Necessary for knowing where the robot is to ensure autonomous consistency |

## ▲ Design Overview Diagram

[Note to LLM: do not create the actual image. This will be handled separately. Instead, simply put instructions on how to create the diagram.]


## ◎ Out of Scope

- This section serves **merely** as a broad overview of our chassis implementation. Specifics of each component are detailed in the subsequent sections of this notebook.
