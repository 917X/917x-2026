# Slew Rate Limiter PRD

|                       |            |
|:----------------------|:-----------|
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- When a PID motion begins from rest, the full commanded output is applied immediately. This causes the motors to snap to high voltage, producing sudden jerks that can tip the robot, cause wheel slip, and reduce positioning accuracy.
- A slew rate limiter addresses this by enforcing a maximum acceleration ramp at the start of every autonomous motion. Motor output starts at a configured minimum speed and increases linearly to the target maximum over a set distance, preventing instantaneous jumps.
- This applies independently to drive, turn, and swing motions, each with their own tunable ramp distance and starting speed.

## ▲ Success Metrics

|  **Goal**                                                                                               |  **Metric**                                                                                               |
|:--------------------------------------------------------------------------------------------------------|:----------------------------------------------------------------------------------------------------------|
| Motor output ramps up smoothly at the start of each autonomous motion rather than jumping to full power | Observed speed profile at motion start is a linear ramp, not a step                                       |
| Slew is independently configurable per motion type and direction                                        | Separate constants exist for forward drive, backward drive, turn, and swing                               |
| Slew disables automatically once the ramp distance is traveled                                          | After the configured ramp distance, the full PID output is no longer constrained by the limiter           |
| Slew is automatically skipped when max speed is below the minimum                                       | When `max_speed < min_speed`, slew is disabled for that motion to avoid a nonsensical ramp                |

## ▲ Implementation Assumptions

- Slew constants are tuned such that the ramp distance is short enough to not meaningfully delay the motion, while still preventing wheel slip
- The control loop runs at a consistent tick rate so the distance-based ramp advances predictably each cycle

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**                            |  **Importance**    |  **Notes**                                                                                  |
|:--------------------------------------------|:-------------------|:--------------------------------------------------------------------------------------------|
| Linear speed ramp from min to max speed     | HIGH               | Core ramp behavior — output increases proportionally with distance traveled                 |
| Per-motion-type configurable constants      | HIGH               | Drive (fwd/rev), turn, and swing each need independent distance and min_speed settings      |
| Automatic disable once ramp is complete     | HIGH               | Slew must stop constraining output once the robot has traveled the ramp distance            |
| Automatic disable when max speed < min speed| MEDIUM             | Guard against invalid configurations that would produce a downward or nonsensical ramp      |

## ▲ Design Overview Diagram

**Diagram Instructions:**
Create a speed-vs-distance plot with two curves:

- **Without slew (red dashed line):** A horizontal line at max speed starting from distance 0 — output immediately at full power.
- **With slew (blue solid line):** A line that starts at `min_speed` at distance 0 and increases linearly to `max_speed` at `distance_to_travel`. After that point, it becomes a flat horizontal line at `max_speed`.

Label the x-axis "Distance Traveled" and the y-axis "Motor Output". Mark `distance_to_travel` on the x-axis with a vertical dotted line, and label the two y-intercept points `min_speed` and `max_speed`.

## ◎ Out of Scope

- Slew rate limiting only applies to the start of a motion. End-of-motion deceleration is handled entirely by the PID controller's proportional term, not the slew limiter.
- Odometry point-to-point movements use slew but apply it through the same drive slew constants — no separate odom-specific configuration is needed.
