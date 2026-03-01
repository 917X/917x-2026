# Motion Chaining PRD

|                       |            |
|:----------------------|:-----------|
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ Objective

- During autonomous, the robot executes a sequence of distinct motions - drives, turns, and swings - one after another.
- By default, each motion fully decelerates to a stop before the next command is issued, which wastes time and creates jerky, unnatural movement between segments.
- Motion chaining is a technique that exits a PID motion early while the robot still carries momentum, allowing it to flow smoothly and continuously into the next motion without a full stop.
- This reduces total autonomous cycle time and produces cleaner, faster multi-motion sequences.

## ▲ Success Metrics

|  **Goal**                                                                                          |  **Metric**                                                                                                                        |
|:---------------------------------------------------------------------------------------------------|:-----------------------------------------------------------------------------------------------------------------------------------|
| Consecutive autonomous motions transition without a full stop between them                         | The robot visually carries velocity from one motion into the next, with no observable deceleration to zero between chained motions |
| Chaining reduces total autonomous time compared to waiting for full settle between every motion    | Sequences using `pid_wait_quick_chain()` complete measurably faster than equivalent sequences using `pid_wait_quick()`             |
| Chain constant is independently configurable for each motion type (drive, turn, swing)             | Separate chain constants exist for drive, turn, and swing, each tunable without affecting the others                              |
| Chain behavior can be overridden on a per-call basis                                               | An optional override parameter in `pid_wait_quick_chain()` allows per-call chain constant values                                  |

## ▲ Implementation Assumptions

- PID exit conditions and chain constants are correctly tuned such that early exit does not result in large positional errors at the end of a chain
- Motion commands are issued immediately after `pid_wait_quick_chain()` returns; delays between calls negate the momentum benefit

## ▲ Roadmap

[Note to LLM: IGNORE. This section will be handled separately.]

## ▲ Requirements

|  **Requirement**                            |  **Importance**    |  **Notes**                                                                                                   |
|:--------------------------------------------|:-------------------|:-------------------------------------------------------------------------------------------------------------|
| Early-exit wait function                    | HIGH               | A wait variant that exits before full settle, preserving momentum for the next motion                        |
| Per-type chain constant configuration       | HIGH               | Drive, turn, and swing each need their own chain constant so each motion type can be tuned independently     |
| Per-call chain constant override            | MEDIUM             | Individual calls should be able to override the global constant for one-off situations                       |
| Directional chain constants (fwd/rev)       | LOW                | Drive and swing should support separate forward and backward chain constants for asymmetric deceleration      |

## ▲ Design Overview Diagram

**Diagram Instructions:**
Create a side-by-side comparison diagram with two rows:

**Row 1 — Without chaining (`pid_wait_quick()`):**
`Motion A` → `[Decelerate → Full Stop]` → `Motion B` → `[Decelerate → Full Stop]` → `Motion C`

**Row 2 — With chaining (`pid_wait_quick_chain()`):**
`Motion A` → `[Early Exit, still moving]` → `Motion B` → `[Early Exit, still moving]` → `Motion C`

Use a velocity curve (smooth ramp-up, ramp-down) under each motion segment to illustrate the speed profile. In Row 1 the curve drops to zero between motions; in Row 2 the curve stays elevated across the transition. Label the early-exit point as "chain exit" with an arrow.

## ◎ Out of Scope

- This feature only applies to `pid_drive_set`, `pid_turn_set`, and `pid_swing_set` motions. Odometry point-to-point and pure pursuit motions use separate sequencing mechanisms.
- Motion chaining does not replace PID tuning - poorly tuned controllers will still accumulate positional error regardless of chaining.
