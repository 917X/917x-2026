# RCL Tracking Implementation (Ray-Casting Localization)

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2026-03-07 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- Why encoder odometry accumulates unbounded error and needs an absolute correction source
- The geometry of ray-casting: how a distance reading to a known wall recovers an absolute coordinate
- Heading convention conversion and why the cardinal-direction gate is critical
- How sensor mounting offsets are handled in exact world-frame coordinates
- The differential dead-reckoning bridge: seamless tracking between absolute fixes
- Rate-limited auto-sync and why it is safe during active PID motions
- Accumulation mode for high-precision static resets
- Code walkthrough, parameter reference, and usage examples

## ▲ Executive Summary

### Purpose & Need

- Encoder odometry continuously integrates small wheel displacements to track position. Because every small error is permanently added to the running total, the position estimate drifts over time. After a full 15-second autonomous run — including wheel slip, carpet drag, and contact from other robots — the accumulated error can be several inches or more. The chassis PID controllers navigate based on this estimate; if it is wrong, every subsequent movement targets the wrong destination.
- RCL solves this by providing an **absolute** position fix that does not drift. The VRC field is a perfectly known enclosure: four flat walls at exactly ±70.5 inches from the center on both axes. If the robot knows its heading and can measure the distance to any one of those walls, it can compute exactly where it is on that axis — from scratch, without reference to any previous position.
- `RclTracking` runs distance sensors against this known geometry in a continuous background task. Each valid reading replaces the accumulated drift on its axis with a ground-truth measurement. Between readings, the encoder odometry dead-reckoning bridges the gap, so the position estimate never has to wait for the next sensor read. Over a full autonomous run, this combination keeps position error bounded to well under 2 inches regardless of how much drift the wheel encoders have accumulated.

### Credit

The RCL algorithm concept and architecture are adapted from [jiazegao/RCL-Tracking](https://github.com/jiazegao/RCL-Tracking), an open-source localization system originally written for LemLib. The implementation here is a complete re-implementation using the EZ-Template chassis interface.

## ▲ Architecture

### External Dependencies

|  **Component**  |  **Type**  |  **Purpose**  |
|:----------------|:-----------|:--------------|
| `pros::Distance` | Kernel Library | VEX V5 Distance sensor; provides distance in mm and a confidence value (0–63) |
| `pros::Task` | Kernel Library | Two background tasks: one for sensor reads and absolute fixes, one for gradual odom sync |
| `Drive::odom_pose_get()` / `odom_xyt_set()` | Chassis Library | Read and write the chassis encoder odometry pose |

### Internal Dependencies

|  **Component**  |  **Purpose**  |
|:----------------|:--------------|
| `RclSensor` | Wraps one distance sensor with its geometric mounting parameters; performs ray-casting and coordinate recovery |
| `RclTracking` | Manages the collection of sensors, the differential dead-reckoning state, and the sync pipeline |

### Interfaces

|  **Type**  |  **Name**  |
|:-----------|:-----------|
| Component Input | Distance sensor readings (mm), encoder odometry pose, current heading |
| Component Output | `get_rcl_pose()` — fused absolute pose; `update_odom_pose()` — writes the RCL fix into encoder odometry |

## ▲ Design Spec

### The Core Idea: Inverting Wall Distance into Absolute Position

Consider a robot somewhere inside the VRC field facing due east. The right-side distance sensor measures 24 inches to the east wall. That wall is at $x = +70.5$ inches from field center. Since the sensor is on the right side and the robot is facing east, the sensor is measuring eastward — so:

$$s_x = 70.5 - 24 = 46.5 \text{ in}$$

The sensor is at $x = 46.5$ inches. Subtract the sensor's distance from the robot's center (say, 5 inches to the right), and:

$$x_\text{robot} = 46.5 - 5 = 41.5 \text{ in}$$

This is the robot's absolute X coordinate — computed entirely from one distance measurement and known field geometry. No accumulated error. No reference to any previous position. This is the fundamental idea behind RCL.

The full implementation generalizes this to arbitrary robot headings, any four field walls, and all four mounting positions, with validity checks that discard readings that would give unreliable geometry.

### Coordinate System and Heading Convention

The field coordinate system follows the EZ-Template standard:
- **Origin**: center of the field
- **$+x$**: east (rightward)
- **$+y$**: north (forward when heading = 0°)
- **$\theta$**: robot heading in degrees, clockwise from north (0° = forward, 90° = faced east)
- **Field walls**: $x = \pm70.5\text{ in}$ (east/west), $y = \pm70.5\text{ in}$ (north/south)

Throughout the RCL code, angles alternate between two conventions that must be carefully tracked:

| **Convention** | **0° direction** | **Positive direction** | **Used for** |
|:---|:---|:---|:---|
| Compass (robot heading) | North ($+y$) | Clockwise | Robot heading θ, sensor `main_angle` |
| Standard trig | East ($+x$) | Counter-clockwise | `cos()` / `sin()` calls in C++ |

The conversion between them is: $\psi_\text{trig} = 90° - \theta_\text{compass}$.

### Step 1: Sensor Mounting Offset — Polar Form

Each sensor is not at the robot's tracking center; it is bolted somewhere on the robot's frame. The constructor takes Cartesian mounting offsets and converts them to polar form, which is more convenient for world-frame rotation:

```cpp
// horiz_offset: positive = sensor is to the right of center
// vert_offset:  positive = sensor is ahead of center
offset_dist_  = std::hypot(horiz_offset, vert_offset);
offset_angle_ = std::fmod(
    std::atan2(vert_offset, horiz_offset) * 180.0 / M_PI + 360.0, 360.0);
```

`offset_dist_` is the straight-line distance from the tracking center to the sensor. `offset_angle_` is the direction from center to sensor, measured **counter-clockwise from the robot's right ($+x$) axis**. For a purely right-side sensor (`horiz=5, vert=0`): $\text{offset\_angle} = \text{atan2}(0, 5) = 0°$. For a purely forward sensor (`horiz=0, vert=5`): $\text{offset\_angle} = \text{atan2}(5, 0) = 90°$.

[Note: Insert diagram here — **"Sensor Mounting Offset Polar Form"**. Draw a top-down view of a rectangular robot. Place the tracking center in the middle of the robot (small crosshair). Place four sensor icons: one on the right side (labeled "horiz=+5, vert=0"), one on the front (labeled "horiz=0, vert=+4"), one on the back-right (labeled "horiz=+3, vert=−3"). For each sensor, draw a line from the tracking center to the sensor icon and label it `offset_dist = hypot(...)`. Label the angle of each line from the robot's +X (right) axis as `offset_angle`, measured counter-clockwise. Annotate the formula for each: e.g., right sensor: `offset_dist=5, offset_angle=0°`; front sensor: `offset_dist=4, offset_angle=90°`. Use dashed lines for the robot's body-frame +X and +Y axes.]

### Step 2: Computing the Sensor's World Position

When the robot is at field position $(x, y)$ with heading $\theta$, the mounting offset vector must be rotated from body frame into field-frame coordinates. Using the EZ-Template heading convention (0° = north, CW+), the robot's body-frame unit vectors in field coordinates are:

$$\hat{e}_\text{right} = \begin{pmatrix}\cos\theta \\ -\sin\theta\end{pmatrix} \qquad \hat{e}_\text{forward} = \begin{pmatrix}\sin\theta \\ \cos\theta\end{pmatrix}$$

**Verification:** At $\theta = 0°$ (facing north): $\hat{e}_\text{right} = (1, 0) = \text{east}$ ✓, $\hat{e}_\text{forward} = (0, 1) = \text{north}$ ✓.

The offset vector in body frame is $(\cos\alpha \cdot d,\; \sin\alpha \cdot d)$ where $\alpha$ = `offset_angle_` and $d$ = `offset_dist_`. Decomposing onto the field-frame axes:

$$s_x = x + d\cos\alpha \cdot \cos\theta + d\sin\alpha \cdot \sin\theta = x + d\cos(\alpha - \theta)$$

$$s_y = y + d\cos\alpha \cdot (-\sin\theta) + d\sin\alpha \cdot \cos\theta = y + d\sin(\alpha - \theta)$$

This is the identity: $\cos A \cos B + \sin A \sin B = \cos(A-B)$ and $-\cos A \sin B + \sin A \cos B = \sin(A-B)$.

The code computes exactly this:

```cpp
double theta = deg_to_rad(offset_angle_ - bot_pose.theta);  // α − θ
sp_.x = bot_pose.x + std::cos(theta) * offset_dist_;
sp_.y = bot_pose.y + std::sin(theta) * offset_dist_;
```

The sensor's ray heading in the world frame is handled separately. If the sensor faces direction `main_angle_` relative to robot forward (0 = forward, 90 = right), and the robot's heading is $\theta$, then the ray heading in the compass convention is:

$$h_\text{ray} = \theta + \text{main\_angle} \pmod{360°}$$

```cpp
sp_.heading = norm_heading(bot_pose.theta + main_angle_);
```

[Note: Insert diagram here — **"Sensor World Position Computation"**. Draw a top-down VRC field with a robot at roughly (30 in, 20 in), heading ~30° clockwise from north. Draw the robot as a rectangle. Mark the tracking center. Draw the body-frame axes (+X right, +Y forward) in dashed lines rotated at the robot's heading. Place a sensor on the robot's right side. Draw the offset vector from the tracking center to the sensor in the body frame. Then draw the rotated offset vector in the field frame (full lines), showing it has been rotated by θ. Label the resulting sensor world position as $(s_x, s_y)$. Show the angle $\alpha - \theta$ between the world +X axis and the offset vector in world frame. Include the formula $s_x = x + d\cos(\alpha - \theta)$.]

### Step 3: Reading Validity — Three Gates

Before a distance reading is used for position correction, it passes three sequential checks:

**Gate 1 — Range check:**
```cpp
if (dist_mm > RCL_MAX_DIST_MM || dist_mm <= 0.0) return false;
```
Readings above 2000 mm (≈79 in, slightly longer than a field half-diagonal) are physically impossible on a standard field and indicate sensor error or open space beyond the wall. Readings of 0 are sensor failures.

**Gate 2 — Confidence check:**
```cpp
if (dist_mm > 200.0 && sensor_->get_confidence() < RCL_MIN_CONF) return false;
```
VEX V5 Distance sensors report a confidence value (0–63). At short range (≤200 mm) the sensor is highly reliable regardless. At longer range, low confidence indicates the beam hit a non-ideal surface (narrow object, angled wall edge, game element) and the reading should be discarded.

**Gate 3 — Cardinal direction gate (the most important one):**
```cpp
double heading_mod = std::fmod(sp_.heading, 90.0);
if (heading_mod > angle_tol_ && heading_mod < (90.0 - angle_tol_)) return false;
```
This is the critical validity check. To understand why it matters, consider what happens when the ray hits a wall at an angle.

**Why oblique angles are dangerous:** The coordinate recovery formula (Step 4 below) projects the measured distance onto one field axis using $\cos_a$ or $\sin_a$. Near a cardinal direction, $\cos_a$ or $\sin_a$ is close to 1.0, and the projection is nearly the raw distance — a 1% heading error causes a ~1% coordinate error. But near 45°, both $\cos_a$ and $\sin_a$ are $\approx 0.707$, and a small heading error causes a much larger error in the computed coordinate. The worst case is that both axes are contaminated simultaneously, giving a useless result.

Mathematically: the coordinate recovery error due to a small heading error $\delta\theta$ is proportional to $\frac{d}{d\theta}(\cos\theta) = -\sin\theta$. Near $\theta = 0°$ (cardinal), $\sin(0°) = 0$ — the error is zero to first order. Near $\theta = 45°$, $\sin(45°) = 0.707$ — errors are amplified by a factor of 0.707 times the distance.

The gate accepts readings only when the ray heading, modulo 90°, is within `angle_tol_` (default 15°) of a grid-aligned direction. The visual pattern is: reading accepted when sensor points roughly N/E/S/W; rejected when pointing diagonally.

[Note: Insert diagram here — **"Cardinal Direction Gate"**. Draw a circle representing all possible ray headings (0°–360°). Divide it into eight 45° sectors. Mark the four cardinal directions (N, E, S, W) with thick radial lines. Shade the region within ±15° of each cardinal direction green (labeled "ACCEPTED"). Shade the region outside these bands — approximately the 45°/135°/225°/315° diagonal sectors — red (labeled "REJECTED"). Label the green arcs "heading_mod ≤ 15° or ≥ 75°" and describe the arrow of a sensor ray hitting a wall at the boundary, showing that near-perpendicular rays (green) give stable coordinate recovery while oblique rays (red) amplify heading errors.]

### Step 4: Ray–Wall Intersection

After confirming validity, the sensor's ray is cast toward the four walls. The ray is parameterized as:

$$P(t) = (s_x + t \cdot \cos_a, \quad s_y + t \cdot \sin_a), \quad t \geq 0$$

where $(\cos_a, \sin_a) = (\cos(90° - h), \sin(90° - h)) = (\sin h, \cos h)$ converts the compass heading $h$ to a standard-trig unit direction vector.

**Verification:** For $h = 0°$ (facing north): $\cos_a = \sin(0°) = 0$, $\sin_a = \cos(0°) = 1$. Direction vector = $(0, 1)$ = north. ✓

Each wall is a line in one coordinate. The intersection parameter $t$ is found by solving:

| **Wall** | **Equation** | **Intersection $t$** | **Valid when** |
|:---------|:-------------|:---------------------|:---------------|
| North ($y = +70.5$) | $s_y + t \cdot \sin_a = 70.5$ | $t_N = (70.5 - s_y) / \sin_a$ | $t > 0$, $|\sin_a| > 10^{-6}$ |
| East ($x = +70.5$) | $s_x + t \cdot \cos_a = 70.5$ | $t_E = (70.5 - s_x) / \cos_a$ | $t > 0$, $|\cos_a| > 10^{-6}$ |
| South ($y = -70.5$) | $s_y + t \cdot \sin_a = -70.5$ | $t_S = (-70.5 - s_y) / \sin_a$ | $t > 0$, $|\sin_a| > 10^{-6}$ |
| West ($x = -70.5$) | $s_x + t \cdot \cos_a = -70.5$ | $t_W = (-70.5 - s_x) / \cos_a$ | $t > 0$, $|\cos_a| > 10^{-6}$ |

All four $t$ values are computed and the **smallest positive $t$** wins — that is the nearest wall in the ray's direction. The near-zero divisor guard (`> 1e-6`) handles the case where the ray is exactly parallel to a pair of walls and would never reach them.

```cpp
double min_t = 1e9;
int wall = -1;  // 1=North, 2=East, 3=South, 4=West

if (std::abs(cos_a) > 1e-6) {
  double t_east = (RCL_FIELD_HALF - sp_.x) / cos_a;
  if (t_east > 0.0 && t_east < min_t) { min_t = t_east; wall = 2; }
  double t_west = (-RCL_FIELD_HALF - sp_.x) / cos_a;
  if (t_west > 0.0 && t_west < min_t) { min_t = t_west; wall = 4; }
}
// ... same for North and South with sin_a
```

[Note: Insert diagram here — **"Ray–Wall Intersection"**. Draw a to-scale top-down VRC field (±70.5 in square). Place a robot slightly off-center (e.g., at field position x=40, y=15) and draw four sensor rays from the sensor icon — one in each cardinal direction (N, E, S, W). For each ray, draw the intersection point at the corresponding wall. Label each intersection with the $t$ value formula. For a non-cardinal case (optional second illustration): show a ray at ~20° from north hitting the north wall, draw the perpendicular from the hit point to the north wall, and annotate the formula for $t_N$. Show the sensor's $s_x, s_y$ origin explicitly.]

### Step 5: Recovering the Absolute Coordinate

Once the wall that was hit is known, the sensor's absolute coordinate on the relevant axis is recovered by inverting the ray travel:

If the sensor hit the **East wall** ($x = 70.5$) after traveling distance `val` (in inches) along a ray with x-component $\cos_a$:

$$s_x = 70.5 - \cos_a \cdot \text{val}$$

The intuition: the sensor is `val × cos_a` inches west of the east wall, so $s_x = 70.5 - (\text{horizontal component of distance traveled})$.

The full set of formulas for all four walls:

$$\text{North: } s_y = 70.5 - \sin_a \cdot \text{val} \qquad \text{East: } s_x = 70.5 - \cos_a \cdot \text{val}$$

$$\text{South: } s_y = -70.5 - \sin_a \cdot \text{val} \qquad \text{West: } s_x = -70.5 - \cos_a \cdot \text{val}$$

**Verification (South wall):** Facing south, $h = 180°$, so $\sin_a = \cos(180°) = -1$. Robot at $y = 0$, sensor at $y \approx 0$, south wall at $y = -70.5$, measured distance = 70.5 in.
$s_y = -70.5 - (-1) \times 70.5 = -70.5 + 70.5 = 0$ ✓

```cpp
if      (wall == 1) { type = Y;  res =  RCL_FIELD_HALF - sin_a * val; }  // North
else if (wall == 2) { type = X;  res =  RCL_FIELD_HALF - cos_a * val; }  // East
else if (wall == 3) { type = Y;  res = -RCL_FIELD_HALF - sin_a * val; }  // South
else                { type = X;  res = -RCL_FIELD_HALF - cos_a * val; }  // West
```

The result is the **sensor's** absolute coordinate. Note that North and South hits recover $y$; East and West hits recover $x$. This is why an individual sensor can only constrain one axis at a time — the axis perpendicular to the wall it hit.

### Step 6: Offset Subtraction — Sensor Coordinate to Robot Coordinate

The final step converts the sensor's absolute coordinate to the robot center's coordinate. From Step 2, the sensor's world offset from robot center is $(d\cos(\alpha - \theta),\; d\sin(\alpha - \theta))$. Subtracting the relevant component:

$$x_\text{robot} = s_x - d\cos(\alpha - \theta) \qquad y_\text{robot} = s_y - d\sin(\alpha - \theta)$$

```cpp
double off_rad = deg_to_rad(offset_angle_ - bot_pose.theta);
if      (type == RclCoordType::X) res -= std::cos(off_rad) * offset_dist_;
else if (type == RclCoordType::Y) res -= std::sin(off_rad) * offset_dist_;
```

This uses the exact same $(\alpha - \theta)$ angle computed in Step 2: the offset vector in world frame projected onto the corrected axis.

### Complete `get_bot_coord()` Flow

```
1. update_pose()       → compute sensor world position (sx, sy) and ray heading
2. is_valid()          → range, confidence, and cardinal-direction checks
3. val = distance in inches
4. Compute (cos_a, sin_a) from compass heading via trig conversion
5. Find nearest wall by solving ray–wall intersection for t on all four walls
6. Recover sensor's absolute coord from: wall_position − component × val
7. Subtract mounting offset projection → robot center absolute coordinate
8. Return {RclCoordType::X or Y, coordinate}
```

[Note: Insert flowchart here — **"`get_bot_coord()` Decision Flow"**. Create a top-down flowchart: "update_pose()" → Diamond "is_valid()?" → NO branch → "return INVALID"; YES branch → "Compute cos_a, sin_a" → "Find nearest wall (min positive t)" → Diamond "wall found?" → NO → "return INVALID"; YES → "Recover sensor coord (W − component × val)" → "Subtract mounting offset" → "Return {type, robot_coord}". Use rounded rectangles for process steps and sharp diamonds for decisions.]

### Step 7: Differential Dead-Reckoning Bridge

Distance sensor readings are intermittent — they are only accepted when the cardinal-direction gate passes, which means they arrive only when the robot is heading in a roughly axis-aligned direction. Between valid readings, the position estimate must still be available and must incorporate the robot's ongoing movement.

The RCL system solves this through a **differential dead-reckoning bridge**:

$$\text{rcl\_pose}(t) = p_\text{precise} + \bigl(\text{odom}(t) - \text{odom}(t_\text{fix})\bigr)$$

Where:
- $p_\text{precise}$ = `latest_precise_`: the most recent absolute fix from a distance sensor (inches)
- $\text{odom}(t_\text{fix})$ = `pose_at_latest_`: the encoder odometry value at the moment that fix was taken
- $\text{odom}(t)$: the encoder odometry value right now

The delta $(\text{odom}(t) - \text{odom}(t_\text{fix}))$ is the encoder-measured displacement since the last absolute fix. This displacement is added on top of the last known good absolute position. The reasoning:

- The absolute fix is accurate at the moment it is computed — it is derived from field walls, not accumulated error.
- Any drift that happened *before* the fix is corrected by the fix.
- Any drift that happens *after* the fix accumulates again in the delta term, but only since the last fix (typically a fraction of a second), so it is very small.
- When the next absolute fix arrives, the delta resets to zero and the accumulated post-fix drift is discarded.

```cpp
ez::pose RclTracking::get_rcl_pose() const {
  ez::pose odom = chassis_->odom_pose_get();
  return {
    latest_precise_.x + (odom.x - pose_at_latest_.x),
    latest_precise_.y + (odom.y - pose_at_latest_.y),
    odom.theta   // heading always from IMU (already accurate)
  };
}
```

[Note: Insert diagram here — **"Differential Dead-Reckoning Bridge"**. Draw a timeline (x-axis = time, y-axis = position on one axis). Draw two lines: a dashed line for "Encoder Odom" which slowly drifts away from true position, and a solid line for "RCL Pose". At regular intervals, draw vertical "anchor" marks labeled "Absolute Fix from sensor". At each anchor: the RCL pose snaps back to the true value (recovered from wall). Between anchors: the RCL pose follows the odom delta (parallel to odom, same shape, but offset by the fix correction). Show visually that the gap between RCL and true position is much smaller than the gap between raw odom and true position.]

### Step 8: Validating and Applying New Absolute Fixes (`main_update()`)

Each cycle of the main background task:

1. **Accumulation phase** (optional): If `accumulating_` is true, the loop collects raw sensor reads every `goal_mspt_` ms, building running totals. When accumulation ends, divides totals by counts to get averaged readings. This is the high-precision static reset mode.

2. **Failsafe check**: If the RCL pose has drifted more than `max_delta_from_odom_` (default 10 in) from the encoder odom, something has gone wrong with the absolute fix — perhaps the robot was in front of a game element that absorbed the sensor rays, producing phantom wall distances. The failsafe nudges `latest_precise_` 1 inch toward the odom estimate per cycle, gently walking the RCL state back toward a sane range without snapping it abruptly.

3. **Sensor queries**: Each sensor's `get_bot_coord()` is called. Results passing all three validity checks are collected into separate vectors of candidate X and Y coordinates. Additionally, each result must pass a **plausibility check**: the recovered coordinate must differ from the current RCL estimate by at most `max_delta_` (default 4 in). This rejects readings where a game object is sitting directly between the sensor and the wall.

4. **Averaging and acceptance**: All candidate X values are averaged; same for Y. The mean is accepted as the new absolute fix only if:
   - The mean coordinate is inside field bounds (within ±70.5 in)
   - The correction magnitude $|mean - rcl_x|$ is at least `min_delta_` (default 0.5 in) — to avoid constantly overwriting a good estimate with sensor noise

```cpp
if (!xs.empty()) {
  double mean_x = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
  if (mean_x > -RCL_FIELD_HALF && mean_x < RCL_FIELD_HALF &&
      std::abs(mean_x - rcl_pose.x) >= min_delta_) {
    latest_precise_.x = mean_x;    // store new absolute fix
    pose_at_latest_.x = odom_now.x; // anchor the dead-reckoning delta
  }
}
```

When `latest_precise_` is updated, `pose_at_latest_` is set to the current odom value simultaneously. This is the key bookkeeping step: it resets the dead-reckoning delta to zero at the moment of correction, so the next call to `get_rcl_pose()` immediately reflects the new absolute fix.

### Step 9: Gradual Sync into Encoder Odometry (`sync_update()`)

The RCL estimate is worth nothing if it stays locked inside the `RclTracking` object and the chassis PID continues navigating by the drifted encoder odom. The sync task closes this gap, but must do so carefully.

**Why can't we just snap the odom to the RCL estimate instantly?** Active PID controllers are, at every moment, computing motor outputs based on `odom_x_get()`. Their derivative term (the "D" term) measures the rate of change of position. If the position jumps by 2 inches instantaneously, the derivative spikes, causing a large sudden motor output change — a jerk. This could destabilize an in-progress turn or straight drive.

Instead, `sync_update()` applies at most `max_sync_pt_` inches of correction per cycle (computed as `max_sync_per_sec / freq_hz`, e.g. $3.0 / 25 = 0.12$ in per 40 ms cycle). This translates to a maximum correction rate of 3 in/s — negligible as a perturbation relative to a robot traveling at 40+ in/s.

```cpp
if (diff <= max_sync_pt_) {
  // Discrepancy is small enough — close it fully in one step
  x_update = x_diff;
  y_update = y_diff;
} else {
  // Cap the update to max_sync_pt_ in the direction of the correction
  double inv = max_sync_pt_ / diff;
  x_update = x_diff * inv;
  y_update = y_diff * inv;
}
chassis_->odom_xyt_set(odom.x + x_update, odom.y + y_update, odom.theta);
```

**Maintaining dead-reckoning consistency after a sync:** When the odom pose is shifted by $(x\_\text{update}, y\_\text{update})$, the dead-reckoning formula `(odom_now - pose_at_latest)` would suddenly see a different value — `pose_at_latest_` is now further behind the new odom, making `get_rcl_pose()` jump. To prevent this, `pose_at_latest_` is shifted by the same amount as the odom pose was shifted:

```cpp
pose_at_latest_.x    += x_update;
pose_at_latest_.y    += y_update;
pose_at_latest_.theta = odom.theta;
```

This keeps `(odom.x - pose_at_latest_.x)` unchanged after the sync step, so `get_rcl_pose()` is perfectly continuous across every sync operation.

[Note: Insert diagram here — **"`sync_update()` Rate-Limiting"**. Draw a 2D field coordinate plane. Place two dots: "Current Odom Pose" and (offset ~3 in away) "RCL Pose (target)". Draw an arrow from Odom toward RCL labeled "sync step = min(diff, max_sync_pt_)". Then draw a small sequence at the bottom showing 5 time steps: Odom dot moves incrementally toward RCL dot each step, shrinking the gap each cycle. Label the final step where diff ≤ max_sync_pt_ and the gap closes fully. Add a note: "max correction = 3 in/s, safe for active PIDs".]

### Accumulation Mode — High-Precision Static Reset

For maximum accuracy at the start of an autonomous routine, the robot can stand still while the sensors collect multiple readings over a window of time and average them. This dramatically reduces the effect of single-sample sensor noise.

```cpp
void RclTracking::accumulate_for(int ms, bool auto_update_after) {
  start_accumulating(auto_update_after);  // sets accumulating_ = true
  uint32_t end = pros::millis() + ms;
  while (pros::millis() < end) { pros::delay(min_pause_ms_); }
  stop_accumulating();
  // main_update() picks up the averages and applies them on its next cycle
}
```

While `accumulating_` is true, `main_update()` enters an inner loop that collects `raw_reading()` from every sensor at every cycle, accumulating totals and counts. When `stop_accumulating()` is called, the loop exits, and the averaged values (`acc_total[i] / acc_count[i]`) are passed to `get_bot_coord()` in place of live readings.

## ▲ Parameter Reference

|  **Parameter**  |  **Default**  |  **Effect**  |
|:----------------|:--------------|:-------------|
| `freq_hz` | 25 | Background update frequency. Higher values increase CPU usage and reduce latency between fixes |
| `auto_sync` | true | Whether to automatically push RCL corrections into encoder odom. Disable for diagnostic or manual-sync workflows |
| `min_delta` | 0.5 in | Minimum correction accepted. Prevents constantly overwriting a good estimate with sub-noise-floor jitter |
| `max_delta` | 4.0 in | Maximum plausible single-cycle correction. Rejects readings obstructed by game elements on the field |
| `max_delta_from_odom` | 10.0 in | Failsafe divergence limit. If RCL and odom disagree by more than this, the failsafe nudge activates |
| `max_sync_per_sec` | 3.0 in/s | Rate cap for gradual odom sync. Keep well below robot travel speed (typical: 40–48 in/s) |
| `min_pause_ms` | 20 ms | Minimum sleep per task cycle. Prevents CPU starvation on other PROS tasks |

### `angle_tol` per sensor (default 15°)

The cardinal-direction tolerance is set per sensor at construction. 15° is a reasonable balance: it allows the sensor to be used while the robot is nearly aligned with a wall (as happens at the start of most autonomous routines), while still rejecting the geometrically unstable diagonal region. Setting `angle_tol` higher (e.g. 30°) accepts more readings but increases position noise.

## ▲ Implementation Overview

### File Structure

```txt
include/
└── subsystems/
    └── rcl_tracking.hpp     # RclSensor, RclTracking declarations, constants, types

src/
└── subsystems/
    └── rcl_tracking.cpp     # All method implementations, background loops
```

### Class Structure Overview

**`RclSensor`** — one instance per physical sensor, declared globally. Auto-registers into a shared static vector on construction, so `RclTracking` finds all sensors automatically without any manual registration step.

**`RclTracking`** — one instance for the whole robot. Manages two background tasks: `main_loop()` (runs sensor reads and position fixes) and `sync_loop()` (applies rate-limited corrections to encoder odom).

### Usage

**Global declarations (devices.cpp):**
```cpp
// Sensor facing right — on the right side of the robot, 4.875 in from center
RclSensor rclRight(&rightDistance, 4.875, 0.0, 90.0);

// Sensor facing back — on the back of the robot, 4.53 in behind center
RclSensor rclBack(&backDistance, 0.0, -4.53, 180.0);

// RclTracking: 25 Hz, auto-sync on
RclTracking rcl(&chassis, 25, true);
```

**Startup (initialize()):**
```cpp
rcl.start();   // launches both background tasks
```

**Start of autonomous — hard-reset from known wall position:**
```cpp
// Tell RCL what the encoder odom currently reads, then apply sensor-based hard resets
rcl.set_rcl_pose(chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
rcl.update_odom_pose(&rclRight);   // snap X to wall reading
rcl.update_odom_pose(&rclBack);    // snap Y to wall reading
```

**Or: high-precision accumulated reset while stationary:**
```cpp
rcl.set_rcl_pose(chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
rcl.accumulate_for(500);           // collect 500 ms of averaged readings, then apply
```

**After any manual odom reset:**
```cpp
chassis.odom_xyt_set(45.0, -6.0, -90.0);
rcl.set_rcl_pose(45.0, -6.0, -90.0);   // keep RCL dead-reckoning state in sync
```

**Read the RCL-corrected position at any time:**
```cpp
pose p = rcl.get_rcl_pose();   // x, y = RCL estimate; theta = encoder odom heading
```
