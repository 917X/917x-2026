# IMU Fusion Implementation

|                       |            |
|:----------------------|:-----------|
| **Target release**    | 2026-03-07 |
| **Document status**   | DRAFT      |
| **Document owner**    | Hugo Huang |
| **Designer**          | Hugo Huang |
| **Technical writers** | Hugo Huang |

## ▲ In This Document:

- Why deriving position from a raw accelerometer is fundamentally difficult, and where the drift comes from
- How body-frame acceleration is rotated into the absolute field frame using the robot's current heading
- The mathematical derivation of a first-order discrete high-pass filter and why it eliminates sensor bias and gravity drift
- Velocity integration with exponential decay and its interpretation as a leaky integrator
- How the complementary filter fuses the IMU position prediction with encoder odometry across complementary frequency bands
- Parameter reference with physical interpretations and tuning guidance
- Code walkthrough and usage examples

## ▲ Executive Summary

### Purpose & Need

- The VEX V5 IMU contains a three-axis accelerometer in addition to the gyroscope that the chassis library uses for heading. By integrating this accelerometer signal twice — once to get velocity, once to get position — the IMU can independently track the robot's linear motion.
- This is specifically valuable for detecting **wheel slip**, the most common silent failure mode of encoder odometry. When driven wheels lose traction and skid, the wheel encoders and tracking wheels stop reflecting the robot's true motion. The chassis position stalls or under-reports displacement. The IMU accelerometer, being inertially referenced, measures real physical acceleration regardless of what the wheels are doing.
- `ImuFusion` fuses this inertial signal with the encoder odometry via a complementary filter: the odometry anchors the long-term position, and the IMU fills in short-term transient dynamics that wheels cannot observe.
- The output of this system is a read-only fused position estimate exposed through `get_x()` and `get_y()`. It does not write to the chassis odometry on its own, so it cannot disrupt active PID controllers or autonomous motions.

### The Drift Problem

Obtaining position from an accelerometer requires two numerical integrations. There is a fundamental obstacle: **all accelerometers carry a small, persistent measurement offset called bias**. Even a sensor sitting perfectly still reports a small non-zero acceleration. Integrating a constant creates a linearly growing velocity; integrating again creates a quadratically growing position error.

**Quantifying the damage:** A bias of just $1\text{ mg} = 0.001\text{ g}$ corresponds to $0.39\text{ in/s}^2$. Integrating once for $t$ seconds produces a velocity error of $0.39t$ in/s. Integrating a second time gives a position error of $0.19t^2$ inches. After a 15-second autonomous: **43 inches of drift** from a bias smaller than a typical IMU calibration margin.

**The solution — two mechanisms working together:**
1. A **high-pass filter** on the raw acceleration removes the DC bias before integration, eliminating the drift source at its root.
2. A **complementary filter** continuously re-anchors the position output to the encoder odometry, bounding any residual drift that survives stage 1.

## ▲ Architecture

### External Dependencies

|  **Component**  |  **Type**  |  **Purpose**  |
|:----------------|:-----------|:--------------|
| `pros::Imu::get_accel()` | Kernel Library | Raw 3-axis accelerometer reading in g-units |
| `pros::Task` | Kernel Library | Runs the fusion loop as a persistent background task |
| `pros::Mutex` | Kernel Library | Serializes concurrent access to `fused_x_` / `fused_y_` between the fusion task and external readers |
| `Drive::odom_x_get()` / `odom_y_get()` | Chassis Library | Current encoder odometry position, sampled each tick for the complementary blend |
| `Drive::odom_theta_get()` | Chassis Library | Current heading in degrees, used to rotate body-frame acceleration into the field frame |

### Internal Dependencies

|  **Component**  |  **Purpose**  |
|:----------------|:--------------|
| `ImuFusion` class | Fully self-contained; depends on nothing else within this project |

### Interfaces

|  **Type**  |  **Name**  |
|:-----------|:-----------|
| Component Input | IMU accelerometer via `chassis->imu.get_accel()`, encoder odom pose, current heading |
| Component Output | `get_x()`, `get_y()` — fused position estimate in inches (read-only) |

## ▲ Design Spec

### Step 1: Body Frame to Field Frame — The Rotation Problem

The IMU is physically attached to the robot. Its accelerometer axes are defined relative to the robot's own body:

- `accel.y`: acceleration along the robot's forward–backward axis (positive = forward)
- `accel.x`: acceleration along the robot's lateral axis (positive = rightward)

The field uses an absolute coordinate system independent of the robot's orientation:

- **Field $x$**: horizontal, positive east
- **Field $y$**: vertical, positive north (robot forward when heading = 0°)

When the robot is facing forward (heading = 0°), body and field axes coincide. As the robot turns, the body axes rotate with it. A "forward" robot acceleration points north at 0°, east at 90°, south at 180°. Before any computation, body-frame measurements must be mapped to field-frame values.

#### The Rotation Matrix

The heading convention is: 0° = north (robot forward), increasing clockwise. Given heading $\theta$ in degrees, the robot's body unit vectors expressed in field coordinates are:

$$\hat{e}_\text{forward} = \begin{pmatrix}\sin\theta \\ \cos\theta\end{pmatrix} \qquad \hat{e}_\text{right} = \begin{pmatrix}\cos\theta \\ -\sin\theta\end{pmatrix}$$

**Verification:** At $\theta = 0°$ (facing north): $\hat{e}_\text{forward} = (0, 1)$ — points in the $+y$ direction. ✓ At $\theta = 90°$ (facing east): $\hat{e}_\text{forward} = (1, 0)$ — points in the $+x$ direction. ✓

The field-frame acceleration is the body-frame acceleration decomposed onto these axes:

$$a_{x,\text{field}} = a_\text{lat} \cdot \cos\theta + a_\text{fwd} \cdot \sin\theta$$

$$a_{y,\text{field}} = -a_\text{lat} \cdot \sin\theta + a_\text{fwd} \cdot \cos\theta$$

where $a_\text{fwd}$ = `accel.y` × `imu_fwd_sign` and $a_\text{lat}$ = `accel.x` × `imu_lat_sign`.

[Note: Insert diagram here — **"Body-Frame to Field-Frame Rotation"**. Draw a top-down view of the VRC field. Place the robot at an angle θ from north. Draw two sets of axes from the robot's center: (1) body-frame axes (dashed) labeled $\hat{x}_\text{body}$ (pointing right of robot) and $\hat{y}_\text{body}$ (pointing forward of robot); (2) field-frame axes (solid) labeled $\hat{x}$ (east) and $\hat{y}$ (north), aligned with the field. Show the angle θ between the body forward axis and field north. Draw an example acceleration vector in the body frame and annotate the projections onto the field $\hat{x}$ and $\hat{y}$ axes using $\sin\theta$ and $\cos\theta$ components.]

#### Axis Sign Configuration

The formulas above assume `accel.y` is forward and `accel.x` is rightward. The actual sign depends on how the VEX brain is physically mounted. The constructor accepts `imu_fwd_sign` and `imu_lat_sign` (each ±1.0) to account for any mounting orientation with no change to the core math.

### Step 2: The High-Pass Filter — Removing Sensor Bias

#### What Is a High-Pass Filter?

A **high-pass filter** passes signals whose frequency is above a threshold (the *cutoff frequency*) and suppresses signals below it. Sensor bias is a zero-frequency (DC) signal — it is constant, not oscillating. By applying a high-pass filter to the raw acceleration, the bias is removed while robot dynamics (which occur at much higher frequencies) pass through completely untouched.

#### Continuous-Time First-Order High-Pass Filter

The simplest high-pass filter is a first-order system whose governing differential equation is:

$$\tau \frac{dy}{dt} + y(t) = \tau \frac{dx}{dt}$$

where $x(t)$ is the input, $y(t)$ is the filtered output, and $\tau$ is the **time constant**. In the Laplace domain, the transfer function is:

$$H(s) = \frac{s\tau}{1 + s\tau}$$

At $s = 0$ (DC frequency): $H(0) = 0$ — DC is completely blocked. At high frequencies ($s \to \infty$): $H \to 1$ — high frequencies pass through unchanged. The $-3\text{ dB}$ cutoff frequency is $f_c = \frac{1}{2\pi\tau}$.

#### Discretizing for Code

To run this filter on a microcontroller, we discretize the differential equation using the forward Euler method, replacing the derivative with $\frac{dy}{dt} \approx \frac{y[n] - y[n-1]}{T}$ where $T = 1/f_s$ is the sample period:

$$\tau \cdot \frac{y[n] - y[n-1]}{T} + y[n] = \tau \cdot \frac{x[n] - x[n-1]}{T}$$

Solving for $y[n]$:

$$y[n]\left(\frac{\tau}{T} + 1\right) = \frac{\tau}{T}\left(y[n-1] + x[n] - x[n-1]\right)$$

Defining $\alpha = \dfrac{\tau}{\tau + T}$, this simplifies to:

$$\boxed{y[n] = \alpha \cdot \bigl( y[n-1] + x[n] - x[n-1] \bigr)}$$

This is the exact recurrence relation used in the code. It requires storing only two values per axis: the previous filtered output $y[n-1]$ and the previous raw input $x[n-1]$.

#### Interpreting $\alpha$

Given parameter $\alpha$ and sample rate $f_s$, the time constant and cutoff frequency are:

$$\tau = \frac{\alpha \cdot T}{1 - \alpha} = \frac{\alpha}{f_s(1-\alpha)} \qquad\qquad f_c = \frac{f_s(1-\alpha)}{2\pi\alpha}$$

With the defaults $\alpha = 0.98$, $f_s = 50\text{ Hz}$:

$$\tau = \frac{0.98}{50 \times 0.02} \approx 0.98 \text{ s} \qquad\qquad f_c \approx \frac{50 \times 0.02}{2\pi \times 0.98} \approx 0.16 \text{ Hz}$$

Everything below 0.16 Hz — including constant sensor bias — is strongly attenuated. Robot dynamic motion (starting, stopping, turning, impact) typically occurs in the 0.5–10 Hz range and passes through completely unaffected.

[Note: Insert diagram here — **"High-Pass Filter Frequency Response"**. Plot amplitude in dB (y-axis, 0 to −40 dB) versus frequency in Hz (x-axis, log scale from 0.01 to 25 Hz). Draw the Bode magnitude curve for the HP filter with α = 0.98 at 50 Hz. Mark the −3 dB crossover at 0.16 Hz with a dashed vertical line. Shade the region below 0.16 Hz red and label it "Bias / drift — blocked". Shade the region above 0.16 Hz green and label it "Robot dynamics — passed". Annotate the 0 dB plateau in the passband.]

### Step 3: Velocity Integration with Decay

After high-pass filtering, we integrate acceleration to get velocity. The unit conversion factor $G = 386.09\text{ in/s}^2/\text{g}$ (derived as $9.80665\text{ m/s}^2 \times 39.3701\text{ in/m}$) converts from g-units to in/s²:

$$v_x[n] = \beta \cdot v_x[n-1] + a_{x,\text{HP}}[n] \cdot G \cdot dt$$

where $\beta$ is the **velocity decay** coefficient (default 0.95).

**Why not pure integration ($\beta = 1$)?** Even after high-pass filtering, a tiny noise floor persists below the cutoff frequency. Without any decay, this residual integrates indefinitely — the estimated velocity slowly drifts even when the robot is completely stationary. The decay factor $\beta < 1$ gently pulls the velocity back toward zero whenever no genuine acceleration is present.

**Physical interpretation of $\beta$:** The velocity behaves like a *leaky integrator* with time constant:

$$\tau_v = \frac{T}{1 - \beta} = \frac{0.02}{0.05} = 0.4 \text{ s}$$

A spurious velocity of 1 in/s injected at time zero decays to $1/e \approx 0.37$ in/s after 0.4 s, and is effectively negligible after ~2 s. Genuine dynamic velocities during robot motion are replenished every cycle and accumulate correctly, because the per-step decay ($5\%$) is far smaller than a real acceleration contribution.

### Step 4: The Complementary Filter — Blending IMU and Odometry

The complementary filter is the final stage. Each tick, we form a weighted blend of two position predictions:

$$\boxed{x_\text{fused}[n] = \alpha \cdot \bigl(x_\text{fused}[n-1] + v_x[n] \cdot dt\bigr) + (1-\alpha) \cdot x_\text{odom}[n]}$$

The two terms cover **complementary frequency bands** — which is where the filter gets its name:

- The **IMU term** $\alpha \cdot (x_\text{fused} + v \cdot dt)$ applies a high-pass weighting, emphasizing recent high-frequency position changes. This is where slip and impact response lives.
- The **odometry term** $(1-\alpha) \cdot x_\text{odom}$ applies a low-pass weighting, continuously anchoring the estimate to the slow-varying, reliable encoder position.

Because the two coefficients sum to 1, the filter never amplifies or attenuates the total signal — it only redistributes trust between the two sources based on frequency.

**Intuition:** Think of estimating where a moving vehicle is. GPS gives accurate absolute position but updates slowly. Wheel odometry gives smooth continuous position but may drift. A complementary filter says: *"For where I am in the long run, I trust GPS. For what I've done in the last second, I trust the wheels."* Here, encoder odometry plays the GPS role (absolute, low-frequency) and the IMU plays the wheel role (relative, high-frequency).

[Note: Insert diagram here — **"Complementary Filter Block Diagram"**. Draw a signal flow diagram with two input branches meeting at an adder (Σ): (1) Upper branch: "IMU accel" → "Body→Field Rotation" → "High-Pass Filter" → "Velocity Decay Integrator" → "× α" → Σ; (2) Lower branch: "Encoder Odom $x_\text{odom}$" → "× (1−α)" → Σ. The output of Σ is labeled "$x_\text{fused}[n]$". Draw a feedback arrow from the Σ output back to the input of the Velocity Decay Integrator, labeled "$x_\text{fused}[n-1]$". Show that the two multiplier boxes "× α" and "× (1−α)" together cover all of the input signal (complementary).]

### Complete Algorithm at a Glance

```
Every 20 ms (50 Hz):
  1. Read accel.y (forward, g) and accel.x (lateral, g); apply imu_fwd_sign / imu_lat_sign
  2. Read heading θ (degrees); normalize to [0°, 360°)
  3. Rotate body frame → field frame:
       ax = a_lat · cos θ + a_fwd · sin θ
       ay = −a_lat · sin θ + a_fwd · cos θ
  4. High-pass filter each axis:
       hp_ax[n] = α · (hp_ax[n−1] + ax[n] − ax[n−1])
       hp_ay[n] = α · (hp_ay[n−1] + ay[n] − ay[n−1])
  5. Integrate to velocity with decay:
       vel_x = vel_x · β + hp_ax · G · dt
       vel_y = vel_y · β + hp_ay · G · dt
  6. Complementary blend with encoder odom:
       fused_x = α · (fused_x + vel_x · dt) + (1−α) · odom_x
       fused_y = α · (fused_y + vel_y · dt) + (1−α) · odom_y
  7. Expose fused_x, fused_y via get_x() / get_y() (mutex-protected)
```

## ▲ Parameter Reference

|  **Parameter**  |  **Default**  |  **Typical Range**  |  **Effect**  |
|:----------------|:--------------|:--------------------|:-------------|
| `alpha` | 0.98 | 0.90–0.999 | HP filter cutoff and complementary weight. Higher → more IMU trust, higher HP cutoff (less bias suppression), more transient sensitivity. Lower → more odom trust, more drift suppression, less responsive |
| `vel_decay` | 0.95 | 0.80–0.999 | Velocity leak rate. Lower → faster decay of spurious velocity (better at rest) but real dynamic velocities are also slightly damped during motion |
| `hz` | 50 | 25–100 | Loop frequency (Hz). Lower = less CPU use; higher = faster transient response |
| `imu_fwd_sign` | +1.0 | ±1.0 | Flip to −1.0 if `accel.y` points backward relative to robot forward on your brain mounting |
| `imu_lat_sign` | +1.0 | ±1.0 | Flip to −1.0 if `accel.x` points leftward relative to robot right on your brain mounting |

## ▲ Implementation Overview

### File Structure

```txt
include/
└── subsystems/
    └── imu_fusion.hpp       # ImuFusion class declaration

src/
└── subsystems/
    └── imu_fusion.cpp       # fusion_loop(), filter math, task lifecycle
```

### Usage

**Basic setup (devices.cpp):**
```cpp
ImuFusion imuFusion(&chassis);       // inactive until start() is called
```

**Launch background task (initialize()):**
```cpp
imuFusion.start();
```

**Read the fused estimate at any time:**
```cpp
double fx = imuFusion.get_x();
double fy = imuFusion.get_y();
```

**Always call reset() after any external odometry change:**
```cpp
chassis.odom_xyt_set(45.0, -6.0, -90.0);
imuFusion.reset();   // re-seeds fused state from the new odom values
```

**For a non-standard brain mounting (e.g., brain rotated 90° so accel.x is now forward):**
```cpp
// imu_fwd_sign: accel.x is forward → use imu_lat field with sign +1, imu_fwd with sign 0 is not right...
// Example: brain rotated CCW 90° — accel.x is now the forward axis and is correct sign:
ImuFusion imuFusion(&chassis, 0.98, 0.95, 50, /*fwd_sign=*/1.0, /*lat_sign=*/-1.0);
```

## ▲ Implementation Explainer

### Section I: Initialization and Task Lifecycle

The constructor stores all parameters and pre-computes `dt_` once at startup:

```cpp
ImuFusion::ImuFusion(Drive* chassis, double alpha, double vel_decay,
                     int hz, double imu_fwd_sign, double imu_lat_sign)
    : chassis_(chassis), alpha_(alpha), vel_decay_(vel_decay),
      hz_(hz), dt_(1.0 / hz), ... {}
```

`start()` creates the PROS task only if one is not already running. The null-check guard makes repeated calls safe:

```cpp
void ImuFusion::start() {
  if (task_ == nullptr)
    task_ = new pros::Task([this] { this->fusion_loop(); });
}
```

`reset()` is a required maintenance call. Any time the encoder odometry pose is externally changed (for example, `chassis.odom_xyt_set()` at the start of an autonomous), the complementary filter's internal state must be re-seeded to the new odometry value. Without this call, the $(1-\alpha)$ anchor term correctly pulls toward the new odom, but the $\alpha$ IMU-predicted term continues adding onto the stale fused position from before the reset, creating an immediate jump error:

```cpp
void ImuFusion::reset() {
  mutex_.lock();
  vel_x_ = vel_y_ = 0.0;
  hp_ax_ = hp_ay_ = prev_ax_field_ = prev_ay_field_ = 0.0;
  fused_x_ = chassis_->odom_x_get();
  fused_y_ = chassis_->odom_y_get();
  mutex_.unlock();
}
```

### Section II: Reading and Rotating Acceleration

At startup the fusion loop seeds the fused position from the current odom, then enters the main loop. Each tick reads the accelerometer, applies the sign corrections, and fetches the current heading:

```cpp
pros::imu_accel_s_t accel = chassis_->imu.get_accel();
double a_fwd = accel.y * imu_fwd_sign_;
double a_lat = accel.x * imu_lat_sign_;

// Normalize heading to [0°, 360°), convert to radians
double theta_rad = /* ... */ * M_PI / 180.0;

double ax_field = a_lat * std::cos(theta_rad) + a_fwd * std::sin(theta_rad);
double ay_field = -a_lat * std::sin(theta_rad) + a_fwd * std::cos(theta_rad);
```

The heading is sourced from `odom_theta_get()` — the same IMU-based heading used by the chassis. Because it is read-only here and the gyroscope is accurate, this does not create any circular dependency.

### Section III: High-Pass Filter

Both axes are filtered simultaneously. The new output values are computed before overwriting the stored state, ensuring both axes are updated from a consistent previous moment in time:

```cpp
double new_hp_ax = alpha_ * (hp_ax_ + ax_field - prev_ax_field_);
double new_hp_ay = alpha_ * (hp_ay_ + ay_field - prev_ay_field_);
prev_ax_field_ = ax_field;   prev_ay_field_ = ay_field;
hp_ax_ = new_hp_ax;          hp_ay_ = new_hp_ay;
```

Four state variables are maintained per axis: the previous raw input (`prev_ax_field_`, the $x[n-1]$ term) and the previous filtered output (`hp_ax_`, the $y[n-1]$ term). Both are required by the recurrence relation derived in Step 2.

### Section IV: Velocity Decay Integration

The constant `G_TO_INS2 = 386.09` converts from g-units to in/s²:

$$386.09 = 9.80665 \;\frac{\text{m}}{\text{s}^2} \times 39.3701 \;\frac{\text{in}}{\text{m}}$$

```cpp
vel_x_ = vel_x_ * vel_decay_ + hp_ax_ * G_TO_INS2 * dt_;
vel_y_ = vel_y_ * vel_decay_ + hp_ay_ * G_TO_INS2 * dt_;
```

The HP-filtered acceleration is multiplied by the conversion factor and step size before being added to the leaky-integrated velocity.

### Section V: Complementary Blend

The encoder odometry is sampled at the same instant as the accelerometer, keeping both inputs time-aligned. Using stale odometry would introduce a timestamp mismatch that the filter has no way to compensate for:

```cpp
double odom_x = chassis_->odom_x_get();
double odom_y = chassis_->odom_y_get();
fused_x_ = alpha_ * (fused_x_ + vel_x_ * dt_) + (1.0 - alpha_) * odom_x;
fused_y_ = alpha_ * (fused_y_ + vel_y_ * dt_) + (1.0 - alpha_) * odom_y;
```

### Section VI: Thread Safety

`fused_x_` and `fused_y_` are written by the background fusion task and read externally via `get_x()` / `get_y()`. A `pros::Mutex` serializes this access. The mutex is held for the entire computation block (not just the write) so that the two output values always correspond to the same computation cycle:

```cpp
double ImuFusion::get_x() const {
  mutex_.lock();
  double v = fused_x_;
  mutex_.unlock();
  return v;
}
```

### Section VII: Task Timing

The fusion loop uses `pros::Task::delay_until()` rather than a bare `pros::delay()`. This subtracts the loop body's own execution time from the sleep duration, keeping the loop period accurate at exactly `period_ms` milliseconds regardless of computation cost:

```cpp
uint32_t now = pros::millis();
while (true) {
  // ... all computation ...
  pros::Task::delay_until(&now, period_ms);
}
```
