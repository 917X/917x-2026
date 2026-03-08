#include "subsystems/imu_fusion.hpp"
#include <cmath>

// 1 g in in/s²  (9.80665 m/s² × 39.3701 in/m)
static constexpr double G_TO_INS2 = 386.09;

ImuFusion::ImuFusion(ez::Drive* chassis,
                     double alpha,
                     double vel_decay,
                     int hz,
                     double imu_fwd_sign,
                     double imu_lat_sign)
    : chassis_(chassis),
      alpha_(alpha),
      vel_decay_(vel_decay),
      hz_(hz),
      dt_(1.0 / hz),
      imu_fwd_sign_(imu_fwd_sign),
      imu_lat_sign_(imu_lat_sign) {}

void ImuFusion::start() {
  if (task_ == nullptr) {
    task_ = new pros::Task([this] { this->fusion_loop(); });
  }
}

void ImuFusion::stop() {
  if (task_ != nullptr) {
    task_->remove();
    delete task_;
    task_ = nullptr;
  }
}

void ImuFusion::reset() {
  mutex_.lock();
  vel_x_ = 0.0;
  vel_y_ = 0.0;
  prev_ax_field_ = 0.0;
  prev_ay_field_ = 0.0;
  hp_ax_ = 0.0;
  hp_ay_ = 0.0;
  fused_x_ = chassis_->odom_x_get();
  fused_y_ = chassis_->odom_y_get();
  mutex_.unlock();
}

double ImuFusion::get_x() const {
  mutex_.lock();
  double v = fused_x_;
  mutex_.unlock();
  return v;
}

double ImuFusion::get_y() const {
  mutex_.lock();
  double v = fused_y_;
  mutex_.unlock();
  return v;
}

void ImuFusion::fusion_loop() {
  // Seed fused position from current encoder odometry
  {
    mutex_.lock();
    fused_x_ = chassis_->odom_x_get();
    fused_y_ = chassis_->odom_y_get();
    mutex_.unlock();
  }

  const uint32_t period_ms = static_cast<uint32_t>(1000.0 / hz_);
  uint32_t now = pros::millis();

  while (true) {
    // ── 1. Read IMU accelerometer (g-units) ──────────────────────────────
    pros::imu_accel_s_t accel = chassis_->imu.get_accel();
    double a_fwd = accel.y * imu_fwd_sign_;    // body-forward  acceleration (g)
    double a_lat = accel.x * imu_lat_sign_;    // body-rightward acceleration (g)

    // ── 2. Read heading, normalise to [0°, 360°) ─────────────────────────
    double theta_deg = chassis_->odom_theta_get();
    theta_deg = std::fmod(theta_deg, 360.0);
    if (theta_deg < 0.0) theta_deg += 360.0;
    double theta_rad = theta_deg * M_PI / 180.0;

    // ── 3. Rotate body frame → field frame ───────────────────────────────
    // EZ-Template heading convention: 0° = robot facing +Y (field north), CW positive.
    //   Robot forward unit vector in field (+X, +Y):  (sin θ,  cos θ)
    //   Robot right   unit vector in field (+X, +Y):  (cos θ, −sin θ)
    //
    //   ax_field = a_lat·cos θ + a_fwd·sin θ
    //   ay_field = −a_lat·sin θ + a_fwd·cos θ
    double ax_field = a_lat * std::cos(theta_rad) + a_fwd * std::sin(theta_rad);
    double ay_field = -a_lat * std::sin(theta_rad) + a_fwd * std::cos(theta_rad);

    mutex_.lock();

    // ── 4. First-order discrete high-pass filter ──────────────────────────
    // Removes DC bias and low-frequency gravity drift from the raw signal.
    //   hp[n] = α·(hp[n−1] + x[n] − x[n−1])
    // At α = 0.98, fs = 50 Hz → fc ≈ 0.16 Hz cutoff.
    double new_hp_ax = alpha_ * (hp_ax_ + ax_field - prev_ax_field_);
    double new_hp_ay = alpha_ * (hp_ay_ + ay_field - prev_ay_field_);
    prev_ax_field_ = ax_field;
    prev_ay_field_ = ay_field;
    hp_ax_ = new_hp_ax;
    hp_ay_ = new_hp_ay;

    // ── 5. Integrate HP-filtered acceleration → velocity with decay ───────
    // vel_decay bounds the velocity from growing without limit when the robot
    // is stationary and a small residual filtered signal persists.
    vel_x_ = vel_x_ * vel_decay_ + hp_ax_ * G_TO_INS2 * dt_;
    vel_y_ = vel_y_ * vel_decay_ + hp_ay_ * G_TO_INS2 * dt_;

    // ── 6. Complementary filter: blend IMU prediction with encoder odom ───
    // High-frequency component (wheel slip, dynamics) comes from IMU.
    // Low-frequency / steady-state position is anchored to encoder odometry.
    //   x_fused[n] = α·(x_fused[n−1] + vel_x·dt) + (1−α)·x_odom
    double odom_x = chassis_->odom_x_get();
    double odom_y = chassis_->odom_y_get();
    fused_x_ = alpha_ * (fused_x_ + vel_x_ * dt_) + (1.0 - alpha_) * odom_x;
    fused_y_ = alpha_ * (fused_y_ + vel_y_ * dt_) + (1.0 - alpha_) * odom_y;

    mutex_.unlock();

    // ── 7. Sleep until next tick (compensates for execution time) ─────────
    pros::Task::delay_until(&now, period_ms);
  }
}
