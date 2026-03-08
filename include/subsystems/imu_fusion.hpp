#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "pros/rtos.hpp"
#include <cmath>

/**
 * @brief Optional IMU-based position fusion addon.
 *
 * Fuses VEX V5 IMU accelerometer data with encoder odometry using:
 *   1. A 1st-order discrete high-pass filter that strips DC bias/gravity offset
 *      from raw body-frame acceleration.
 *   2. Velocity integration with per-step decay to prevent unbounded drift.
 *   3. A complementary filter that anchors the long-term position estimate to the
 *      encoder odometry while letting the IMU fill in transient dynamics (e.g.
 *      wheel slip, sudden acceleration).
 *
 * Axis convention (configurable via constructor):
 *   Default assumes VEX brain is mounted flat on the robot with:
 *     IMU accel.y  →  robot-forward axis    (imu_fwd_sign   = +1.0)
 *     IMU accel.x  →  robot-rightward axis  (imu_lat_sign   = +1.0)
 *   Flip the corresponding sign if the axis is reversed on your mounting.
 *
 * Heading convention: matches EZ-Template default.
 *   0° = initial robot facing, clockwise = positive.
 *
 * Usage:
 *   ImuFusion fusion(&chassis);
 *   fusion.start();          // call inside initialize() to run in background
 *   double x = fusion.get_x();
 *   fusion.reset();          // call after any chassis pose reset
 *   fusion.stop();
 *
 * NOTE: get_x() / get_y() are READ-ONLY; this class does not push to the
 * chassis pose. Use the values as a supplementary estimate or manually call
 * chassis.odom_xyt_set() if you wish to apply them.
 *
 * The existing encoder odometry is completely unaffected when this class is not
 * instantiated or not started.
 */
class ImuFusion {
 public:
  /**
   * @param chassis       Pointer to the EZ-Template drive chassis.
   * @param alpha         Complementary / HP filter coefficient (0 < alpha < 1).
   *                      Higher values weight IMU more. Default 0.98 gives a
   *                      HP cutoff of ~0.16 Hz at 50 Hz, blocking DC drift
   *                      while passing robot acceleration dynamics.
   * @param vel_decay     Per-step velocity decay (0 < vel_decay <= 1).
   *                      Bounds unbounded velocity integration drift.
   *                      Default 0.95 at 50 Hz ≈ ~0.4 s time constant.
   * @param hz            Background loop frequency in Hz. Default 50.
   * @param imu_fwd_sign  Sign of IMU accel.y relative to robot forward. ±1.0.
   * @param imu_lat_sign  Sign of IMU accel.x relative to robot rightward. ±1.0.
   */
  ImuFusion(ez::Drive* chassis,
            double alpha = 0.98,
            double vel_decay = 0.95,
            int hz = 50,
            double imu_fwd_sign = 1.0,
            double imu_lat_sign = 1.0);

  /** @brief Start the background fusion task. */
  void start();

  /** @brief Stop the background fusion task. */
  void stop();

  /**
   * @brief Reset velocities and re-sync fused position to current encoder odom.
   * Must be called whenever the chassis pose is externally reset.
   */
  void reset();

  /** @brief Get the fused X position estimate in inches. */
  double get_x() const;

  /** @brief Get the fused Y position estimate in inches. */
  double get_y() const;

 private:
  ez::Drive* chassis_;
  double alpha_;
  double vel_decay_;
  int hz_;
  double dt_;            // seconds per tick = 1.0 / hz_
  double imu_fwd_sign_;
  double imu_lat_sign_;

  // First-order discrete HP filter state
  // Filter eq: hp[n] = alpha * (hp[n-1] + x[n] - x[n-1])
  double prev_ax_field_ = 0.0;  // previous field-frame accel X (g)
  double prev_ay_field_ = 0.0;  // previous field-frame accel Y (g)
  double hp_ax_ = 0.0;          // current HP-filtered field accel X (g)
  double hp_ay_ = 0.0;          // current HP-filtered field accel Y (g)

  // Integrated velocity (in/s)
  double vel_x_ = 0.0;
  double vel_y_ = 0.0;

  // Fused position output (in)
  double fused_x_ = 0.0;
  double fused_y_ = 0.0;

  pros::Task* task_ = nullptr;
  mutable pros::Mutex mutex_;

  void fusion_loop();
};
