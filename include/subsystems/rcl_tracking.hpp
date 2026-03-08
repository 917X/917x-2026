#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

// ── Field dimensions ─────────────────────────────────────────────────────────
// VRC High Stakes 2024-25.  Interior wall edge = ±70.5 inches from center.
static constexpr double RCL_FIELD_HALF = 70.5;

// ── Sensor reading limits ─────────────────────────────────────────────────────
static constexpr double RCL_MAX_DIST_MM   = 2000.0;   // beyond this → invalid
static constexpr int    RCL_MIN_CONF      = 40;        // minimum confidence (0-63) when dist > 200 mm
static constexpr double RCL_MM_TO_IN      = 0.039370078740157;

// ── Coordinate axis that a sensor contributes to ─────────────────────────────
enum class RclCoordType { X, Y, INVALID };

// ── World-frame pose of a sensor ray origin ───────────────────────────────────
struct RclSensorPose {
  double x       = 0.0;   // inches
  double y       = 0.0;   // inches
  double heading = 0.0;   // degrees, CW from initial robot facing
};

// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief A single distance sensor configured for Ray-Casting Localization.
 *
 * Each RclSensor wraps a VEX V5 Distance sensor together with the sensor's
 * geometric mounting position relative to the robot's tracking center, and
 * the direction the sensor faces.  On construction the object registers itself
 * into RclSensor::sensor_collection so that RclTracking picks it up
 * automatically.
 *
 * Math overview
 * ─────────────
 * Given the robot pose (x, y, θ) the sensor's world position is:
 *
 *   s_x = x + cos(offset_angle − θ) × offset_dist
 *   s_y = y + sin(offset_angle − θ) × offset_dist
 *
 * where offset_dist  = hypot(horiz_offset, vert_offset)
 *       offset_angle = atan2(vert_offset, horiz_offset)  [CCW from robot +X]
 *
 * The sensor fires a ray in direction (main_angle + θ).  The ray is cast
 * toward the four field walls (±70.5 in each axis) and the nearest wall is
 * selected.  The measured distance is projected back to recover the sensor's
 * absolute coordinate, then the mounting offset is subtracted to obtain the
 * robot center's coordinate.
 *
 * Cardinal-direction validity
 * ───────────────────────────
 * Readings are only used when the sensor ray is within angle_tol degrees of a
 * cardinal direction (N/E/S/W).  This prevents small heading errors from
 * producing large position errors when the ray grazes a wall at an oblique
 * angle.
 */
class RclSensor {
 public:
  /**
   * @brief Construct and auto-register a new RCL sensor.
   *
   * @param sensor        Pointer to a VEX V5 Distance sensor object.
   * @param horiz_offset  Sensor X offset from tracking center (in). Right = +.
   * @param vert_offset   Sensor Y offset from tracking center (in). Forward = +.
   * @param main_angle    Direction sensor is facing relative to robot forward (deg).
   *                      Forward = 0, Right = 90, Backward = 180, Left = 270.
   * @param angle_tol     Max allowed heading deviation from a cardinal direction
   *                      for a reading to be accepted (deg). Default 15.
   */
  RclSensor(pros::Distance* sensor,
            double horiz_offset,
            double vert_offset,
            double main_angle,
            double angle_tol = 15.0);

  /**
   * @brief Update the sensor's world-frame position and ray heading from the
   *        current robot pose.
   */
  void update_pose(const ez::pose& bot_pose);

  /**
   * @brief Validate a raw distance reading in mm.
   * @return true if the reading is usable for localization.
   */
  bool is_valid(double dist_mm) const;

  /**
   * @brief Compute the robot center's absolute coordinate from this sensor.
   *
   * @param bot_pose  Current best-estimate robot pose.
   * @param accum_mm  Pre-averaged reading in mm; pass NAN to read sensor now.
   * @return {RclCoordType, coordinate_in_inches} or {INVALID, 0}.
   */
  std::pair<RclCoordType, double> get_bot_coord(const ez::pose& bot_pose,
                                                 double accum_mm = NAN);

  /** @brief Get the raw sensor reading in mm. */
  int raw_reading() const;

  /** @brief Get the sensor's current computed world-frame pose. */
  RclSensorPose get_pose() const;

  /**
   * Global registry — every RclSensor instance automatically appends itself
   * here during construction.  RclTracking iterates this collection.
   */
  static std::vector<RclSensor*> sensor_collection;

 private:
  pros::Distance* sensor_;
  double offset_dist_;    // hypot of mounting offsets (in)
  double offset_angle_;   // angle of sensor from tracking center, CCW from robot +X (deg)
  double main_angle_;     // sensor facing direction relative to robot forward (deg)
  double angle_tol_;      // heading tolerance for cardinal direction check (deg)
  RclSensorPose sp_;      // current world-frame sensor pose (updated each call)
};


// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Ray-Casting Localization (RCL) tracking system.
 *
 * Maintains an absolute (x, y) position estimate by casting rays from
 * registered RclSensor objects to field walls and inverting the geometry to
 * compute the robot's coordinate.  An odometry-delta bridge ensures smooth,
 * continuous position tracking between absolute corrections:
 *
 *   rcl_pose = latest_precise + (odom_now − odom_at_last_precise)
 *
 * Optionally auto-syncs the RCL estimate back into the chassis encoder
 * odometry at a rate-limited pace that is safe during active PID motions.
 *
 * This class is purely additive.  The existing encoder odometry runs
 * completely independently and is not affected unless start() is called.
 *
 * Typical usage
 * ─────────────
 *   // Global declarations (devices.cpp or similar)
 *   RclSensor rcl_right(&rightDistance, 4.875, 0.0,    90.0);
 *   RclSensor rcl_back (&backDistance,  0.0,  -4.53, 180.0);
 *   RclTracking rcl(&chassis, 25, true);
 *
 *   // Inside initialize()
 *   rcl.start();
 *
 *   // Inside autonomous(), before motions begin
 *   rcl.set_rcl_pose(chassis.odom_x_get(), chassis.odom_y_get(),
 *                    chassis.odom_theta_get());
 *   rcl.update_odom_pose(&rcl_right);   // hard-reset X from right sensor
 *
 *   // Whenever the chassis pose is externally reset
 *   rcl.set_rcl_pose(new_x, new_y, new_theta);
 */
class RclTracking {
 public:
  /**
   * @param chassis              Pointer to the EZ-Template drive chassis.
   * @param freq_hz              Background update frequency (Hz). Default 25.
   * @param auto_sync            If true, gradually sync RCL estimate into chassis
   *                             odometry. Default true.
   * @param min_delta            Minimum correction magnitude accepted (in).
   *                             Filters out sensor noise.  Default 0.5.
   * @param max_delta            Maximum plausible single-step correction (in).
   *                             Rejects readings obstructed by field objects.
   *                             Default 4.0.
   * @param max_delta_from_odom  Maximum allowed divergence between RCL and odom
   *                             before a failsafe nudge is applied (in). Default 10.
   * @param max_sync_per_sec     Maximum rate at which the RCL estimate is synced
   *                             into odom (in/s). Keeps active PID motions stable.
   *                             Default 3.0.
   * @param min_pause_ms         Minimum task sleep (ms). Prevents CPU starvation.
   *                             Default 20.
   */
  RclTracking(ez::Drive* chassis,
              int freq_hz              = 25,
              bool auto_sync           = true,
              double min_delta         = 0.5,
              double max_delta         = 4.0,
              double max_delta_from_odom = 10.0,
              double max_sync_per_sec  = 3.0,
              int min_pause_ms         = 20);

  /** @brief Start the background main and sync tasks. */
  void start();

  /** @brief Stop the background tasks. */
  void stop();

  /**
   * @brief Get the current RCL-estimated pose.
   *
   * Theta is always taken directly from encoder odometry (the IMU inside
   * EZ-Template already handles heading accurately).
   *
   * @return latest_precise + (odom_delta since last absolute fix)
   */
  ez::pose get_rcl_pose() const;

  /**
   * @brief Set the RCL internal reference pose.
   *
   * Call this whenever the chassis odometry pose is reset externally —
   * e.g. after chassis.odom_xyt_set() — to keep the RCL differential
   * tracking in sync.
   */
  void set_rcl_pose(double x, double y, double theta);
  void set_rcl_pose(const ez::pose& p);

  /**
   * @brief Immediately write the RCL pose estimate into the chassis odometry.
   * Applies get_rcl_pose() → chassis.odom_xyt_set().
   */
  void update_odom_pose();

  /**
   * @brief Hard-reset one axis of the chassis odometry from a single sensor.
   *
   * Useful at the start of an autonomous routine when the robot is close to a
   * known wall and the sensor reading is unobstructed.
   *
   * @param sensor  Sensor to use for the reset.  Pass nullptr to do nothing.
   */
  void update_odom_pose(RclSensor* sensor);

  /**
   * @brief Begin collecting averaged sensor readings for higher precision.
   *
   * While accumulating, the main_update loop collects readings on every cycle
   * instead of applying them immediately.  Call stop_accumulating() (or use
   * accumulate_for()) to stop and optionally apply the averaged result.
   *
   * @param auto_update_after  If true, automatically call update_odom_pose()
   *                           after accumulation ends.
   */
  void start_accumulating(bool auto_update_after = true);

  /** @brief Stop accumulation. */
  void stop_accumulating();

  /**
   * @brief Accumulate for a fixed duration (ms), then stop.
   *
   * This is a blocking call — it waits for the specified number of
   * milliseconds before returning.
   *
   * @param ms                 Duration to accumulate in milliseconds.
   * @param auto_update_after  If true, apply the averaged result afterward.
   */
  void accumulate_for(int ms, bool auto_update_after = true);

  /**
   * @brief Reset the RCL internal state to match the current chassis odom pose.
   * Discards any accumulated absolute fix history.
   */
  void discard_data();

 private:
  ez::Drive* chassis_;
  int goal_mspt_;              // milliseconds per tick
  int min_pause_ms_;
  double max_sync_pt_;         // max sync distance applied per tick (in)
  double min_delta_;
  double max_delta_;
  double max_delta_from_odom_;
  bool auto_sync_;
  bool accumulating_        = false;
  bool update_after_accum_  = false;

  // Differential dead-reckoning state
  ez::pose latest_precise_;   // last absolute RCL fix
  ez::pose pose_at_latest_;   // odom pose at the time of that fix

  pros::Task* main_task_ = nullptr;
  pros::Task* sync_task_ = nullptr;

  // Internal per-cycle update functions
  void main_update();
  void sync_update();

  // Background task loop bodies
  void main_loop();
  void sync_loop();
};
