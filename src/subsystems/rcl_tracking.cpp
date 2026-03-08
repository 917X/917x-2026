// rcl_tracking.cpp
//
// Ray-Casting Localization (RCL) — adapted from jiazegao/RCL-Tracking for
// EZ-Template (PROS 4 / VEX V5).
//
// Algorithm overview
// ──────────────────
// Each registered RclSensor casts an imaginary ray from its world-frame
// position toward a field wall.  By comparing the measured distance with the
// known field geometry, the robot's absolute coordinate along one axis is
// recovered.  Multiple sensor readings are averaged and validated each cycle
// to produce a refined absolute fix.
//
// Between absolute fixes, odometry dead-reckoning is used to bridge the gap
// (see get_rcl_pose()).  When auto_sync is enabled, the result is gradually
// nudged into the chassis encoder odometry at a rate-limited pace.

#include "subsystems/rcl_tracking.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// ── Internal helpers (file-local) ─────────────────────────────────────────────

// Convert robot heading (EZ-Template convention: 0° = forward, CW positive)
// to standard mathematical angle (CCW from +X east).
static inline double bot_to_trig(double heading_deg) {
  return 90.0 - heading_deg;
}

static inline double deg_to_rad(double deg) {
  return deg * M_PI / 180.0;
}

// Normalise heading to the half-open interval (0°, 360°].
static inline double norm_heading(double h) {
  h = std::fmod(h, 360.0);
  if (h <= 0.0) h += 360.0;
  return h;
}

// ── Static member ─────────────────────────────────────────────────────────────

std::vector<RclSensor*> RclSensor::sensor_collection;

// ── RclSensor ─────────────────────────────────────────────────────────────────

RclSensor::RclSensor(pros::Distance* sensor,
                     double horiz_offset,
                     double vert_offset,
                     double main_angle,
                     double angle_tol)
    : sensor_(sensor),
      main_angle_(main_angle),
      angle_tol_(std::abs(angle_tol)) {
  // Convert Cartesian mounting offsets to polar form.
  // offset_angle is measured CCW from the robot's +X axis (right), in degrees.
  offset_dist_  = std::hypot(horiz_offset, vert_offset);
  offset_angle_ = std::fmod(
      std::atan2(vert_offset, horiz_offset) * 180.0 / M_PI + 360.0, 360.0);
  sensor_collection.push_back(this);
}

void RclSensor::update_pose(const ez::pose& bot_pose) {
  // Sensor world position:
  //   theta = offset_angle - bot_theta  (rotation of offset vector into world frame)
  double theta = deg_to_rad(offset_angle_ - bot_pose.theta);
  sp_.x        = bot_pose.x + std::cos(theta) * offset_dist_;
  sp_.y        = bot_pose.y + std::sin(theta) * offset_dist_;

  // Sensor ray heading in world frame, normalised to (0°, 360°].
  sp_.heading = norm_heading(bot_pose.theta + main_angle_);
}

bool RclSensor::is_valid(double dist_mm) const {
  // Distance out of range
  if (dist_mm > RCL_MAX_DIST_MM || dist_mm <= 0.0) return false;

  // Low confidence on longer-range readings
  if (dist_mm > 200.0 && sensor_->get_confidence() < RCL_MIN_CONF) return false;

  // Only accept readings when ray is close to a cardinal direction.
  // heading_mod in [0°, 90°); valid if ≤ angle_tol_ or ≥ (90° - angle_tol_).
  double heading_mod = std::fmod(sp_.heading, 90.0);
  if (heading_mod > angle_tol_ && heading_mod < (90.0 - angle_tol_)) return false;

  return true;
}

std::pair<RclCoordType, double> RclSensor::get_bot_coord(const ez::pose& bot_pose,
                                                          double accum_mm) {
  update_pose(bot_pose);

  double val_mm = std::isnan(accum_mm)
                      ? static_cast<double>(sensor_->get())
                      : accum_mm;

  if (!is_valid(val_mm)) return {RclCoordType::INVALID, 0.0};

  double val = val_mm * RCL_MM_TO_IN;  // mm → inches

  // Convert sensor ray heading to standard trig angle, then to unit vector.
  // cos_a = cos(90° - heading) = sin(heading)
  // sin_a = sin(90° - heading) = cos(heading)
  double trig_ang = deg_to_rad(bot_to_trig(sp_.heading));
  double cos_a    = std::cos(trig_ang);
  double sin_a    = std::sin(trig_ang);

  // ── Ray–wall intersection ───────────────────────────────────────────────
  // Ray: P(t) = (sp_.x + t·cos_a,  sp_.y + t·sin_a),  t ≥ 0
  // Walls:  East  x =  70.5   North y =  70.5
  //         West  x = −70.5   South y = −70.5
  //
  // Solve for t:
  //   East:  t = (70.5 − sp_.x) / cos_a
  //   West:  t = (−70.5 − sp_.x) / cos_a
  //   North: t = (70.5 − sp_.y) / sin_a
  //   South: t = (−70.5 − sp_.y) / sin_a
  //
  // Only t > 0 is valid (wall must be in front of the sensor).
  // Select the smallest positive t to find the closest wall.

  double min_t = 1e9;
  int    wall  = -1;     // 1=North, 2=East, 3=South, 4=West

  if (std::abs(cos_a) > 1e-6) {
    double t_east = (RCL_FIELD_HALF - sp_.x) / cos_a;
    if (t_east > 0.0 && t_east < min_t) { min_t = t_east; wall = 2; }

    double t_west = (-RCL_FIELD_HALF - sp_.x) / cos_a;
    if (t_west > 0.0 && t_west < min_t) { min_t = t_west; wall = 4; }
  }

  if (std::abs(sin_a) > 1e-6) {
    double t_north = (RCL_FIELD_HALF - sp_.y) / sin_a;
    if (t_north > 0.0 && t_north < min_t) { min_t = t_north; wall = 1; }

    double t_south = (-RCL_FIELD_HALF - sp_.y) / sin_a;
    if (t_south > 0.0 && t_south < min_t) { min_t = t_south; wall = 3; }
  }

  if (wall == -1) return {RclCoordType::INVALID, 0.0};

  // ── Recover sensor world coordinate from measured distance ───────────────
  // If the sensor is at position sp_ and the wall is at known coordinate W,
  // then the sensor position along the relevant axis is:
  //   sp_coord = W − component(val)
  //
  // e.g. East wall (x = 70.5):  sp_.x = 70.5 − cos_a·val
  //      North wall (y = 70.5): sp_.y = 70.5 − sin_a·val

  double      res;
  RclCoordType type;

  if      (wall == 1) { type = RclCoordType::Y;  res =  RCL_FIELD_HALF - sin_a * val; }  // North
  else if (wall == 2) { type = RclCoordType::X;  res =  RCL_FIELD_HALF - cos_a * val; }  // East
  else if (wall == 3) { type = RclCoordType::Y;  res = -RCL_FIELD_HALF - sin_a * val; }  // South
  else                { type = RclCoordType::X;  res = -RCL_FIELD_HALF - cos_a * val; }  // West

  // ── Convert sensor coordinate → robot center coordinate ─────────────────
  // Subtract the projection of the mounting offset onto the relevant axis.
  //   offset in world frame: (cos(offsetAngle − θ), sin(offsetAngle − θ)) × offsetDist
  double off_rad = deg_to_rad(offset_angle_ - bot_pose.theta);
  if      (type == RclCoordType::X) res -= std::cos(off_rad) * offset_dist_;
  else if (type == RclCoordType::Y) res -= std::sin(off_rad) * offset_dist_;

  return {type, res};
}

int RclSensor::raw_reading() const {
  return sensor_->get();
}

RclSensorPose RclSensor::get_pose() const {
  return sp_;
}

// ── RclTracking ───────────────────────────────────────────────────────────────

RclTracking::RclTracking(ez::Drive* chassis,
                         int freq_hz,
                         bool auto_sync,
                         double min_delta,
                         double max_delta,
                         double max_delta_from_odom,
                         double max_sync_per_sec,
                         int min_pause_ms)
    : chassis_(chassis),
      goal_mspt_(static_cast<int>(std::round(1000.0 / freq_hz))),
      min_pause_ms_(min_pause_ms),
      max_sync_pt_(max_sync_per_sec / freq_hz),
      min_delta_(min_delta),
      max_delta_(max_delta),
      max_delta_from_odom_(max_delta_from_odom),
      auto_sync_(auto_sync),
      latest_precise_({0.0, 0.0, 0.0}),
      pose_at_latest_({0.0, 0.0, 0.0}) {}

void RclTracking::start() {
  if (main_task_ == nullptr) {
    main_task_ = new pros::Task([this] { this->main_loop(); });
  }
  if (sync_task_ == nullptr) {
    sync_task_ = new pros::Task([this] { this->sync_loop(); });
  }
}

void RclTracking::stop() {
  if (main_task_ != nullptr) {
    main_task_->remove();
    delete main_task_;
    main_task_ = nullptr;
  }
  if (sync_task_ != nullptr) {
    sync_task_->remove();
    delete sync_task_;
    sync_task_ = nullptr;
  }
}

// ── Pose accessors ────────────────────────────────────────────────────────────

ez::pose RclTracking::get_rcl_pose() const {
  ez::pose odom = chassis_->odom_pose_get();
  return {
    latest_precise_.x + (odom.x - pose_at_latest_.x),
    latest_precise_.y + (odom.y - pose_at_latest_.y),
    odom.theta
  };
}

void RclTracking::set_rcl_pose(double x, double y, double theta) {
  latest_precise_ = {x, y, theta};
  pose_at_latest_ = chassis_->odom_pose_get();
}

void RclTracking::set_rcl_pose(const ez::pose& p) {
  latest_precise_ = p;
  pose_at_latest_ = chassis_->odom_pose_get();
}

// ── Pose application to odom ──────────────────────────────────────────────────

void RclTracking::update_odom_pose() {
  ez::pose p = get_rcl_pose();
  chassis_->odom_xyt_set(p.x, p.y, p.theta);
  set_rcl_pose(p);
}

void RclTracking::update_odom_pose(RclSensor* sensor) {
  if (sensor == nullptr) return;

  ez::pose current = chassis_->odom_pose_get();
  auto [type, coord] = sensor->get_bot_coord(current);

  if (type == RclCoordType::X) {
    chassis_->odom_xyt_set(coord, current.y, current.theta);
    set_rcl_pose(coord, current.y, current.theta);
  } else if (type == RclCoordType::Y) {
    chassis_->odom_xyt_set(current.x, coord, current.theta);
    set_rcl_pose(current.x, coord, current.theta);
  }
  // INVALID → do nothing; chassis pose is unchanged
}

// ── Accumulation control ──────────────────────────────────────────────────────

void RclTracking::start_accumulating(bool auto_update_after) {
  accumulating_       = true;
  update_after_accum_ = auto_update_after;
}

void RclTracking::stop_accumulating() {
  accumulating_ = false;
}

void RclTracking::accumulate_for(int ms, bool auto_update_after) {
  start_accumulating(auto_update_after);
  uint32_t end = pros::millis() + static_cast<uint32_t>(ms);
  while (pros::millis() < end) {
    pros::delay(min_pause_ms_);
  }
  stop_accumulating();
  // The background main_update task handles the final pose update via
  // update_after_accum_; no direct call needed here.
}

void RclTracking::discard_data() {
  ez::pose odom = chassis_->odom_pose_get();
  latest_precise_ = odom;
  pose_at_latest_ = odom;
}

// ── main_update (called once per main_loop tick) ──────────────────────────────

void RclTracking::main_update() {
  if (RclSensor::sensor_collection.empty()) return;

  const size_t n = RclSensor::sensor_collection.size();
  std::vector<int> acc_total(n, 0);
  std::vector<int> acc_count(n, 0);

  // If currently accumulating, collect readings on every mini-cycle until done.
  while (accumulating_) {
    for (size_t i = 0; i < n; ++i) {
      acc_total[i] += RclSensor::sensor_collection[i]->raw_reading();
      acc_count[i]++;
    }
    pros::delay(goal_mspt_);
  }

  // ── Get current best-estimate pose (differential dead-reckoning) ─────────
  ez::pose rcl_pose = get_rcl_pose();
  ez::pose odom_now = chassis_->odom_pose_get();

  // ── Failsafe: nudge RCL toward odom if it has drifted too far ────────────
  // This prevents a stale absolute fix from pulling the odom far off-course.
  double failsafe_diff = std::hypot(rcl_pose.x - odom_now.x,
                                    rcl_pose.y - odom_now.y);
  if (failsafe_diff > max_delta_from_odom_) {
    double inv = 1.0 / failsafe_diff;
    latest_precise_.x += (odom_now.x - rcl_pose.x) * inv;
    latest_precise_.y += (odom_now.y - rcl_pose.y) * inv;
    rcl_pose = get_rcl_pose();
  }

  // ── Query each sensor for its absolute coordinate estimate ────────────────
  std::vector<double> xs, ys;

  for (size_t i = 0; i < n; ++i) {
    RclSensor* sens    = RclSensor::sensor_collection[i];
    double     avg_mm  = (acc_count[i] > 0)
                             ? (static_cast<double>(acc_total[i]) / acc_count[i])
                             : NAN;
    auto [type, coord] = sens->get_bot_coord(rcl_pose, avg_mm);

    if (type == RclCoordType::X) {
      if (std::abs(coord - rcl_pose.x) <= max_delta_) xs.push_back(coord);
    } else if (type == RclCoordType::Y) {
      if (std::abs(coord - rcl_pose.y) <= max_delta_) ys.push_back(coord);
    }
  }

  // ── Update absolute fix with the mean of valid X estimates ───────────────
  if (!xs.empty()) {
    double mean_x = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
    if (mean_x > -RCL_FIELD_HALF && mean_x < RCL_FIELD_HALF &&
        std::abs(mean_x - rcl_pose.x) >= min_delta_) {
      latest_precise_.x = mean_x;
      pose_at_latest_.x = odom_now.x;
    }
  }

  // ── Update absolute fix with the mean of valid Y estimates ───────────────
  if (!ys.empty()) {
    double mean_y = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
    if (mean_y > -RCL_FIELD_HALF && mean_y < RCL_FIELD_HALF &&
        std::abs(mean_y - rcl_pose.y) >= min_delta_) {
      latest_precise_.y = mean_y;
      pose_at_latest_.y = odom_now.y;
    }
  }

  // ── Apply accumulated result to odom if requested ─────────────────────────
  bool any_accumulated = std::any_of(acc_count.begin(), acc_count.end(),
                                     [](int c) { return c > 0; });
  if (update_after_accum_ && any_accumulated) {
    update_odom_pose();
  }
}

// ── sync_update (called once per sync_loop tick) ──────────────────────────────

void RclTracking::sync_update() {
  ez::pose rcl  = get_rcl_pose();
  ez::pose odom = chassis_->odom_pose_get();

  double x_diff = rcl.x - odom.x;
  double y_diff = rcl.y - odom.y;
  double diff   = std::hypot(x_diff, y_diff);

  if (diff < 1e-9) return;  // no meaningful discrepancy

  double x_update, y_update;

  if (diff <= max_sync_pt_) {
    // Discrepancy is small enough to close fully in one step.
    x_update = x_diff;
    y_update = y_diff;
  } else {
    // Apply a capped partial correction in the direction of the RCL estimate.
    double inv    = max_sync_pt_ / diff;
    x_update = x_diff * inv;
    y_update = y_diff * inv;
  }

  chassis_->odom_xyt_set(odom.x + x_update, odom.y + y_update, odom.theta);

  // Keep pose_at_latest_ in sync so get_rcl_pose() stays consistent.
  pose_at_latest_.x     += x_update;
  pose_at_latest_.y     += y_update;
  pose_at_latest_.theta  = odom.theta;
}

// ── Background task loops ─────────────────────────────────────────────────────

void RclTracking::main_loop() {
  while (true) {
    uint32_t now = pros::millis();
    main_update();
    int elapsed   = static_cast<int>(pros::millis() - now);
    int remaining = goal_mspt_ - elapsed;
    pros::delay(remaining > min_pause_ms_ ? remaining : min_pause_ms_);
  }
}

void RclTracking::sync_loop() {
  while (true) {
    uint32_t now = pros::millis();
    if (auto_sync_) sync_update();
    int elapsed   = static_cast<int>(pros::millis() - now);
    int remaining = goal_mspt_ - elapsed;
    pros::delay(remaining > min_pause_ms_ ? remaining : min_pause_ms_);
  }
}
