#include "main.h"

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;



// Calculate the offsets of your tracking wheels

void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}


void solo_awp() {

  intake.set(Intake::IntakeState::INTAKING, 127);

  chassis.odom_xyt_set(45_in,-6_in,-90_deg);
  // chassis.pid_odom_set({{ 22_in, -19_in,180_deg},fwd,80,{-18_in, -24_in}, fwd, 80}, true);
  chassis.pid_odom_set({{22_in,-21_in,-160_deg}, fwd, 90},true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{{17_in, -26_in}, fwd, 60}}, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::LEFT_SWING,135,90,-23,true); //20
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-13_in,80,true);
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::LOWSCORING,127);
  pros::delay(1400);
  
  intake.set(Intake::IntakeState::INTAKING,127);
  chassis.pid_odom_set({{40_in, -44_in}, fwd, 110}, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 91_deg, 90, 0, true);
  chassis.pid_wait_quick_chain();
  pros::delay(50);
  intakePiston.set(true);
 
  chassis.pid_drive_set(7_in, 70, true);
  chassis.pid_wait_quick_chain();
  pros::delay(675);
  chassis.pid_drive_set(-31_in,90,true);
  chassis.pid_wait_quick_chain();
  intakePiston.set(false);
  intake.set(Intake::IntakeState::TOPSCORING,127);
  pros::delay(2000);

  
  intake.set(Intake::IntakeState::ROLLERONLY, 90);
  chassis.pid_swing_set(ez::RIGHT_SWING,0_deg,80,20,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{19_in,25_in,-10_deg},fwd,100});
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::STOPPED);

  chassis.pid_turn_set(-135_deg,100); //50
  chassis.pid_wait_quick();

  chassis.pid_drive_set(12_in,40,true);
  pros::delay(300);
  intake.set(Intake::IntakeState::OUTTAKE,100);
  chassis.pid_wait_quick_chain();
  
  pros::delay(750);
  intake.set(Intake::IntakeState::STOPPED);

}


void left_elims(){
  
  intake.set(Intake::IntakeState::INTAKING, 127);

  chassis.odom_xyt_set(45_in,-6_in,-90_deg);
  // chassis.pid_odom_set({{ 22_in, -19_in,180_deg},fwd,80,{-18_in, -24_in}, fwd, 80}, true);
  chassis.pid_odom_set({{22_in,-21_in,-160_deg}, fwd, 90},true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{{17_in, -26_in}, fwd, 60}}, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::LEFT_SWING,135,90,-23,true); //20
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-13_in,80,true);
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::LOWSCORING,127);
  pros::delay(1400);
  
  intake.set(Intake::IntakeState::INTAKING,127);
  chassis.pid_odom_set({{40_in, -44_in}, fwd, 110}, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 91_deg, 90, 0, true);
  chassis.pid_wait_quick_chain();
  intakePiston.set(true);
  pros::delay(50);
  chassis.pid_drive_set(7_in, 70, true);
  chassis.pid_wait_quick_chain();
  pros::delay(675);
  chassis.pid_drive_set(-31_in,90,true);
  chassis.pid_wait_quick_chain();
  intakePiston.set(false);
  intake.set(Intake::IntakeState::TOPSCORING,127);
  pros::delay(3000);
  intake.set(Intake::IntakeState::STOPPED);
  intakePiston.set(false);

}
void right_elims() {
  intake.set(Intake::IntakeState::INTAKING, 127);

  // Mirrored: 45_in,-6_in,-90_deg becomes 45_in,6_in,90_deg
  chassis.odom_xyt_set(45_in, 6_in, -90_deg);
  
  // Mirrored: 22_in,-21_in,-160_deg becomes 22_in,21_in,160_deg
  chassis.pid_odom_set({{22_in, 21_in, -20_deg}, fwd, 90}, true);
  chassis.pid_wait_quick_chain();
  
  // Mirrored: 17_in,-26_in becomes 17_in,26_in
  chassis.pid_odom_set({{{17_in, 26_in}, fwd, 60}}, true);
  chassis.pid_wait_quick_chain();


  chassis.pid_turn_set(75_deg, 70);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{40_in, 44_in}, fwd, 110}, true);
  chassis.pid_wait_quick_chain();
  
  // Mirrored: RIGHT_SWING,91_deg becomes LEFT_SWING,-91_deg
  chassis.pid_swing_set(ez::LEFT_SWING, 89_deg, 90, 0, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20_in, 60, true);
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::TOPSCORING, 127);
  pros::delay(2000);
  
  chassis.pid_drive_set(27.5_in, 55, true);
  pros::delay(100);
  intake.set(Intake::IntakeState::INTAKING, 127);
  intakePiston.set(true);
  chassis.pid_wait_quick_chain();
  pros::delay(900);
  chassis.pid_drive_set(-27.5_in, 60, true);
  chassis.pid_wait_quick_chain();
  intakePiston.set(false);
  intake.set(Intake::IntakeState::TOPSCORING, 127);
  pros::delay(3000);

  
  
  intake.set(Intake::IntakeState::STOPPED);
}
