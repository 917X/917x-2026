#include "main.h"
#include "autons.hpp"
double forwards;
double turning;
float up;
float down;
void arcadeCurve(pros::controller_analog_e_t power,
                 pros::controller_analog_e_t turn, pros::Controller mast,
                 float f) {
  up = mast.get_analog(power);
  down = mast.get_analog(turn);
  forwards =
      (exp(-f / 10) + exp((fabs(up) - 127) / 10) * (1 - exp(-f / 10))) * up;
  turning = -1 * down;
  for (auto &leftMotor : chassis.left_motors) {
    leftMotor.move_velocity(forwards * 0.95 - turning);
  }
  for (auto &rightMotor : chassis.right_motors) {
    rightMotor.move_velocity(forwards * 0.95 + turning);
  }
}

void initialize() {
  pros::delay(500);
  colorSort.set_led_pwm(100);
  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  // set default PID constants
  default_constants();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and "
       "calculate your offsets for your tracking wheels.",
       measure_offsets},
  });

  // Start intake
  pros::Task intakeControl{[=] { intake.intakeControl(); }};

  // Initialize chassis
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  // clear chassis states
  chassis.pid_targets_reset();  // Resets PID targets to 0
  chassis.drive_imu_reset();    // Reset gyro position to 0
  chassis.drive_sensor_reset(); // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in,
                       0_deg);               // Reset odometry position to 0,0,0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold position
  solo_awp();
  //ez::as::auton_selector.selected_auton_call(); // Calls selected auton
}
/**
 * @brief Prints the values of a tracking wheel to the screen
 *
 * @param tracker
 * @param name
 * @param line
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name,
                          int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " +
                    util::to_string_with_precision(
                        tracker->get()); // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(
                                      tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width,
                   line); // Print final tracker text
}

/**
 * @brief debug task for displaying robot status
 *
 */
void telemetry() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print(
              "x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                  "\ny: " +
                  util::to_string_with_precision(chassis.odom_y_get()) +
                  "\na: " +
                  util::to_string_with_precision(chassis.odom_theta_get()),
              1);

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(100);
  }
}
pros::Task telem(telemetry);

void opcontrol() {
  chassis.pid_targets_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  while (true) {
    arcadeCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y,
                pros::E_CONTROLLER_ANALOG_RIGHT_X, master, 5);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.set(Intake::IntakeState::TOPSCORING, 127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.set(Intake::IntakeState::LOWSCORING, 127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.set(Intake::IntakeState::INTAKING, 127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.set(Intake::IntakeState::OUTTAKE, 127);
    } else {
      intake.set(Intake::IntakeState::STOPPED);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
