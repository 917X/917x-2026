#include "main.h"
#include "autons.hpp"
#include "devices.hpp"

void initialize() {
	pros::delay(500);
	chassis.opcontrol_curve_buttons_toggle(false);
	chassis.opcontrol_drive_activebrake_set(0.0);
	chassis.opcontrol_curve_default_set(2.0, 2.0);

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

	// tune pid constants
	// chassis.pid_drive_set(24_in, 90, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(-24_in, 90, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(10_in, 90, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(-10_in, 90, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(48_in, 90, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(-48_in, 90, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(48_in, 90, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(-48_in, 90, true);

	// chassis.pid_turn_set(90_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_turn_set(-90_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_turn_set(45_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_turn_set(-45_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_turn_set(180_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_turn_set(-180_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_turn_set(0_deg, 100);
	// chassis.pid_wait();

	// chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 100);
	// chassis.pid_wait();

	// chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_swing_set(ez::LEFT_SWING, -45_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_swing_set(ez::LEFT_SWING, -180_deg, 100);
	// chassis.pid_wait();
	// chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 100);
	// chassis.pid_wait();
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    
    chassis.pid_odom_set({{36_in, 36_in, 90_deg}, fwd, 90});
    chassis.pid_wait();
    chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, 90});
    chassis.pid_wait();

	while (true) {
		;
	}

	flapperPiston.set(true);
	// clear chassis states
	chassis.pid_targets_reset();  // Resets PID targets to 0
	chassis.drive_imu_reset();	  // Reset gyro position to 0
	chassis.drive_sensor_reset(); // Reset drive sensors to 0
	chassis.odom_xyt_set(0_in, 0_in,
						 0_deg); // Reset odometry position to 0,0,0
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold position
											   // left_elims();
	// solo_awp();
	right_elims();
	// ez::as::auton_selector.selected_auton_call(); // Calls selected auton
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
		if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
			if (ez::as::page_blank_is_on(0)) {
				// Display X, Y, and Theta
				ez::screen_print(
					"x: " +
						util::to_string_with_precision(chassis.odom_x_get()) +
						"\ny: " +
						util::to_string_with_precision(chassis.odom_y_get()) +
						"\na: " +
						util::to_string_with_precision(
							chassis.odom_theta_get()),
					1);

				// Display all trackers that are being used
				screen_print_tracker(chassis.odom_tracker_left, "l", 4);
				screen_print_tracker(chassis.odom_tracker_right, "r", 5);
				screen_print_tracker(chassis.odom_tracker_back, "b", 6);
				screen_print_tracker(chassis.odom_tracker_front, "f", 7);
			}
		}
		pros::delay(100);
	}
}

pros::Task telem(telemetry);

void opcontrol() {
	flapperPiston.set(true);
	chassis.pid_targets_reset();
	chassis.drive_brake_set(MOTOR_BRAKE_COAST);
	while (true) {
		chassis.opcontrol_arcade_standard(ez::SPLIT);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.set(Intake::IntakeState::SCORE, 127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake.set(Intake::IntakeState::INTAKE, 127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake.set(Intake::IntakeState::OUTTAKE, 127);
		} else {
			intake.set(Intake::IntakeState::STOP);
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if (flapperPiston.get() == false) {
				flapperPiston.set(true);
			}
		} else {
			if (flapperPiston.get() == true) {
				flapperPiston.set(false);
			}
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			if (loaderPiston.get() == false) {
				loaderPiston.set(true);
			}
		} else {
			if (loaderPiston.get() == true) {
				loaderPiston.set(false);
			}
		}

		liftPiston.button_toggle(
			master.get_digital(pros::E_CONTROLLER_DIGITAL_B));

		pros::delay(ez::util::DELAY_TIME);
	}
}