#include "main.h"
#include "autons.hpp"
#include "devices.hpp"

void initialize() {
	pros::delay(500);
	chassis.opcontrol_curve_buttons_toggle(false);
	chassis.opcontrol_drive_activebrake_set(0.0);
	chassis.opcontrol_curve_default_set(2.0, 2.0);

	// Enable vector scaling and set turn bias for better turning at high speeds
	chassis.opcontrol_arcade_scaling(true);
	chassis.opcontrol_arcade_turn_bias_set(1.34); // Prioritize turn input 1.5x

	// set default PID constants
	default_constants();

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
	flapperPiston.set(true);
	// clear chassis states
	chassis.pid_targets_reset();  // Resets PID targets to 0
	chassis.drive_imu_reset();	  // Reset gyro position to 0
	chassis.drive_sensor_reset(); // Reset drive sensors to 0
	chassis.odom_xyt_set(0_in, 0_in,
						 0_deg); // Reset odometry position to 0,0,0
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold position
											   // left_elims();
	solo_awp();
	//right_elims();
    //skills();
    //left_elims();
	// ez::as::auton_selector.selected_auton_call(); // Calls selected auton
}

/**
 * @brief debug task for displaying robot status
 *
 */
void telemetry() {
	while (true) {
		if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
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
			if (flapperPiston.get() == true) {
				flapperPiston.set(false);
			}
		} else {
			if (flapperPiston.get() == false) {
				flapperPiston.set(true);
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