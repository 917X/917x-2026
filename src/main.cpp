#include "main.h"
#include "autons.hpp"
#include "devices.hpp"
#include "pros/misc.h"

bool skillsActive = true;

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
            ez::screen_print("hue: " + util::to_string_with_precision(intake.colorSensor->get_hue()), 5);
        ez::screen_print("sat: " + util::to_string_with_precision(intake.colorSensor->get_saturation()), 6);
        ez::screen_print("prox: " + util::to_string_with_precision(intake.colorSensor->get_proximity()), 7);
		}
        
}

void initialize() {
	pros::delay(500);
    intake.colorSensor->set_led_pwm(100);
	chassis.opcontrol_curve_buttons_toggle(false);
	chassis.opcontrol_drive_activebrake_set(0.0);
	chassis.opcontrol_curve_default_set(2.0, 2.0);
	pros::Task telem(telemetry);
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
	loaderPiston.set(false);
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

    //liftPiston.set(true);
    // intake.set(Intake::IntakeState::INTAKE,127);
    // pros::delay(500);
    // chassis.drive_set(67, 67);
    // wait_for_imu_bump(3);
    
    // chassis.drive_set(60,60);
    // wait_for_imu_bump(3);
    // loaderPiston.set(true);
    // chassis.drive_set(0,0);
    // left_elims();
	// skills(); 
	// left_side_9_ball();
	// left_7_rush();
	solo_awp();
	// move_forward();
	// right_elims();
	// right_elims_mid_ball();
    // skills();
    // left_elims();
	// ez::as::auton_selector.selected_auton_call(); // Calls selected auton	
}






void opcontrol() {
	flapperPiston.set(true);
	chassis.pid_targets_reset();
	chassis.drive_brake_set(MOTOR_BRAKE_COAST);

	const double speed_scales[] = {0.2, 0.4, 0.6, 0.8, 1.0};
	int scale_index = 4; // start at full speed
	bool up_pressed_last = false;
    int loader_debounce_delay = 0;

	while (true) {
		bool up_pressed_now = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
		if (up_pressed_now && !up_pressed_last) {
			scale_index = (scale_index + 1) % 5;
			controller.rumble(".");
		}
		up_pressed_last = up_pressed_now;
        controller.print(0, 0, "Speed Scale: %.1f", speed_scales[scale_index]);

		int max_speed = (int)(127 * speed_scales[scale_index]);
		chassis.opcontrol_speed_max_set(max_speed);
		chassis.opcontrol_arcade_standard(ez::SPLIT);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            skillsActive = true;
            controller.rumble(".");
        }


		//INTAKE LOGIC
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			intake.set(Intake::IntakeState::SCORE, 80, 127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			if (skillsActive && liftPiston.get() == true) {

                intake.set(Intake::IntakeState::SCORE, 35, 80);
            } else {
                intake.set(Intake::IntakeState::SCORE, 127);
            }
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake.set(Intake::IntakeState::INTAKE, 127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			if (skillsActive) {
				intake.set(Intake::IntakeState::OUTTAKE, 60);
			}
			else {
				intake.set(Intake::IntakeState::OUTTAKE, 127);
			}
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

        
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if (flapperPiston.get() == true) {
				flapperPiston.set(false);
			}
		} else {
			if (flapperPiston.get() == false) {
				flapperPiston.set(true);
			}
		}

        
		loaderPiston.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT));

		liftPiston.button_toggle(
			master.get_digital(pros::E_CONTROLLER_DIGITAL_B));
	}
}