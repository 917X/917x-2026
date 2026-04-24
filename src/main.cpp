#include "main.h"
#include "autons.hpp"
#include "devices.hpp"
#include "pros/misc.h"
#include <vector>

bool skillsActive = false;

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
    leverProfiler.setProfile({});
	pros::delay(500);
	leverProfiler.stepTo(0, false);
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
	// long_7_rush();
    // left_elims();
	//skills(); 
	// left_side_9_ball();
	// left_7_rush();
	// solo_awp();
    // four_goal_solo_awp();
	// skills_83_route();
	// four_goal_solo_awp_push();
	//old_solo_awp();
	// move_forward();
	right_elims();
	// right_elims_mid_ball();
    //skills();
    // left_elims();
	// ez::as::auton_selector.selected_auton_call(); // Calls selected auton	
	//  right_7_rush();

	// left_4_rush();

	// right_4_rush();
	//right_4_loader_rush();
	// chassis.pid_drive_exit_condition_set(100_ms, 1_in, 200000_ms, 3_in, 20000000_ms,
	// 									 25000000_ms);
	// chassis.pid_drive_constants_forward_set(12.5, 0.0,34);
	// chassis.pid_drive_set(48,90,true, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(-48,90,false, true);
	// chassis.pid_wait();
	// chassis.pid_drive_constants_forward_set(12.5, 0.0,34);
	// chassis.pid_drive_set(48,90,true, true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(-48,90,false, true);
	// chassis.pid_wait();

	// chassis.pid_turn_exit_condition_set(100_ms, 1.75_deg, 3000000_ms, 5_deg, 250000_ms,
	// 									250000_ms);
	// chassis.pid_turn_constants_set(2.1, 0.0, 17);  //2.4,0,13.7
	// chassis.pid_turn_set(100,127);
	// chassis.pid_wait();
	// chassis.pid_turn_set(0,127);
	// chassis.pid_wait();
	// chassis.pid_turn_set(100,127);
	// chassis.pid_wait();
	// chassis.pid_turn_set(0,127);
	// chassis.pid_wait();
	// chassis.pid_turn_set(100,127);
	// chassis.pid_wait();
	// chassis.pid_turn_set(0,127);
	// chassis.pid_wait();

	// chassis.pid_swing_exit_condition_set(100_ms, 2_deg, 2000050_ms, 7_deg, 5000000_ms,
	// 									 5000000_ms);
	// chassis.pid_swing_constants_set(4,0,23);
	// chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 127, 0, false);
	// chassis.pid_wait();
	// pros::delay(500);
	// chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 127, 0, false);
	// chassis.pid_wait();
	// pros::delay(500);
	// chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 127, 0, false);
	// chassis.pid_wait();
	// pros::delay(500);
	// chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 127, 0, false);
	// chassis.pid_wait();
	// pros::delay(500);
	// chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 127, 0, false);
	// chassis.pid_wait();
	// pros::delay(500);
	// chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 127, 0, false);
	// chassis.pid_wait();
}


std::vector<std::pair<double, double>> groupingProfile_6 = {{-10, 127},{90,45},{100,40}};
std::vector<std::pair<double, double>> groupingProfile_4 = {{-10, 60},{80,75},{95,52},{110,35}}; //{-10, 60},{80,85},{95,60},{110,40} or 38
std::vector<std::pair<double, double>> speedProfile = {{-10, 70},{20,100},{40,110},{60,127}}; //{-10, 95},{20,100},{40,110},{60,127}

std::vector<std::pair<double, double>> groupingProfile_4_lowered = {{-10, 60}, {50, 50}};
std::vector<std::pair<double, double>> speedProfile_lowered = {{-10, 90}, {50, 80}};
bool usingToggle = false;
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
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			if (liftPiston.get() == false){ leverProfiler.setProfile({speedProfile}); }
			else { leverProfiler.setProfile(speedProfile_lowered); }
			intake.set(Intake::IntakeState::SCORE, 127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			if (liftPiston.get() == false){ leverProfiler.setProfile({groupingProfile_4}); }
			else { leverProfiler.setProfile(groupingProfile_4_lowered); }
			intake.set(Intake::IntakeState::SCORE, 127);
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

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && liftPiston.get() == false) {
			if (flapperPiston.get() == true) {
				flapperPiston.set(false);
			}
			usingToggle = false;
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && liftPiston.get()) {
			usingToggle = true;
		} else if (liftPiston.get() == false){
			if (flapperPiston.get() == false) {
				flapperPiston.set(true);
			}
		}

		if (usingToggle) { flapperPiston.button_toggle(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)); }

        
		hoodPiston.button_toggle(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN));
        
		loaderPiston.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT));

		liftPiston.button_toggle( master.get_digital(pros::E_CONTROLLER_DIGITAL_B));
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			if (flapperPiston.get() == true) {
				flapperPiston.set(false);
			}
		}
	
	}
}