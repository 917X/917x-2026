#include "devices.hpp"
#include "main.h"
#include "pros/motors.h"
#include "subsystems/intake.hpp"

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
	if (chassis.odom_tracker_left != nullptr)
		chassis.odom_tracker_left->reset();
	if (chassis.odom_tracker_right != nullptr)
		chassis.odom_tracker_right->reset();
	if (chassis.odom_tracker_back != nullptr)
		chassis.odom_tracker_back->reset();
	if (chassis.odom_tracker_front != nullptr)
		chassis.odom_tracker_front->reset();

	for (int i = 0; i < iterations; i++) {
		// Reset pid targets and get ready for running an auton
		chassis.pid_targets_reset();
		chassis.drive_imu_reset();
		chassis.drive_sensor_reset();
		chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
		chassis.odom_xyt_set(0_in, 0_in, 0_deg);
		double imu_start = chassis.odom_theta_get();
		double target =
			i % 2 == 0 ? 90
					   : 270; // Switch the turn target every run from 270 to 90

		// Turn to target at half power
		chassis.pid_turn_set(target, 63, ez::raw);
		chassis.pid_wait();
		pros::delay(250);

		// Calculate delta in angle
		double t_delta = util::to_rad(
			fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

		// Calculate delta in sensor values that exist
		double l_delta = chassis.odom_tracker_left != nullptr
							 ? chassis.odom_tracker_left->get()
							 : 0.0;
		double r_delta = chassis.odom_tracker_right != nullptr
							 ? chassis.odom_tracker_right->get()
							 : 0.0;
		double b_delta = chassis.odom_tracker_back != nullptr
							 ? chassis.odom_tracker_back->get()
							 : 0.0;
		double f_delta = chassis.odom_tracker_front != nullptr
							 ? chassis.odom_tracker_front->get()
							 : 0.0;

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
	if (chassis.odom_tracker_left != nullptr)
		chassis.odom_tracker_left->distance_to_center_set(l_offset);
	if (chassis.odom_tracker_right != nullptr)
		chassis.odom_tracker_right->distance_to_center_set(r_offset);
	if (chassis.odom_tracker_back != nullptr)
		chassis.odom_tracker_back->distance_to_center_set(b_offset);
	if (chassis.odom_tracker_front != nullptr)
		chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

void solo_awp() {
    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-16_in,180_deg);

	chassis.pid_odom_ptp_set({{-46_in,-47_in},fwd,90},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(150);
	chassis.pid_drive_set(11_in,127,false, true, -90); 
	chassis.pid_wait_quick();
    pros::delay(50);

    // score first batch
	chassis.pid_drive_set(-27_in,80,true, true, -90);
	chassis.pid_wait_until(-23_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::SCORE);
    pros::delay(1300);
    intake.set(Intake::IntakeState::INTAKE);

    // get mid balls 
    chassis.pid_swing_set(ez::LEFT_SWING, 50_deg, 100, -10, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-18_in,-22_in},fwd,60},false); 

    
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-19_in,21.5_in},fwd,60},false); 
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKE, 80);
    chassis.pid_wait_quick();

    // score mid balls
    chassis.pid_swing_set(ez::LEFT_SWING, -45_deg, 120, 0, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-10_in, 80,true);
    liftPiston.set(true);


    loaderPiston.set(true);
    chassis.pid_wait_quick();
    intake.set(Intake::IntakeState::OUTTAKE, 127);
    pros::delay(250);
    intake.set(Intake::IntakeState::SCORE, 80, 127);

    //pros::delay(1000);
    
    //return;
    pros::delay(4000);
    loaderPiston.set(false);
    liftPiston.set(false);
    chassis.pid_drive_set(10_in,80,true);
    pros::delay(10000);

    // get second batch matchloads
    liftPiston.set(false);
    intake.set(Intake::IntakeState::INTAKE, 127);
    chassis.pid_odom_ptp_set({{-40_in,44_in},fwd,85},true); 
    chassis.pid_wait_quick();
    chassis.pid_turn_set(-90_deg,127);
    chassis.pid_wait_quick_chain();

    loaderPiston.set(true);
    pros::delay(200);
    chassis.pid_drive_set(13.5_in,100,false, true, -90);
    chassis.pid_wait_quick();
    pros::delay(300);

    // score second batch
    chassis.pid_drive_set(-27_in,80,true, true, -90);
    pros::delay(200);
    intake.set(Intake::IntakeState::OUTTAKE,127);
    pros::delay(175);
    intake.set(Intake::IntakeState::STOP);
    chassis.pid_wait_until(-23_in);
    loaderPiston.set(false);
    
    intake.set(Intake::IntakeState::SCORE);

}

void left_elims() {

	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(45_in, -6_in, -90_deg);

	chassis.pid_odom_set({{22_in, -21_in, -160_deg}, fwd, 90}, true);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_set({{{17_in, -26_in}, fwd, 60}}, true);
	chassis.pid_wait_quick();

	chassis.pid_swing_set(ez::LEFT_SWING, 135, 90, -23, true); 
    intake.set(Intake::STOP);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(-13_in, 80, true);
    liftPiston.set(true);
	chassis.pid_wait_quick_chain();
	intake.set(Intake::IntakeState::SCORE, 105);
	pros::delay(1300);

	intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_odom_ptp_set({{39_in, -42.5_in}, fwd, 110}, true);
	liftPiston.set(false);
    chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 90, 0, true);
	chassis.pid_wait_quick_chain();
	loaderPiston.set(true);
	pros::delay(500);

	chassis.pid_drive_set(7_in, 80, true);
	chassis.pid_wait_quick_chain();
	pros::delay(800);
	chassis.pid_drive_set(-31_in, 90, true);
	chassis.pid_wait_quick_chain();
	
	intake.set(Intake::IntakeState::SCORE, 127);
	pros::delay(1500);
    loaderPiston.set(false);
}
void right_elims() {

    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-16_in,180_deg);

	chassis.pid_odom_ptp_set({{-46_in,-46.5_in},fwd,90},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(150);
	chassis.pid_drive_set(11.5_in,127,false, true, -90); 
	chassis.pid_wait_quick();
    pros::delay(50);

    // score first batch
	chassis.pid_drive_set(-27_in,80,true, true, -90);
	chassis.pid_wait_until(-23_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::SCORE);
    pros::delay(1300);
    intake.set(Intake::IntakeState::INTAKE);

    // get mid balls 
    chassis.pid_swing_set(ez::LEFT_SWING, 50_deg, 100, -10, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-18_in,-22_in},fwd,60},false); 
    chassis.pid_wait_quick();
    chassis.pid_drive_set(-18_in,80,true);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::LEFT_SWING,-90_deg,120,0,false);
    chassis.pid_wait();
    chassis.pid_drive_set(-8_in,80,true);

    intake.set(Intake::IntakeState::SCORE,100);


}

void skills(){
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,16_in,0_deg);
	chassis.pid_turn_set({-46,48},fwd,90);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-46_in,46_in},fwd,120},false); //-46,48
    chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(250);
	chassis.pid_drive_set(9.7_in,127,false, true, -90); 
	chassis.pid_wait_quick();

	pros::delay(900);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_drive_set(-10_in,80,true);
	chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::RIGHT_SWING,90_deg,90, 0.0, ez::e_angle_behavior::counterclockwise,true);
	intake.set(Intake::IntakeState::STOP);
	loaderPiston.set(false);
	chassis.pid_wait_quick();


	//Going to score on opposite side
	chassis.pid_drive_set(60,85,true);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{42_in,49_in},fwd,90},true);
	chassis.pid_wait();
	chassis.pid_turn_set(94,90);
	chassis.pid_wait();
	chassis.pid_drive_set(-18,80,true);
	chassis.pid_wait();
    intake.set(Intake::IntakeState::OUTTAKE,127);
    pros::delay(300);
	intake.set(Intake::IntakeState::SCORE);
	pros::delay(1500);

	chassis.pid_drive_set(28.5_in,60,false);
	loaderPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_wait();
	pros::delay(2000);
    chassis.pid_turn_set(90,90);
    chassis.pid_wait();
    chassis.pid_drive_set(-29,80,true);
    chassis.pid_wait();
    intake.set(Intake::IntakeState::OUTTAKE,127);
    pros::delay(300);
	intake.set(Intake::IntakeState::SCORE);
	pros::delay(1500);
	loaderPiston.set(false);


	//going to intake on the other side loadbar (3rd)

	chassis.pid_swing_set(ez::LEFT_SWING,180,90,10,true);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{44,-45},fwd,110},false);
	chassis.pid_wait_quick();
	chassis.pid_turn_set(90,90);
	loaderPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(9.3_in,50,true);
	chassis.pid_wait();
	pros::delay(2000);
	intake.set(Intake::IntakeState::STOP);
	chassis.pid_drive_set(-10_in,60,true);
	loaderPiston.set(false);
    chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::RIGHT_SWING,-90_deg,90,ez::e_angle_behavior::counterclockwise,true);
	chassis.pid_wait_quick_chain();

    

	//going to score on the other side (4thside)
	chassis.pid_drive_set(60,100,true);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_set({{-44,-43},fwd,110},false);
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,90);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(-15,120,true);
	chassis.pid_wait();
    intake.set(Intake::IntakeState::OUTTAKE,127);
    pros::delay(300);
	intake.set(Intake::IntakeState::SCORE);
	pros::delay(2000);


	chassis.pid_drive_set(28.5_in,50,true);
    intake.set(Intake::INTAKE,127);
    loaderPiston.set(true);
	chassis.pid_wait_quick_chain();
    pros::delay(2000);
    chassis.pid_drive_set(-30,80,true);
    chassis.pid_wait();
    intake.set(Intake::OUTTAKE, 127);
    pros::delay(500);
    intake.set(Intake::SCORE);



	

}


void tune(){

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
}