#include "EZ-Template/util.hpp"
#include "devices.hpp"
#include "main.h"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/localizer.hpp"

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
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 100_ms,
										 250_ms);
    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-16_in,180_deg);

	chassis.pid_odom_ptp_set({{-46_in,-47.5_in},fwd,90},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(11_in,127,false, true, -90); 
	chassis.pid_wait_quick();

    // score first batch
	chassis.pid_drive_set(-27_in,80,true, true, -90);
	chassis.pid_wait_until(-23_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::SCORE);
    pros::delay(1100);
    intake.set(Intake::IntakeState::INTAKE);

    // get mid balls 
    chassis.pid_swing_set(ez::LEFT_SWING, 50_deg, 100, 5, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-19_in,-22_in},fwd,40},false); 

    
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-19_in,22.3_in},fwd,60},false); 
	chassis.pid_wait_until(24_in);
    loaderPiston.set(true);
    chassis.pid_wait_quick();

    // score mid balls
    chassis.pid_swing_set(ez::LEFT_SWING, -45_deg, 120, 0, false);
	
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-10.2_in, 80,true);
    liftPiston.set(true);
    intake.set(Intake::IntakeState::OUTTAKE);
	pros::delay(200);
	intake.set(Intake::INTAKE);

    chassis.pid_wait_until(-5_in);
    intake.set(Intake::IntakeState::SCORE, 55, 100);

    pros::delay(1300);

    loaderPiston.set(false);

    // get second batch matchloads
    liftPiston.set(false);
    intake.set(Intake::IntakeState::INTAKE, 127);
    chassis.pid_odom_ptp_set({{-40_in,45_in},fwd,85},true); //-40,45.2
    chassis.pid_wait_quick();
    chassis.pid_turn_set(-90_deg,127);
    chassis.pid_wait_quick_chain();

    loaderPiston.set(true);
    pros::delay(400);
    chassis.pid_drive_set(14_in,100,false, true, -90);
    chassis.pid_wait_quick();

    // score second batch
    chassis.pid_drive_set(-27_in,80,true, true, -90);
    
    chassis.pid_wait_until(-23_in);
    loaderPiston.set(false);
    
    intake.set(Intake::IntakeState::SCORE);

}

void left_elims() {
	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(45_in, -6_in, -90_deg);

	chassis.pid_odom_ptp_set({{22_in, -21_in, -160_deg}, fwd, 60}, true);
	chassis.pid_wait_until(10);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{16_in,-26_in},fwd,67},false); //-46,48
	chassis.pid_wait_quick();

	chassis.pid_swing_set(ez::LEFT_SWING, 135, 90, -23, true);
	chassis.pid_wait_quick_chain();
    liftPiston.set(true);
	chassis.pid_drive_set(-11_in, 80, true);
	chassis.pid_wait_quick_chain();
	intake.set(Intake::IntakeState::SCORE, 60,127);
	pros::delay(1300);

	intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_odom_ptp_set({{39_in, -42_in}, fwd, 110}, true); //39,-41.5
	liftPiston.set(false);
    chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 90, 0, true);
	chassis.pid_wait_quick();
	loaderPiston.set(true);
	pros::delay(500);
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 100_ms,
										 250_ms);
	chassis.pid_drive_set(7.5_in, 80, true);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(-31_in, 90, true);
	chassis.pid_wait_quick_chain();
	
	intake.set(Intake::IntakeState::SCORE, 127);
	pros::delay(1500);
	loaderPiston.set(false);
    // chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 100, 6, false);
	chassis.pid_swing_set(ez::LEFT_SWING, 155_deg, 110, 3, false); //158
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::RIGHT_SWING, 94_deg, 100, false);
    chassis.pid_wait_quick_chain();
	

	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 15000_ms,
										 15000_ms);
    flapperPiston.set(false);
    chassis.pid_drive_set(-36.7_in,85,true);
}

void move_forward(){
	chassis.pid_drive_set(5,50);
}


void right_elims() {
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 100_ms,
										 250_ms);
    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-16_in,180_deg);

	chassis.pid_odom_ptp_set({{-46_in,-47.7_in},fwd,127},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(11.5_in,127,false, true, -90); 
	chassis.pid_wait_quick();


    // score first batch
	chassis.pid_drive_set(-27_in,127,false, true, -90);
	chassis.pid_wait_until(-22.5_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::SCORE);
    pros::delay(1200);
    intake.set(Intake::IntakeState::INTAKE);

    // chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 100, 6, false);
	chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 110, 3, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::RIGHT_SWING, -86_deg, 100, false);
    chassis.pid_wait_quick_chain();
	
	// chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 15000_ms,
										//  15000_ms);
    flapperPiston.set(false);
    chassis.pid_drive_set(-36.7_in,117,false); //85
	chassis.pid_wait_quick();
	// chassis.pid_swing_exit_condition_set(10000_ms, 1_deg, 10000_ms, 7_deg, 15000_ms,
	// 									 15000_ms);
	chassis.pid_swing_set(ez::RIGHT_SWING,-140,127,-5,true);

	



}

void right_elims_mid_ball() {
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 95_ms,
										 250_ms);
    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-16_in,180_deg);

	chassis.pid_odom_ptp_set({{-46_in,-47.7_in},fwd,127},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(11.5_in,127,false, true, -90); 
	chassis.pid_wait_quick();


    // score first batch
	chassis.pid_drive_set(-27_in,127,false, true, -90);
	chassis.pid_wait_until(-22.5_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::SCORE);
    pros::delay(1200);
    intake.set(Intake::IntakeState::INTAKE);

    // get mid balls 

    chassis.pid_swing_set(ez::LEFT_SWING, 56_deg, 100, 7, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(33_in,60,false); 
	chassis.pid_wait_quick();
	chassis.pid_swing_set(ez::LEFT_SWING,34,127);
	chassis.pid_wait();
	liftPiston.set(true);
	chassis.pid_drive_set(4_in, 60, true);
	chassis.pid_wait_quick_chain();
	pros::delay(1000);
	intake.set(Intake::IntakeState::OUTTAKE,80);
	pros::delay(500);
	intake.set(Intake::IntakeState::INTAKE,127);
	pros::delay(300);
	intake.set(Intake::IntakeState::OUTTAKE,70);
	pros::delay(1000);
	
	chassis.pid_drive_set(-5,127);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-18_in,-45_in},rev,100},false);
	liftPiston.set(false);
	chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::LEFT_SWING,-90,127);
	chassis.pid_wait_quick_chain();
	flapperPiston.set(false);
	chassis.pid_drive_set(-20_in,117,false); //85
	



}


void skills_83_route() {
	okapi::QLength x;
	okapi::QLength y;
	flapperPiston.set(true);
	chassis.odom_xyt_set(-52_in,-18_in,180_deg);
	intake.set(Intake::IntakeState::INTAKE,127);


	chassis.pid_swing_set(ez::RIGHT_SWING,90_deg,90,4,true);
	chassis.pid_wait();
	chassis.pid_drive_set(50_in,65,true);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(29.9_in,40,true); //29.5
	chassis.pid_wait_until(22.5_in);
	loaderPiston.set(true);
	chassis.pid_wait();

	//INTAKED EIGHT BALLS GOING TO SCORE IN MIDDLE
	chassis.pid_swing_set(ez::RIGHT_SWING,135_deg,90,33,true);
	liftPiston.set(true);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(-9.2_in,70,true); //8.8
	chassis.pid_wait_quick();
	intake.set(Intake::IntakeState::SCORE,50,100);
	pros::delay(500);
	intake.set(Intake::IntakeState::OUTTAKE,127);
	pros::delay(200);
	intake.set(Intake::IntakeState::SCORE,50,127);
	pros::delay(2000);


	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_odom_ptp_set({{46_in,-44.7_in},fwd,80},true); //46,46
	liftPiston.set(false);
	chassis.pid_wait_quick_chain();

	chassis.pid_turn_set(88,127);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(14.7_in,60,true);
	chassis.pid_wait();
	pros::delay(500);
	//localizing
	x = (71 - 14.3125)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::BR);
	chassis.odom_xy_set(x,y);
	pros::delay(500);
	
	chassis.pid_drive_set(-30_in,100,true);
	chassis.pid_wait_quick(); 
	intake.set(Intake::IntakeState::SCORE,127);
	pros::delay(700);
	loaderPiston.set(false);
	pros::delay(1000);
	intake.set(Intake::IntakeState::INTAKE,127);
	
	// // CLEAR OPPOSITE PARK
	// chassis.pid_odom_ptp_set({{62_in,-24_in},fwd,90},true);
	// intake.set(Intake::INTAKE,127);
	// chassis.pid_wait();
	// chassis.pid_turn_set(8,127);
	// chassis.pid_wait_until(80_deg);
	// loaderPiston.set(true);
	// chassis.pid_wait();
	
	// chassis.pid_drive_set(60_in,75,false,false);
	// chassis.pid_wait_until(20_in);
	// loaderPiston.set(false);
	// chassis.pid_wait();
	// chassis.pid_drive_set(-10_in,80,true,false);
	// chassis.pid_wait_quick_chain();

	chassis.pid_drive_set(5_in,80,true,false);
	chassis.pid_wait_quick();
	chassis.pid_turn_set(0,127);
	chassis.pid_wait();
	// x = localizer.get_localized_coordinate(Localizer::Corner::TL);
	// y = (16)*1_in;
	// chassis.odom_xyt_set(x,y, 0_deg);

	//Going to second matchloader
	chassis.pid_odom_ptp_set({{40_in,44.5_in},fwd,90},true);
	chassis.pid_wait();
	chassis.pid_turn_set(90,127);
	chassis.pid_wait();
	loaderPiston.set(true);
	pros::delay(300);
	// chassis.pid_drive_set(-12,100,true);
	// chassis.pid_wait_until(-6_in);
	// intake.set(Intake::SCORE,127);
	// chassis.pid_wait_quick_chain();
	// pros::delay(500);
	chassis.pid_drive_set(23_in,50,true,false); //18.5. 60
	chassis.pid_wait();
	intake.set(Intake::INTAKE,127);


	//GETTING BALLS FROM 2nd MATCHLOADER 
	
	// chassis.pid_drive_set(31.5_in,50,true,false);
	// loaderPiston.set(true);
	// chassis.pid_wait();
	x = (71 - 14.3125)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::TR);
	chassis.odom_xy_set(x,y);
	pros::delay(1500);
	chassis.pid_drive_set(-5,90,true);
	chassis.pid_wait_quick_chain();
	chassis.pid_turn_set(-135,127);
	chassis.pid_wait();
	chassis.pid_drive_set(-16_in,90,true);
	chassis.pid_wait_quick_chain();
	loaderPiston.set(false);
	// chassis.pid_drive_set(-5_in,90,true);
	// chassis.pid_wait_quick();
	// chassis.pid_swing_set(ez::RIGHT_SWING,-90_deg,90,0,cw,true);
	// chassis.pid_turn_set(-90,127);
	// chassis.pid_wait();

	//MOVE TO OTHER SIDE TO SCORE
	chassis.pid_turn_set({-18_in,59_in},fwd,120);
	chassis.pid_wait();
	chassis.pid_odom_ptp_set({{-18_in,59.5_in},fwd,90},true);
	chassis.pid_wait();
	chassis.pid_odom_ptp_set({{-42_in,47.2_in},fwd,75},true); //-42,47.4
	chassis.pid_wait();

	chassis.pid_turn_set(-90,127);
	chassis.pid_wait();
	//SCORING
	chassis.pid_drive_set(-25.5_in,70,true);
	chassis.pid_wait_until(-19_in);
	intake.set(Intake::SCORE,127,100);
	chassis.pid_wait_quick_chain();
	pros::delay(1000);

	//INTAKE NEXT 3rd MATCHLOADER
	chassis.pid_turn_set(-89,127);
	chassis.pid_wait();
	intake.set(Intake::INTAKE,127);
	chassis.pid_drive_set(32.3_in,60,true,true); //32.3
	loaderPiston.set(true);
	chassis.pid_wait();
	// x = (-71 + 14.3125)*1_in;
	// y = localizer.get_localized_coordinate(Localizer::Corner::TL);
	// chassis.odom_xy_set(x,y);
	pros::delay(1000);
	chassis.pid_drive_set(-30_in,70,true);
	chassis.pid_wait_quick_chain();
	intake.set(Intake::SCORE,110,127); 
	pros::delay(1500);
	loaderPiston.set(false);
	intake.set(Intake::INTAKE,127);


	//GO TO FINAL MATCHLOADER
	chassis.pid_drive_set(7_in,80,true,false);
	chassis.pid_wait_quick();
	chassis.pid_targets_reset();
	chassis.pid_turn_set(180,127);
	chassis.pid_wait();
	chassis.pid_drive_set(45_in,100,false,false);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-41_in,-48.8_in},fwd,70},true);
	chassis.pid_wait();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait();

	intake.set(Intake::INTAKE,127);
	loaderPiston.set(true);
	pros::delay(300);
	chassis.pid_drive_set(15_in,60,false);
	chassis.pid_wait();
	pros::delay(1000);


	//SCORING FINAL MATCHLOADER
	chassis.pid_drive_set(-30_in,90,true);
	chassis.pid_wait_quick_chain();
	intake.set(Intake::SCORE,127);
	pros::delay(1500); 
	
	loaderPiston.set(false);
	chassis.pid_drive_set(10_in,80,true,false);
	chassis.pid_wait();
	chassis.pid_turn_set(-150,127);
	chassis.pid_wait();
	chassis.pid_drive_set(-12.5_in,80,true);
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait();
	flapperPiston.set(false);
	chassis.pid_drive_set(-17_in,40,true);
	chassis.pid_wait_quick();

	//PARK
	chassis.pid_odom_ptp_set({{-53.5_in,-23_in},fwd,90},true);
	chassis.pid_wait();
	chassis.pid_swing_set(ez::LEFT_SWING,-5,127,-5);
	chassis.pid_wait();
	chassis.pid_drive_set(32_in,100,false,false);
	chassis.pid_wait();
	intake.set(Intake::INTAKE);
	chassis.pid_turn_set(30,127);
	chassis.pid_wait_quick_chain();
	chassis.pid_turn_set(0,127);
	chassis.pid_wait_quick_chain();


	chassis.pid_drive_set(-5,80,true);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(8,80,true,false);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(-5,80,true);
	chassis.pid_wait_quick();




}

void skills(){
	okapi::QLength x;
	okapi::QLength y;
	flapperPiston.set(true);
	
	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(-45_in, 6_in, 90_deg);

	//Score two balls in middle for redundancy
	chassis.pid_odom_ptp_set({{-24,17},fwd,70}, true);
	chassis.pid_wait_quick();
	intake.set(Intake::IntakeState::STOP);
	chassis.pid_swing_set(ez::RIGHT_SWING,-45,90,-5);
	liftPiston.set(true);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(-18_in, 80, true);
	chassis.pid_wait_until(-4_in);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_wait_quick();
	intake.set(Intake::IntakeState::SCORE,50,100);
	pros::delay(750);


	// //First Matchloader
	// intake.set(Intake::IntakeState::INTAKE, 127);
	// liftPiston.set(false);
	// chassis.pid_odom_ptp_set({{-46,46},fwd,120}, true);
	// chassis.pid_wait_quick_chain();
	// chassis.pid_turn_set(-90,127);
	// loaderPiston.set(true);
	// chassis.pid_wait();
	// chassis.pid_drive_set(14_in, 80, false, true);
	// chassis.pid_wait();
	// x = (-71 + 14.3125)*1_in;
	// y = localizer.get_localized_coordinate(Localizer::Corner::TL);
	// chassis.odom_xy_set(x,y);
	// pros::delay(1000);


	// //Go to other side to score
	// chassis.pid_odom_ptp_set({{-29,58},rev,90}, true);
	// chassis.pid_wait_quick_chain();
	// chassis.pid_odom_ptp_set({{32,58},rev,90}, true);
	// chassis.pid_wait_quick_chain(6);
	// chassis.pid_odom_ptp_set({{38,46},rev,75}, true);
	// chassis.pid_wait();
	// chassis.pid_turn_set(90,127);
	// chassis.pid_wait();


	// //Scoring first matchloader
	// chassis.pid_drive_set(-10_in, 70, true);
	// chassis.pid_wait_until(-6_in);
	// intake.set(Intake::IntakeState::SCORE,127);
	// pros::delay(1000);


	// //Get second matchloader and score
	// chassis.pid_drive_set(32.3_in,60,true,true);
	// chassis.pid_wait();
	// x = (71 - 14.3125)*1_in;
	// y = localizer.get_localized_coordinate(Localizer::Corner::TR);
	// chassis.odom_xy_set(x,y);
	// pros::delay(1000);
	// chassis.pid_drive_set(-30_in,70,true);
	// chassis.pid_wait_quick_chain();
	// intake.set(Intake::SCORE,110,127); 
	// pros::delay(1500);
	// loaderPiston.set(false);
	// intake.set(Intake::INTAKE,127);

	// //Moving To Park
	// chassis.pid_odom_ptp_set({{49,39},fwd,80}, true);
	// chassis.pid_wait_quick_chain(4);
	// chassis.pid_odom_ptp_set({{61,29},fwd,80}, true);
	// chassis.pid_odom_ptp_set({{63,14.5},fwd,80}, true);
	// chassis.pid_wait();
	// chassis.pid_turn_set(180,127);
	// chassis.pid_wait();
	// //Clearing Park
	// chassis.pid_drive_set(40_in,75,false,false);
	// chassis.pid_wait_quick();

	// chassis.pid_drive_set(-10_in,100,false,false);
	// chassis.pid_wait_quick_chain();

	// x = localizer.get_localized_coordinate(Localizer::Corner::TR);
	// y = (-16)*1_in;
	// chassis.odom_xy_set(x,y);
	

	// //get one extra ball in addition to park balls for redundancy
	// chassis.pid_drive_set(7,127,false,false);
	// chassis.pid_wait_quick_chain();
	// chassis.pid_odom_ptp_set({{25,-20},fwd,90}, true);
	// chassis.pid_wait();
	// intake.set(Intake::STOP);

	// //score 7 of same color in middle
	// chassis.pid_turn_set(135,127);
	// liftPiston.set(true);
	// chassis.pid_wait_quick();
	// chassis.pid_drive_set(-11_in, 80, true);
	// intake.set(Intake::SCORE,50,100);
	// pros::delay(2000);

	// //Intake Third Matchloader
	// intake.set(Intake::IntakeState::INTAKE,127);
	// chassis.pid_odom_ptp_set({{46_in,-44.7_in},fwd,80},true); //46,46
	// liftPiston.set(false);
	// chassis.pid_wait_quick_chain();

	// chassis.pid_turn_set(88,127);
	// loaderPiston.set(true);
	// chassis.pid_wait_quick();
	// chassis.pid_drive_set(14.7_in,60,true);
	// chassis.pid_wait();
	// pros::delay(500);
	// //localizing
	// x = (71 - 14.3125)*1_in;
	// y = localizer.get_localized_coordinate(Localizer::Corner::BR);
	// chassis.odom_xy_set(x,y);


	// //Go to other side to score
	// chassis.pid_odom_ptp_set({{29,-58},rev,90}, true);
	// chassis.pid_wait_quick_chain();
	// chassis.pid_odom_ptp_set({{-32,-58},rev,90}, true);
	// chassis.pid_wait_quick_chain(6);
	// chassis.pid_odom_ptp_set({{-38,-46},rev,75}, true);
	// chassis.pid_wait();
	// chassis.pid_turn_set(-90,127);
	// chassis.pid_wait();

	// //Scoring third matchloader
	// chassis.pid_drive_set(-14_in, 70, false,false);
	// chassis.pid_wait_until(-8_in);
	// intake.set(Intake::IntakeState::SCORE,127);
	// pros::delay(1500);
	// intake.set(Intake::IntakeState::INTAKE,127);
	// loaderPiston.set(false);

	// //Intake Final Matchloader
	// chassis.pid_turn_set(-90,127);
	// chassis.pid_wait();
	// chassis.pid_drive_set(32.3_in,60,true,true);
	// chassis.pid_wait();
	// x = (-71 + 14.3125)*1_in;
	// y = localizer.get_localized_coordinate(Localizer::Corner::BL);
	// chassis.odom_xy_set(x,y);
	// pros::delay(1000);
	// chassis.pid_drive_set(-30_in,70,true);
	// chassis.pid_wait_quick_chain();
	// intake.set(Intake::SCORE,110,127);
	// pros::delay(1000);
	// loaderPiston.set(false);
	// pros::delay(500);


	// //Go to Park
	// chassis.pid_odom_ptp_set({{-49,-39},fwd,80}, true);
	// chassis.pid_wait_quick_chain(4);
	// chassis.pid_odom_ptp_set({{-61,-29},fwd,80}, true);
	// chassis.pid_odom_ptp_set({{-63,-14.5},fwd,80}, true);
	// chassis.pid_wait();
	// chassis.pid_turn_set(0,127);
	// chassis.pid_wait();

	// //Clearing Park
	// chassis.pid_drive_set(32_in,100,false,false);
	// chassis.pid_wait();
	// intake.set(Intake::INTAKE,127);
	// chassis.pid_turn_set(30,127);
	// chassis.pid_wait_quick_chain();
	// chassis.pid_turn_set(0,127);
	// chassis.pid_wait_quick_chain();


	// chassis.pid_drive_set(-5,80,true);
	// chassis.pid_wait_quick();
	// chassis.pid_drive_set(8,80,true,false);
	// chassis.pid_wait_quick();
	// chassis.pid_drive_set(-5,80,true);
	// chassis.pid_wait_quick();


	
}