#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "devices.hpp"
#include "main.h"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/localizer.hpp"
#include <iostream>
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;


// wait until the imu pitch is raised above a certain threshold and then back down
void wait_for_imu_bump(double threshold) {
  while((chassis.imu.get_pitch()) > threshold) {
	pros::delay(10);
  }
  pros::delay(200);
}

void raw_turn(int theta, int speed){
	int error = theta - chassis.odom_theta_get();
	int counter = 0;
	while(std::abs(error) > 1){
		error = theta - chassis.odom_theta_get();
		chassis.drive_set(ez::util::sgn(error)*speed, -ez::util::sgn(error)*speed);
		counter+=1/10;
		pros::delay(10);
		if (counter>400){
			break;
		}
	}
	chassis.drive_set(0,0);
}

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
void solo_awp(){
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 150_ms,
										 250_ms);

	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-46.5_in,-6_in,0_deg);
	intake.set(Intake::IntakeState::INTAKE,127);

	chassis.pid_drive_set(7_in,127,false, false);
	chassis.pid_wait_quick();
	chassis.pid_odom_ptp_set({{-45_in,-46_in},rev,80},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(16_in,55,false, true, -90);
	chassis.pid_wait_quick();
	pros::delay(400);


    // score first batch
	// chassis.pid_drive_set(-27_in,127,false, true, -90);
	chassis.pid_odom_ptp_set({{-27_in,-48.5_in},rev,80},false);
	chassis.pid_wait_until(-22.7_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::AUTON_SCORING);
	pros::delay(500);
    intake.set(Intake::IntakeState::INTAKE,127);

    // get mid balls 
	chassis.pid_drive_set(8,80,false,false);
	chassis.pid_wait_quick();
	chassis.pid_turn_set({-25,-29},fwd,90);
	chassis.pid_wait_quick();
    chassis.pid_odom_ptp_set({{-29_in,-29_in},fwd,60},false); 
	chassis.pid_wait_until({-26_in,-28_in});
    // loaderPiston.set(true);
    chassis.pid_wait_quick_chain(6);
	loaderPiston.set(false);
    chassis.pid_odom_ptp_set({{-24_in,17.5_in},fwd,60},false);
	chassis.pid_wait_until(28_in);
    loaderPiston.set(true);
    chassis.pid_wait_quick();

	
    // // score mid balls
    // chassis.pid_swing_set(ez::LEFT_SWING, -45_deg, 120, 0, false);
	
    // chassis.pid_wait_quick_chain();
    // chassis.pid_drive_set(-12_in, 80,true);
    // liftPiston.set(true);
    // intake.set(Intake::IntakeState::OUTTAKE);
	// pros::delay(200);
	// intake.set(Intake::INTAKE);

    // chassis.pid_wait_until(-5_in);
    // intake.set(Intake::IntakeState::AUTON_SCORING, 65, 100);

    // pros::delay(1000);

    // loaderPiston.set(false);

	//score second long goal
    // liftPiston.set(false);
    intake.set(Intake::IntakeState::INTAKE, 127);
    chassis.pid_odom_ptp_set({{-40_in,41_in},fwd,85},true); //-40,45.2
    chassis.pid_wait_quick();
    chassis.pid_turn_set(-90_deg,90);
    chassis.pid_wait_quick_chain(5);
    loaderPiston.set(true);
    pros::delay(400);
    chassis.pid_drive_set(-21_in,100,true, true, -90);
    
    chassis.pid_wait_until(-14_in);
	// chassis.pid_odom_ptp_set({{-20,44.5},rev,90},true);
	// chassis.pid_wait_until(-23_in);  
    intake.set(Intake::IntakeState::AUTON_SCORING);
	chassis.pid_wait_quick();
	pros::delay(200);
	intake.set(Intake::IntakeState::INTAKE);
	chassis.pid_drive_set(30,45,true, true, -90);
	chassis.pid_wait_quick();
	

	//Score middle goal
	chassis.pid_drive_set(-7,120,true);
	chassis.pid_wait_quick_chain();
	loaderPiston.set(false);
	chassis.pid_turn_set({-13,6},rev,90);
	chassis.pid_wait_quick_chain();
	liftPiston.set(true);
	leverProfiler.setProfile({{-10, 60}, {50, 50}});
	chassis.pid_odom_ptp_set({{-14,10},rev,90},true,15,50);
	chassis.pid_wait_until({-15,15});
	
	intake.set(Intake::IntakeState::AUTON_SCORING, 90);
	chassis.pid_wait_quick();
	pros::delay(500);
	hoodPiston.set(false);
}
void four_goal_solo_awp() {
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 80_ms,
										 250_ms);
    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-19_in,180_deg);

	chassis.pid_odom_ptp_set({{-45_in,-46_in},fwd,80},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(5_in,127,false, true, -90); 
	chassis.pid_wait_quick();


    // score first batch
	// chassis.pid_drive_set(-27_in,127,false, true, -90);
	chassis.pid_odom_ptp_set({{-27_in,-49.5_in},rev,80},false);
	chassis.pid_wait_until(-22.7_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::AUTON_SCORING);
	pros::delay(500);
    intake.set(Intake::IntakeState::INTAKE);

    // get mid balls 
	chassis.pid_drive_set(8,80,false,false);
	chassis.pid_wait_quick();
    // chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 100, 7, false); //59
    // chassis.pid_wait_quick_chain();
	chassis.pid_turn_set({-10.5,-11},fwd,90);
	chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-8.5,-9},fwd,40},false);
	chassis.pid_wait();
	intake.set(Intake::IntakeState::OUTTAKE,80);
	std::cout<<"start delay"<<std::endl;
	pros::delay(1000);
	std::cout<<"end delay"<<std::endl;
    
	chassis.pid_swing_set(ez::LEFT_SWING,-10_deg, 100, 10, false);
	chassis.pid_wait_quick_chain();
	return;
	
	//scoring second long
    intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_odom_ptp_set({{-22,28},fwd,90}, true);
	chassis.pid_wait_quick_chain();

    chassis.pid_odom_ptp_set({{-40_in,42_in},fwd,85},true); //-40,45.2
    chassis.pid_wait_quick();
    chassis.pid_turn_set(-90_deg,90);
    chassis.pid_wait_quick_chain(5);
    loaderPiston.set(true);
    pros::delay(400);
    chassis.pid_drive_set(-21_in,100,true, true, -90);
    
    chassis.pid_wait_until(-14_in);
	// chassis.pid_odom_ptp_set({{-20,44.5},rev,90},true);
	// chassis.pid_wait_until(-23_in);  
    intake.set(Intake::IntakeState::AUTON_SCORING);
	chassis.pid_wait_quick();
	pros::delay(200);
	intake.set(Intake::IntakeState::INTAKE);
	chassis.pid_drive_set(30,45,true, true, -90);
	chassis.pid_wait_quick();
	

	//Score middle goal
	chassis.pid_drive_set(-7,120,true);
	chassis.pid_wait_quick_chain();
	loaderPiston.set(false);
	chassis.pid_turn_set({-13,6},rev,90);
	chassis.pid_wait_quick_chain();
	liftPiston.set(true);
	chassis.pid_odom_ptp_set({{-16,8},rev,90},true,15,50);
	chassis.pid_wait_until({-17,15});
	
	intake.set(Intake::IntakeState::AUTON_SCORING, 90);
	chassis.pid_wait_quick();

}

void four_goal_solo_awp_push() {
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 80_ms,
										 250_ms);

	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-46.5_in,-6_in,0_deg);
	intake.set(Intake::IntakeState::INTAKE,127);

	chassis.pid_drive_set(7_in,127,false, false);
	chassis.pid_wait_quick();
	chassis.pid_odom_ptp_set({{-45_in,-41.5_in},rev,100},false); //-46,48
	chassis.pid_wait_quick_chain(3);
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(15_in,127,false, true, -90); 
	chassis.pid_wait_quick();


    // score first batch
	// chassis.pid_drive_set(-27_in,127,false, true, -90);
	chassis.pid_odom_ptp_set({{-27_in,-48_in},rev,80},false);
	chassis.pid_wait_until(-22.7_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::AUTON_SCORING);
	pros::delay(500);
    intake.set(Intake::IntakeState::INTAKE,127);

    // get mid balls 
	chassis.pid_drive_set(13,80,false,false);
	chassis.pid_wait_quick();
    // chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 100, 7, false); //59
    // chassis.pid_wait_quick_chain();
	chassis.pid_turn_set({-20,-20},fwd,90);
	chassis.pid_wait_quick();
	chassis.pid_odom_ptp_set({{-20,-20},fwd,80},false);
	chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-16,-17},fwd,60},false);
	chassis.pid_wait_quick();
	intake.set(Intake::IntakeState::OUTTAKE,60);
	std::cout<<"start delay"<<std::endl;
	pros::delay(1000);
	std::cout<<"end delay"<<std::endl;
    
	chassis.pid_swing_set(ez::LEFT_SWING,0, 100, 20, false);
	// chassis.pid_turn_set({-20,-15},fwd,90);
	chassis.pid_wait_quick_chain();
	
    intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_odom_ptp_set({{-21_in,14_in},fwd,80},false);
	chassis.pid_wait_until(28_in);
    // loaderPiston.set(true);
    chassis.pid_wait_quick_chain();

	//score second long goal
    chassis.pid_odom_ptp_set({{-40_in,53_in},fwd,85},true); //-40,45.2
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-90_deg,90);
    chassis.pid_wait_quick_chain(5);
    loaderPiston.set(true);
    pros::delay(400);
    chassis.pid_drive_set(-15_in,100,false,false);
	pros::delay(200);
	// chassis.pid_odom_ptp_set({{-20,44.5},rev,90},true);
	// chassis.pid_wait_until(-23_in);  
    intake.set(Intake::IntakeState::AUTON_SCORING);
	pros::delay(200);
	chassis.pid_wait_quick();
	intake.set(Intake::IntakeState::INTAKE);
	chassis.pid_drive_set(25,45,true, true, -90);
	chassis.pid_wait_quick();
	pros::delay(400);
	

	//Score middle goal
	chassis.pid_drive_set(-6,120,true);
	chassis.pid_wait_quick_chain();
	loaderPiston.set(false);
	chassis.pid_turn_set({-13,6},rev,90);
	chassis.pid_wait_quick_chain();
	liftPiston.set(true);
	chassis.pid_odom_ptp_set({{-13,9.5},rev,90},true,15,50);
	chassis.pid_wait_until({-18,18});
	leverProfiler.setProfile({{-10,80}});
	intake.set(Intake::AUTON_SCORING_HOLD);
	chassis.pid_wait_quick();

}

void old_solo_awp() {
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 100_ms, 250_ms);
    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-18_in,180_deg);

	chassis.pid_odom_ptp_set({{-46_in,-47_in},fwd,90},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(11_in,127,false, true, -90); 
	chassis.pid_wait_quick();

    // score first batch
	chassis.pid_drive_set(-28_in,80,true, true, -90);
	chassis.pid_wait_until(-23_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::AUTON_SCORING);
    pros::delay(1100);
    intake.set(Intake::IntakeState::INTAKE);

    // get mid balls 
    chassis.pid_swing_set(ez::LEFT_SWING, 50_deg, 100, 5, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-19_in,-22_in},fwd,40},false); 

    
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_ptp_set({{-18.5_in,22.3_in},fwd,60},false); 
	chassis.pid_wait_until(24_in);
    loaderPiston.set(true);
    chassis.pid_wait_quick();

    // score mid balls
    chassis.pid_swing_set(ez::LEFT_SWING, -45_deg, 120, 0, false);
	
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-12_in, 80,true);
    liftPiston.set(true);
    intake.set(Intake::IntakeState::OUTTAKE);
	pros::delay(200);
	intake.set(Intake::INTAKE);

    chassis.pid_wait_until(-5_in);
    intake.set(Intake::IntakeState::AUTON_SCORING, 65, 100);

    pros::delay(1000);

    loaderPiston.set(false);

    // get second batch matchloads
    liftPiston.set(false);
    intake.set(Intake::IntakeState::INTAKE, 127);
    chassis.pid_odom_ptp_set({{-40_in,44.2_in},fwd,85},true); //-40,45.2
    chassis.pid_wait_quick();
    chassis.pid_turn_set(-90_deg,127);
    chassis.pid_wait_quick_chain();

    loaderPiston.set(true);
    pros::delay(400);
    chassis.pid_drive_set(13.7_in,100,false, true, -90);
    chassis.pid_wait_quick();

    // score second batch
    chassis.pid_drive_set(-28_in,80,true, true, -90);
    
    chassis.pid_wait_until(-23_in);
	// chassis.pid_odom_ptp_set({{-20,44.5},rev,90},true);
	// chassis.pid_wait_until(-23_in);
    loaderPiston.set(false);
    
    intake.set(Intake::IntakeState::AUTON_SCORING);
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
	intake.set(Intake::IntakeState::AUTON_SCORING, 60,127);
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
	
	intake.set(Intake::IntakeState::AUTON_SCORING, 127);
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

void left_side_9_ball(){
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 85_ms,
										 250_ms);
	flapperPiston.set(true);
	
	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(-46.5_in, 6_in, 90_deg);
	chassis.pid_drive_set(6,90,30);
	chassis.pid_wait_quick_chain(); 

	chassis.pid_turn_set({-8,30},fwd,90);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-9 ,35},fwd,70},true); //-9.5,35
	// chassis.pid_wait_until(30);
	// loaderPiston.set(true);
	chassis.pid_wait_quick();
	chassis.pid_swing_set(ez::RIGHT_SWING, -3_deg, 40, -4, true);
	pros::delay(400);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(2.5_in, 80, true);
	chassis.pid_wait_quick_chain();
	pros::delay(500);
	chassis.pid_drive_set(-5_in, 80, true);
	loaderPiston.set(false);
	chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 80, 0, true);
	chassis.pid_wait_quick();
	chassis.pid_odom_ptp_set({{-27,36.3},rev,90},true);
	chassis.pid_wait_quick();
	chassis.pid_swing_set(ez::RIGHT_SWING,-90_deg,80,8,cw,true);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(-7_in, 80, true);
	intake.set(Intake::IntakeState::AUTON_SCORING, 127);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain();
	// pros::delay(100);
	intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_drive_set(29_in, 60, true,true,-90);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(-7_in, 80,30, true);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-14.5 ,7},rev,90},true);
	liftPiston.set(true);
	chassis.pid_wait_until(10);
	loaderPiston.set(false);
	chassis.pid_wait_quick();
	intake.set(Intake::IntakeState::AUTON_SCORING, 127);
	pros::delay(1000);
	chassis.pid_odom_ptp_set({{-14,29.5},fwd,90},true);
	liftPiston.set(false);
	intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_wait_quick();
	flapperPiston.set(false);
	chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 90, 0, true);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(5_in, 80, true);
	chassis.pid_wait_quick();
	chassis.pid_turn_set(95, 80);
}

void left_7_rush(){
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 60_ms,
										 100_ms);
	flapperPiston.set(true);
	
	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(-46.5_in, 6_in, 90_deg);
	chassis.pid_odom_ptp_set({{-29,17},fwd,127},true); //-9.5,35
	chassis.pid_wait_until(10);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain(3);
	// chassis.pid_swing_set(ez::RIGHT_SWING, -20_deg, 100, 5, true);
	chassis.pid_turn_set(0,100);
	chassis.pid_wait_quick_chain(20);

	chassis.pid_odom_ptp_set({{-42,41.7},fwd,127},true); //-9.5,35
	loaderPiston.set(false);
	chassis.pid_wait_quick_chain(5);
	chassis.pid_turn_set(-90,100);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(11.5_in,127,false, true, -90);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-27.5,48.5},rev,127},true);
	chassis.pid_wait_until({-36,48});
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
	chassis.pid_wait_quick_chain(4);
	loaderPiston.set(false);
	pros::delay(500);
	
	// chassis.pid_drive_set(1,127,50,false);
	// chassis.pid_wait_quick_chain(3);
	// chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 250_ms,
	// 									 250_ms);
	// chassis.pid_swing_set(ez::RIGHT_SWING,90_deg,127,-20,ccw,false);
	// // chassis.pid_swing_set(ez::RIGHT_SWING, 125, 120, -95, ccw,false);  //ANGLE ADJUSTED TO CHAIN. NOT ACTUALLY 130 DEGREES
	// chassis.pid_wait_quick();
	// chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 500_ms,
	// 									 500_ms);
	// chassis.pid_drive_set(28,127,false, false);
	// flapperPiston.set(false);
	// chassis.pid_wait_quick();
	// chassis.pid_turn_set(70,127);

	chassis.pid_swing_set(ez::LEFT_SWING, -23_deg, 110, 5, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::RIGHT_SWING, -88_deg, 100, false);
    chassis.pid_wait_quick();
	
	// chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 15000_ms,
										//  15000_ms);
    flapperPiston.set(false);
    chassis.pid_drive_set(-16_in,127,80,false); //85
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-10_in,49.5_in},rev,127,127},false);
	chassis.pid_wait_quick();
	// chassis.pid_swing_exit_condition_set(10000_ms, 1_deg, 10000_ms, 7_deg, 15000_ms,
	// 									 15000_ms);
	intake.set(Intake::IntakeState::STOP);
	chassis.pid_swing_set(ez::RIGHT_SWING,-140,127,-5,true);
}

void right_4_rush(){
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 60_ms,
										 100_ms);
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.odom_xyt_set(-56.5_in, -17_in, 90_deg);

	leverProfiler.setProfile({{-10,120},{60,90}});

	chassis.pid_odom_ptp_set({{-28,-24},fwd,100},true);
	chassis.pid_wait_until(10);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain(3);
	chassis.pid_turn_set({-50,-52},rev,90);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(-12_in,100,80,false,false);
	chassis.pid_wait_quick_chain(4);

	chassis.pid_swing_exit_condition_set(50_ms, 2_deg, 50_ms, 7_deg, 50_ms,
										 50_ms);
	chassis.pid_swing_set(ez::LEFT_SWING,-90_deg,100,10,true);
	chassis.pid_wait_quick();
	chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 500_ms,
										 500_ms);
	loaderPiston.set(false);

	intake.set(Intake::IntakeState::SCORE,127);
	pros::delay(400);
	chassis.pid_swing_set(ez::LEFT_SWING, -24, 100, -20, true);
	chassis.pid_wait_quick();
	chassis.pid_swing_set(ez::RIGHT_SWING, -90, 100, 0, true);
	chassis.pid_wait_quick();

	chassis.pid_drive_set(-25_in, 100, 80, false, false);
	flapperPiston.set(false);
	chassis.pid_wait_quick();       //REMOVE PLS
	
	chassis.pid_swing_set(ez::RIGHT_SWING,140,127,-5,true);
	// chassis.pid_turn_set({-22,-37},rev,100);
	// chassis.pid_wait_quick_chain();
	// chassis.pid_odom_ptp_set({{-22,-37},rev,100},true);
	// chassis.pid_wait_quick_chain();
	// chassis.pid_odom_ptp_set({{-9,-38},rev,100},true);
	



}


void left_4_rush(){
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 60_ms,
										 100_ms);
	flapperPiston.set(true);
	
	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(-46.5_in, -6_in, 90_deg);
	chassis.pid_odom_ptp_set({{-29,-17},fwd,127},true); //-9.5,35
	chassis.pid_wait_until(10);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain(3);
	// chassis.pid_swing_set(ez::RIGHT_SWING, -20_deg, 100, 5, true);
	chassis.pid_turn_set(180,100);
	chassis.pid_wait_quick_chain(20);

	chassis.pid_odom_ptp_set({{-42,-41.4},fwd,127},true); //-9.5,35
	loaderPiston.set(false);
	chassis.pid_wait_quick_chain(5);
	chassis.pid_turn_set(-90,100);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-27.5,-48},rev,127},true);
	chassis.pid_wait_until({-36,-48});
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
	chassis.pid_wait_quick_chain(4);
	
	// chassis.pid_drive_set(1,127,50,false);
	// chassis.pid_wait_quick_chain(3);
	// chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 250_ms,
	// 									 250_ms);
	// chassis.pid_swing_set(ez::RIGHT_SWING,90_deg,127,-20,ccw,false);
	// // chassis.pid_swing_set(ez::RIGHT_SWING, 125, 120, -95, ccw,false);  //ANGLE ADJUSTED TO CHAIN. NOT ACTUALLY 130 DEGREES
	// chassis.pid_wait_quick();
	// chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 500_ms,
	// 									 500_ms);
	// chassis.pid_drive_set(28,127,false, false);
	// flapperPiston.set(false);
	// chassis.pid_wait_quick();
	// chassis.pid_turn_set(70,127);

	chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 110, 5, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::RIGHT_SWING, -87_deg, 100, false);
    chassis.pid_wait_quick();
	
	// chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 15000_ms,
										//  15000_ms);
    flapperPiston.set(false);
    chassis.pid_drive_set(-16_in,127,80,false); //85
	chassis.pid_wait_quick_chain();
	// chassis.pid_odom_ptp_set({{-10_in,-49.5_in},rev,127,127},false);
	// chassis.pid_wait_quick();
	// chassis.pid_swing_exit_condition_set(10000_ms, 1_deg, 10000_ms, 7_deg, 15000_ms,
	// 									 15000_ms);
	intake.set(Intake::IntakeState::STOP);
	chassis.pid_swing_set(ez::RIGHT_SWING,-140,127,-5,true);
}

void right_7_rush(){
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 60_ms,
										 100_ms);
	flapperPiston.set(true);
	
	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(-46.5_in, -6_in, 90_deg);
	chassis.pid_odom_ptp_set({{-29,-17},fwd,127},true); //-9.5,35
	chassis.pid_wait_until(10);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain(3);
	// chassis.pid_swing_set(ez::RIGHT_SWING, -20_deg, 100, 5, true);
	chassis.pid_turn_set(180,100);
	chassis.pid_wait_quick_chain(20);

	chassis.pid_odom_ptp_set({{-42,-39},fwd,127},true); //-9.5,35
	loaderPiston.set(false);
	chassis.pid_wait_quick_chain(5);
	chassis.pid_turn_set(-90,100);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(11.5_in,127,false, true, -90);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-27.5,-46},rev,127},true);
	chassis.pid_wait_until({-36,-48});
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
	chassis.pid_wait_quick_chain(4);
	loaderPiston.set(false);
	pros::delay(500);
	
	// chassis.pid_drive_set(1,127,50,false);
	// chassis.pid_wait_quick_chain(3);
	// chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 250_ms,
	// 									 250_ms);
	// chassis.pid_swing_set(ez::RIGHT_SWING,90_deg,127,-20,ccw,false);
	// // chassis.pid_swing_set(ez::RIGHT_SWING, 125, 120, -95, ccw,false);  //ANGLE ADJUSTED TO CHAIN. NOT ACTUALLY 130 DEGREES
	// chassis.pid_wait_quick();
	// chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 500_ms,
	// 									 500_ms);
	// chassis.pid_drive_set(28,127,false, false);
	// flapperPiston.set(false);
	// chassis.pid_wait_quick();
	// chassis.pid_turn_set(70,127);

	chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 110, 5, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::RIGHT_SWING, -88_deg, 100, false);
    chassis.pid_wait_quick();
	
	// chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 15000_ms,
										//  15000_ms);
    flapperPiston.set(false);
    chassis.pid_drive_set(-16_in,127,80,false); //85
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-15_in,-49.5_in},rev,127,127},false);
	chassis.pid_wait_quick();
	// chassis.pid_swing_exit_condition_set(10000_ms, 1_deg, 10000_ms, 7_deg, 15000_ms,
	// 									 15000_ms);
	intake.set(Intake::IntakeState::STOP);
	chassis.pid_swing_set(ez::RIGHT_SWING,-140,127,-5,true);
}

void move_forward(){
	chassis.pid_drive_set(5,50);
}


void right_elims() {
    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-18_in,180_deg);

	chassis.pid_odom_ptp_set({{-46_in,-45.4_in},fwd,127},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(11.5_in,127,false, true, -90); 
	chassis.pid_wait_quick();


    // score first batch
	// chassis.pid_drive_set(-27_in,127,false, true, -90);
	chassis.pid_odom_ptp_set({{-27_in,-48.5_in},rev,127},false);
	chassis.pid_wait_until(-22.7_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::AUTON_SCORING);
    pros::delay(1200);
    intake.set(Intake::IntakeState::INTAKE);

    // chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 100, 6, false);
	chassis.pid_swing_set(ez::LEFT_SWING, -23_deg, 110, 5, false);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::RIGHT_SWING, -88_deg, 100, false);
    chassis.pid_wait_quick();
	
	// chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 15000_ms,
										//  15000_ms);
    flapperPiston.set(false);
    chassis.pid_drive_set(-16_in,117,80,false); //85
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-8_in,-49.5_in},rev,127},false);
	chassis.pid_wait_quick();
	// chassis.pid_swing_exit_condition_set(10000_ms, 1_deg, 10000_ms, 7_deg, 15000_ms,
	// 									 15000_ms);
	intake.set(Intake::IntakeState::STOP);
	chassis.pid_swing_set(ez::RIGHT_SWING,-140,127,-5,true);

	



}

void right_elims_mid_ball() {
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 80_ms,
										 250_ms);
    // get matchloads
	flapperPiston.set(true);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.odom_xyt_set(-53_in,-18_in,180_deg);

	chassis.pid_odom_ptp_set({{-45_in,-46_in},fwd,127},false); //-46,48
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait_quick();
    
    loaderPiston.set(true);
    pros::delay(300);
	chassis.pid_drive_set(11.5_in,127,false, true, -90); 
	chassis.pid_wait_quick();


    // score first batch
	// chassis.pid_drive_set(-27_in,127,false, true, -90);
	chassis.pid_odom_ptp_set({{-27_in,-48.5_in},rev,127},false);
	chassis.pid_wait_until(-22.7_in);

    loaderPiston.set(false);
    intake.set(Intake::IntakeState::AUTON_SCORING);
    pros::delay(1200);
    intake.set(Intake::IntakeState::INTAKE);

    // get mid balls 

    chassis.pid_swing_set(ez::LEFT_SWING, 63.5_deg, 100, 7, false); //59
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(29_in,40,false); 
	chassis.pid_wait_quick();
	chassis.pid_wait_quick();
	chassis.pid_swing_set(ez::LEFT_SWING,28,127); //31.5
	chassis.pid_wait();
	liftPiston.set(true);
	chassis.pid_drive_set(5_in, 60, true);
	chassis.pid_wait_quick_chain();
	pros::delay(1000);
	intake.set(Intake::IntakeState::OUTTAKE,80);
	pros::delay(500);
	intake.set(Intake::IntakeState::INTAKE,127);
	pros::delay(500);
	intake.set(Intake::IntakeState::OUTTAKE,70);
	pros::delay(1000);
	liftPiston.set(false);
	pros::delay(800);
	
	chassis.pid_drive_set(-5,127);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-33_in,-31.5_in},rev,100},false);
	chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::LEFT_SWING,-80,127);
	chassis.pid_wait_quick();
	flapperPiston.set(false);
	chassis.pid_drive_set(-21,127,false);

	// chassis.pid_drive_set(-20_in,117,false); //85
	



}


void skills_83_route() {
	leverProfiler.setProfile({{-10, 127},{90,45},{100,40}});
    chassis.pid_odom_drive_exit_condition_set(100_ms, 1_in, 150_ms, 5_in, 250_ms, 500_ms);
	okapi::QLength x;
	okapi::QLength y;
	flapperPiston.set(true);
	
	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(-46.5_in, 6_in, 90_deg);
	// Score two balls in middle for redundancy
	chassis.pid_odom_ptp_set({{-30,22}, fwd,60}, true, 17, 25); //28,18
    chassis.pid_wait_until({-27_in, 17_in});
    pros::delay(500);
	chassis.pid_wait_quick();
    // chassis.pid_turn_set(-43, 100);
    // chassis.pid_wait_quick();
    // loaderPiston.set(false);

    // chassis.pid_drive_set(-20_in, 80, true, true, -45);
    // chassis.pid_wait_until(-16_in);
    
	// intake.set(Intake::IntakeState::AUTON_SCORING,90);
    // // intake.waitUntilColor(190, 260, 0.3, 0, 600);
	// pros::delay(200);
    // chassis.pid_wait_quick();
	// intake.set(Intake::IntakeState::INTAKE,127);


	// moving to score 4
	chassis.pid_turn_set({-36,44},fwd,70);
	chassis.pid_wait_quick();
	chassis.pid_odom_ptp_set({{-36,41.5},fwd,80}, true);  //-36,43.5
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait();

    //score 4
    chassis.pid_drive_set(-15_in, 80, true);
    chassis.pid_wait_until(-10_in);
    intake.set(Intake::IntakeState::AUTON_SCORING,127,80);
    chassis.pid_wait_quick();
	chassis.drive_set(-90,-90);
	pros::delay(200);
	chassis.drive_set(0,0);

	x = (-30)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::TL);
	chassis.odom_xy_set(x,y);
	std::cout<<chassis.odom_x_get()<<","<<chassis.odom_y_get()<<", "<<chassis.odom_theta_get()<<std::endl;
  
	// first matchloader
    loaderPiston.set(true);
    pros::delay(500);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_turn_set({-63,45},fwd,90);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(29.0_in,55, false, true);
	chassis.pid_wait_until(10);
	// liftPiston.set(true);
	hoodPiston.set(false);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_wait();
	pros::delay(200);
	chassis.pid_drive_set(-4_in, 80, true);
	chassis.pid_wait();
	chassis.pid_drive_set(4.7_in, 80, true);
	chassis.pid_wait_quick();
	pros::delay(1550);
	
	//Go to other side to score
	chassis.pid_drive_set(-4.5_in, 80,30, true);
	chassis.pid_wait_quick_chain();
	intake.set(Intake::OUTTAKE,127);
	pros::delay(100);
	intake.set(Intake::INTAKE,127);
	chassis.pid_turn_set({-36,60.5},rev,90);
	chassis.pid_wait_quick_chain();
	liftPiston.set(false);
	chassis.pid_odom_ptp_set({{-37,59.5},rev,80}, true);
	intake.set(Intake::STOP);
	pros::delay(100);
	intake.set(Intake::INTAKE,127);
	chassis.pid_wait_quick_chain(3);
    // chassis.pid_turn_set(-90,127);
    // chassis.pid_wait_quick_chain(10);
	chassis.pid_odom_ptp_set({{23,54.5},rev,80}, true);
    

    chassis.pid_wait_until(-43_in);
    loaderPiston.set(false);
	chassis.pid_wait_quick();
    // chassis.pid_turn_set({40_in,46_in},rev,90);
	chassis.pid_odom_ptp_set({{40,44},rev,75}, true);
    chassis.pid_wait_quick();
	chassis.pid_turn_set(90,90);
	chassis.pid_wait_quick();
    //wallreset
    x = chassis.odom_x_get()*1_in;
    y = localizer.get_localized_coordinate(Localizer::Corner::TR);
    chassis.odom_xy_set(x,y);

	//Scoring first matchloader
	chassis.pid_odom_ptp_set({{22,47.5},rev,90},true);
	chassis.pid_wait_until({26_in, 46_in});
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
    chassis.pid_wait_quick();
    chassis.drive_set(-90, -90);
    loaderPiston.set(true);
    pros::delay(600);
    chassis.drive_set(0, 0);
	pros::delay(200);
    intake.set(Intake::IntakeState::INTAKE,127);


	//Get second matchloader and score
	chassis.pid_turn_set({60,42.7},fwd,90);  //60,41.3
	chassis.pid_wait();
	chassis.pid_drive_set(29.2_in,55, false, true);
	chassis.pid_wait_until(4);
	// liftPiston.set(true);
	hoodPiston.set(false);
	chassis.pid_wait();
	pros::delay(500);
	chassis.pid_drive_set(-1.7_in, 80, true);
	chassis.pid_wait();
	chassis.pid_drive_set(2_in, 80, true);
	chassis.pid_wait_quick();
	pros::delay(1250);
	chassis.pid_odom_ptp_set({{22,46.5},rev,80},true);
	liftPiston.set(false);
	intake.set(Intake::OUTTAKE,127);
	pros::delay(100);
	intake.set(Intake::INTAKE,127);
	chassis.pid_wait_quick();
	pros::delay(700);
	leverProfiler.setProfile({{-10, 80},{50,40}});
	intake.set(Intake::AUTON_SCORING,127);
	chassis.pid_wait_quick();
	std::cout<<chassis.odom_x_get()<<","<<chassis.odom_y_get()<<", "<<chassis.odom_theta_get()<<std::endl;
    chassis.drive_set(-90, -90);
    pros::delay(700);
    chassis.drive_set(0, 0);
	pros::delay(300);
	loaderPiston.set(false);
	leverProfiler.setProfile({{-10, 127},{90,45},{100,40}});

	x = (30)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::TR);

	chassis.odom_xy_set(x,y);
	chassis.pid_drive_set(7,80,true,false);
	chassis.pid_wait_quick_chain();



	// Go to Park
	chassis.pid_odom_ptp_set({{49,41},fwd,80}, true);
	chassis.pid_wait_quick_chain(4);
	chassis.pid_odom_ptp_set({{60.4,21.5},fwd,80}, true); //61.5,21
	pros::delay(300);
	hoodPiston.set(false);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain(3);
	chassis.pid_swing_set(ez::LEFT_SWING, 169, 90,2);
	intake.set(Intake::OUTTAKE,127);
	// chassis.pid_turn_set(170,90);
	pros::delay(600);
	chassis.pid_wait_quick();
	//Clearing Park
	// chassis.pid_drive_set(32_in,80,false,false);
	chassis.drive_set(76,76);  //73,73
	pros::delay(400);
	loaderPiston.set(false);
	pros::delay(300);
    intake.set(Intake::INTAKE,127);
	pros::delay(000);
	chassis.drive_set(0,0);
	// chassis.pid_turn_exit_condition_set(90_ms, 1.75_deg, 100_ms, 5_deg, 100_ms,
	// 									100_ms);
	// chassis.pid_turn_set(178,90);
	// chassis.pid_wait_quick();
	// chassis.pid_turn_exit_condition_set(90_ms, 1.75_deg, 300_ms, 5_deg, 250_ms,
	// 									250_ms);
	pros::delay(250); 
	chassis.drive_set(60,60); //70,70
	pros::delay(1700);
	// chassis.drive_set(60,60);
	// pros::delay(400);
	chassis.drive_set(0,0);

	//PARK RESET
	chassis.pid_drive_set(-5,100,40,false,false);
	chassis.pid_wait_quick_chain(2);
	chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 100_ms,
										 100_ms);
	chassis.pid_swing_set(ez::RIGHT_SWING,-90,90);
	chassis.pid_wait_quick();
	chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 500_ms,
										 500_ms);
	// chassis.odom_theta_set(-90_deg); // REMOVE ONCE DONE TESTING REST OF CODE
	// //REMOVE BELOW FOR TIME SAVING (ONLY. IF. NECESSARY.)
	chassis.pid_drive_set(6,90,true,true,-90);
	chassis.pid_wait_quick();
	
	std::vector<okapi::QLength> l = localizer.get_park_clear_localization(Localizer::Corner::BL,3);
	x = l[0];
	y = l[1];
	chassis.odom_xy_set(x,y);
	

	// x = (71-7.5)*1_in;                                                //REMOVE ONCE DONE TESTING REST OF CODE
	// y = localizer.get_localized_coordinate(Localizer::Corner::BL);     //REMOVE ONCE DONE TESTING REST OF CODE
	// chassis.odom_xyt_set(x,y,-90_deg);                                 //REMOVE ONCE DONE TESTING REST OF CODE
	
	
	//moving to score park
	pros::delay(200);
	chassis.pid_odom_ptp_set({{43_in,-40_in},fwd,80},true); //46,46
	intake.set(Intake::IntakeState::OUTTAKE,127);
	pros::delay(200);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_wait_quick();
	chassis.pid_turn_set(90,127);
	chassis.pid_wait_quick();
	chassis.pid_odom_ptp_set({{24_in,-47_in},rev,80},true); //46,46
	chassis.pid_wait_quick();
	pros::delay(500);
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
	chassis.drive_set(-120,-120);
	pros::delay(700);
	chassis.drive_set(0,0);
	//localizing
	x = (30)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::BR);
	chassis.odom_xy_set(x,y);
	
		
	//getting third matchloader
	chassis.pid_turn_set({60,-44},fwd,90);
	loaderPiston.set(true);
	chassis.pid_wait_quick();
	hoodPiston.set(false);
	chassis.pid_drive_set(29.5_in,57, false, true);
	// liftPiston.set(true);
    intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_wait();
	pros::delay(500);
	chassis.pid_drive_set(-1.5_in, 80, true);
	chassis.pid_wait();
	chassis.pid_drive_set(2.2_in, 80, true);
	chassis.pid_wait_quick();
	pros::delay(1250);
	chassis.pid_drive_set(-5_in,80,30,true);
	chassis.pid_wait_quick_chain();
	liftPiston.set(false);


	//Go to other side to score
	chassis.pid_odom_ptp_set({{35,-60},rev,80}, true);
	intake.set(Intake::OUTTAKE,127);
	pros::delay(100);
	intake.set(Intake::INTAKE,127);
	chassis.pid_wait_quick_chain(4);
	chassis.pid_turn_set({-23,-54.7},rev,90);
	chassis.pid_wait_quick_chain();

	chassis.pid_odom_ptp_set({{-23,-54},rev,85}, true);
    chassis.pid_wait_until(-43_in);
    loaderPiston.set(false);
	chassis.pid_wait_quick_chain(3);
    // chassis.pid_turn_set({-40_in,-46_in},rev,90);
	chassis.pid_odom_ptp_set({{-41.7,-42},rev,80}, true);
    chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,90);
	chassis.pid_wait_quick_chain();
    //wallreset
    x = chassis.odom_x_get()*1_in;
    y = localizer.get_localized_coordinate(Localizer::Corner::BL);
    chassis.odom_xy_set(x,y);

	//Scoring third matchloader
	chassis.pid_odom_ptp_set({{-23.3,-45.4},rev,90},true); 
	chassis.pid_wait_until({-27.5_in, -46_in});
	intake.set(Intake::OUTTAKE,127);
	pros::delay(100);
	intake.set(Intake::INTAKE,127);
    chassis.pid_wait_quick();
	pros::delay(400);
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
    loaderPiston.set(true);
    chassis.drive_set(-120, -120);
    pros::delay(700);
    chassis.drive_set(0, 0);
    intake.set(Intake::IntakeState::INTAKE,127);


	//Get last matchloader and score
	chassis.pid_turn_set({-60,-45},fwd,-90);
	chassis.pid_drive_set(29.2_in,55, false, true);
	chassis.pid_wait_until(12);
	// liftPiston.set(true);
	hoodPiston.set(false);
	chassis.pid_wait();
	pros::delay(200);
	chassis.pid_drive_set(-1.3_in, 80, true);
	chassis.pid_wait();
	chassis.pid_drive_set(1.7_in, 80, true);
	chassis.pid_wait_quick();
	pros::delay(1550);
	chassis.pid_odom_ptp_set({{-24.5,-46},rev,90},true);
	intake.set(Intake::IntakeState::OUTTAKE,127);
	pros::delay(100);
	intake.set(Intake::IntakeState::INTAKE,127);
	liftPiston.set(false);
    chassis.pid_wait_quick();
	pros::delay(500);
	leverProfiler.setProfile({{-10, 70}});
    intake.set(Intake::IntakeState::AUTON_SCORING,127);
	chassis.drive_set(-90,-90);
	pros::delay(500);
	chassis.drive_set(0,0);

	//localizing
	x = (-30)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::BL);
	chassis.odom_xy_set(x,y);

	// chassis.odom_theta_set(-90_deg); //REMOVE ONCE DONE TESTING REST OF CODE

	loaderPiston.set(false);
	intake.set(Intake::INTAKE,127);
	chassis.pid_drive_set(5.5,80,30,true,false);
	chassis.pid_wait_quick();


	//Go to Park
	chassis.pid_odom_ptp_set({{-49,-39},fwd,80}, true);
	chassis.pid_wait_quick_chain(4);
	chassis.pid_odom_ptp_set({{-60,-20.5},fwd,60}, true); //61.5,21
	chassis.pid_wait_quick_chain(3);
	loaderPiston.set(true);
	chassis.pid_swing_set(ez::LEFT_SWING, -10, 90,0);
	chassis.pid_wait_quick();
	
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms,							500_ms);

	//Clearing Park
	// chassis.pid_drive_set(39_in,127,43,false,false);
	chassis.drive_set(75,75);
	pros::delay(380);
	loaderPiston.set(false);
	pros::delay(400); 
	// chassis.pid_wait();
	chassis.drive_set(0,0);


	chassis.pid_drive_set(-5,80,true);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(8,80,true,false);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(-5,80,true);
	chassis.pid_wait_quick();




}
 
void skills(){
	leverProfiler.setProfile({{-10, 127},{90,45},{100,40}});
    chassis.pid_odom_drive_exit_condition_set(100_ms, 1_in, 150_ms, 5_in, 250_ms, 500_ms);
	okapi::QLength x;
	okapi::QLength y;
	flapperPiston.set(true);
	
	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(-46.5_in, 6_in, 90_deg);
	liftPiston.set(true);
	// Score two balls in middle for redundancy
	chassis.pid_odom_ptp_set({{-24,23}, fwd,60}, true, 17, 25); //28,18
    chassis.pid_wait_until({-27_in, 17_in});
    loaderPiston.set(true);
    pros::delay(500);
    loaderPiston.set(false);
	chassis.pid_wait_quick();
	//intake.set(Intake::STOP);
    // chassis.pid_odom_ptp_set({{-27,17.5}, rev,70}, false);
    // pros::delay(200);
    chassis.pid_turn_set(-43, 100);
    chassis.pid_wait_quick();
    loaderPiston.set(false);
	// chassis.pid_swing_set(ez::RIGHT_SWING,-45,70,-7);
	// intake.set(Intake::IntakeState::INTAKE,127);
	// chassis.pid_wait_quick();
	// chassis.pid_drive_set(-21.5_in, 80, true, true, -90);
    // liftPiston.set(true);
	// chassis.pid_wait_until(-17_in);
    chassis.pid_drive_set(-29.5_in, 80, true, true, -45);
    chassis.pid_wait_until(-16_in);
    
	intake.set(Intake::IntakeState::AUTON_SCORING,90);
    // intake.waitUntilColor(190, 260, 0.3, 0, 600);
	pros::delay(200);
    chassis.pid_wait_quick();
	intake.set(Intake::IntakeState::INTAKE,127);


	// moving to score three
	liftPiston.set(false);
	chassis.pid_odom_ptp_set({{-36,44.5},fwd,70}, true);
	chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,127);
	chassis.pid_wait();

    //score 3
    chassis.pid_drive_set(-15_in, 80, true);
    chassis.pid_wait_until(-10_in);
    intake.set(Intake::IntakeState::AUTON_SCORING,127,80);
    chassis.pid_wait_quick();

	// first matchloader
    loaderPiston.set(true);
    pros::delay(350);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_turn_set({-60,49},fwd,90);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(31_in,43, false, true, -90);
	intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_wait(); 
	x = (-71 + 14.3125)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::TL);
	chassis.odom_xy_set(x,y);
	pros::delay(600);


	//Go to other side to score
	chassis.pid_drive_set(-4.5_in, 80,30, true);
	chassis.pid_wait_quick_chain();
	chassis.pid_turn_set({-36,60.5},rev,90);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_ptp_set({{-37,59.5},rev,80}, true);
	chassis.pid_wait_quick_chain(3);
    // chassis.pid_turn_set(-90,127);
    // chassis.pid_wait_quick_chain(10);
	chassis.pid_odom_ptp_set({{23,54.5},rev,80}, true);
    

    chassis.pid_wait_until(-43_in);
    loaderPiston.set(false);
	chassis.pid_wait_quick();
    // chassis.pid_turn_set({40_in,46_in},rev,90);
	chassis.pid_odom_ptp_set({{40,41.4},rev,75}, true);
    chassis.pid_wait_quick();
	chassis.pid_turn_set(90,90);
	chassis.pid_wait_quick();
    //wallreset
    x = chassis.odom_x_get()*1_in;
    y = localizer.get_localized_coordinate(Localizer::Corner::TR);
    chassis.odom_xy_set(x,y);

	//Scoring first matchloader
	chassis.pid_odom_ptp_set({{22,46},rev,90},true);
	chassis.pid_wait_until({26_in, 46_in});
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
    chassis.pid_wait_quick();
    chassis.drive_set(-90, -90);
    loaderPiston.set(true);
    pros::delay(600);
    chassis.drive_set(0, 0);
	pros::delay(200);
    intake.set(Intake::IntakeState::INTAKE,127);


	//Get second matchloader and score
	chassis.pid_turn_set({60,48},fwd,90);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(30_in,50,true,true, 90);
	chassis.pid_wait();
	x = (71 - 14.3125)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::TR);
	chassis.odom_xy_set(x,y);
	pros::delay(750);
	chassis.pid_odom_ptp_set({{30,46.5},rev,80},true);
	chassis.pid_wait_until({32_in, 46_in});
	intake.set(Intake::IntakeState::AUTON_SCORING, 100, 90);
	chassis.pid_wait_quick();
    chassis.drive_set(-90, -90);
    pros::delay(700);
    chassis.drive_set(0, 0);
	pros::delay(300);
	intake.set(Intake::IntakeState::AUTON_SCORING,80,127);
	pros::delay(550);
	loaderPiston.set(false);
	// chassis.odom_xyt_set(30.83_in,48.35_in,90_deg);  //REMOVE ONCE DONE TESTING REST OF CODE
	chassis.pid_drive_set(7,80,true,false);
	chassis.pid_wait_quick_chain();




	// Go to Park
	chassis.pid_odom_ptp_set({{49,39},fwd,80}, true);
	chassis.pid_wait_quick_chain(4);
	chassis.pid_odom_ptp_set({{63,21},fwd,80}, true); //61.5,21
	pros::delay(600);
	loaderPiston.set(true);
    intake.set(Intake::INTAKE,127);
	chassis.pid_wait_quick_chain(3);
	chassis.pid_swing_set(ez::LEFT_SWING, 169, 90,2);
	// chassis.pid_turn_set(170,90);
	// pros::delay(600);
	// loaderPiston.set(false);
	chassis.pid_wait_quick();
	//Clearing Park
	// chassis.pid_drive_set(32_in,80,false,false);
	chassis.drive_set(73,73);  //67,67
	pros::delay(100);
	loaderPiston.set(false);
	pros::delay(770);
	chassis.drive_set(0,0);
	chassis.pid_turn_exit_condition_set(90_ms, 1.75_deg, 100_ms, 5_deg, 100_ms,
										100_ms);
	chassis.pid_turn_set(178,90);
	chassis.pid_wait_quick();
	chassis.pid_turn_exit_condition_set(90_ms, 1.75_deg, 300_ms, 5_deg, 250_ms,
										250_ms);
	pros::delay(250);  //300 gets 6 balls
	chassis.drive_set(75,75); //70,70
	pros::delay(1000);
	// chassis.drive_set(60,60);
	// pros::delay(400);
	chassis.drive_set(0,0);
	
	
	


	// OLD RESET VERSION

	// //LOWERING VELOCITY EXIT FOR QUICK END TO RESET MOVEMENTS
    // chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 50_ms,
	// 									 50_ms);
	// chassis.pid_drive_set(-30,60,false,true,180);
	// chassis.pid_wait_quick();
    // chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 200_ms,
	// 									 250_ms);

	// x = localizer.get_localized_coordinate(Localizer::Corner::TR);
	// // y = -17*1_in;
	// y = -localizer.get_localized_coordinate(Localizer::Corner::RAW_FRONT);
	


	chassis.pid_drive_set(-3,100,40,false,false);
	chassis.pid_wait_quick_chain(2);
	chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 100_ms,
										 100_ms);
	chassis.pid_swing_set(ez::RIGHT_SWING,-90,90);
	chassis.pid_wait_quick();
	chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 500_ms,
										 500_ms);
	// chassis.odom_theta_set(-90_deg); // REMOVE ONCE DONE TESTING REST OF CODE
	// //REMOVE BELOW FOR TIME SAVING (ONLY. IF. NECESSARY.)
	chassis.pid_drive_set(6,90,true,true,-90);
	chassis.pid_wait_quick();
	
	std::vector<okapi::QLength> l = localizer.get_park_clear_localization(Localizer::Corner::BL,3);
	x = l[0];
	y = l[1];
	chassis.odom_xy_set(x,y);


	// x = (71-7.5)*1_in;                                                //REMOVE ONCE DONE TESTING REST OF CODE
	// y = localizer.get_localized_coordinate(Localizer::Corner::BL);     //REMOVE ONCE DONE TESTING REST OF CODE
	// chassis.odom_xyt_set(x,y,-90_deg);                                 //REMOVE ONCE DONE TESTING REST OF CODE
	
	
    // chassis.pid_drive_set(7,80, false);    ADD BACK FOR OLD RESET
	chassis.pid_swing_set(ez::LEFT_SWING,-30,90);

	// get one extra ball in addition to park balls for redundancy
    chassis.pid_odom_ptp_set({{42,-15},fwd,90}, true);
	chassis.pid_wait_quick_chain(3);
	chassis.pid_odom_ptp_set({{26,-27},fwd,60}, true,12,25); //25.7,-28.5
	chassis.pid_wait_until({31,-15});
    intake.set(Intake::STOP);
	chassis.pid_wait_quick();
	
	


	//score 7 of same color in middle SCORING FOR MID
	// chassis.pid_drive_set(3.8_in,60,true);
	chassis.pid_turn_set(132,40, ez::e_angle_behavior::cw, true);
	pros::delay(500);
	intake.set(Intake::IntakeState::INTAKE,60);
	chassis.pid_wait_quick();
	liftPiston.set(true);
    pros::delay(100);
	chassis.pid_drive_set(-20_in, 80, true);//-19.5
	chassis.pid_wait_until(-17.5_in);
    intake.set(Intake::AUTON_SCORING,40,90);
	chassis.pid_wait_quick();
	chassis.pid_drive_set(2.1_in,60, 10, true);//1.8   //NOTE: Pid wait at the end of scoringz for this movement

	loaderPiston.set(false);
    pros::delay(300);
	intake.set(Intake::AUTON_SCORING,-40,-20); //edging in the middle
	pros::delay(100);
    intake.set(Intake::AUTON_SCORING,40,80);
	pros::delay(1000);
    intake.set(Intake::AUTON_SCORING,36,75);
    intake.waitUntilColor(190, 260, 0.3, 0, 1650);
    intake.set(Intake::IntakeState::AUTON_SCORING, -20, 50);
	liftPiston.set(false);
    //chassis.pid_drive_set(1.3_in,60, 40, false);
	chassis.pid_wait();

	//moving to score 3
	pros::delay(200);
	chassis.pid_odom_ptp_set({{35_in,-46_in},fwd,80},true); //46,46
	pros::delay(300);
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
    
	chassis.pid_wait_quick();
	
	chassis.pid_turn_set(90,90);
	// chassis.pid_wait_quick_chain();
    // intake.set(Intake::IntakeState::INTAKE,127);

	// //score 3
	// chassis.pid_odom_ptp_set({{24,-49},rev,70},true);
	// pros::delay(700);
	// chassis.pid_wait_quick();
	// loaderPiston.set(true);
	// // pros::delay(300);
	
	// //Intake Third Matchloader
	// chassis.pid_turn_set({60,-48.5},fwd,90);
	chassis.pid_wait_quick();

    // ignore scoring
    loaderPiston.set(true);
    pros::delay(100);

	//chassis.pid_drive_set(30.5_in,42,true, true, 90);
    chassis.pid_drive_set(19_in,40,true, true, 90);
	chassis.pid_wait_until(7_in);
    intake.set(Intake::IntakeState::INTAKE,127);
	chassis.pid_wait();
	pros::delay(500);
	//localizing
	x = (71 - 14.3125)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::BR);
	chassis.odom_xy_set(x,y);
	chassis.pid_drive_set(-5_in,80,30,true);
	chassis.pid_wait_quick_chain();


	//Go to other side to score
	chassis.pid_odom_ptp_set({{35,-61},rev,80}, true);
	chassis.pid_wait_quick_chain(4);
	chassis.pid_turn_set({-23,-54.7},rev,90);
	chassis.pid_wait_quick_chain();

	chassis.pid_odom_ptp_set({{-23,-55},rev,85}, true);
    chassis.pid_wait_until(-43_in);
    loaderPiston.set(false);
	chassis.pid_wait_quick_chain(3);
    // chassis.pid_turn_set({-40_in,-46_in},rev,90);
	chassis.pid_odom_ptp_set({{-41.7,-41.3},rev,80}, true);
    chassis.pid_wait_quick();
	chassis.pid_turn_set(-90,90);
	chassis.pid_wait_quick_chain();
    //wallreset
    x = chassis.odom_x_get()*1_in;
    y = localizer.get_localized_coordinate(Localizer::Corner::BL);
    chassis.odom_xy_set(x,y);

	//Scoring third matchloader
	chassis.pid_odom_ptp_set({{-23.3,-47.2},rev,90},true); 
	chassis.pid_wait_until({-25.5_in, -46_in});
	intake.set(Intake::IntakeState::AUTON_SCORING,127);
    chassis.pid_wait_quick();
    loaderPiston.set(true);
    chassis.drive_set(-90, -90);
    pros::delay(700);
    chassis.drive_set(0, 0);
    pros::delay(400);
    intake.set(Intake::IntakeState::INTAKE,127);

	//Get last matchloader and score
	chassis.pid_turn_set({-60,-43},fwd,-90);
	chassis.pid_drive_set(30_in,40,true,true, -90);
	chassis.pid_wait();
	x = (-71 + 14.3125)*1_in;
	y = localizer.get_localized_coordinate(Localizer::Corner::BL);
	chassis.odom_xy_set(x,y);
	pros::delay(750);
	chassis.pid_odom_ptp_set({{-27.5,-46.5},rev,90},true);
    chassis.pid_wait_until({-32_in, -46_in});
    intake.set(Intake::IntakeState::AUTON_SCORING,80);
	pros::delay(1500);
	chassis.pid_drive_set(1.5,80,10,true,false);
	intake.set(Intake::IntakeState::AUTON_SCORING,65);
	pros::delay(700);
    chassis.pid_wait_quick();
	loaderPiston.set(false);
	intake.set(Intake::INTAKE,127);
	chassis.pid_drive_set(5.5,80,30,true,false);
	chassis.pid_wait_quick();


	//Go to Park
	chassis.pid_odom_ptp_set({{-49,-39},fwd,80}, true);
	chassis.pid_wait_quick_chain(4);
	chassis.pid_odom_ptp_set({{-64,-23},fwd,80}, true); //61.5,21
	pros::delay(700);
	loaderPiston.set(true);
	chassis.pid_wait_quick_chain(3);
	chassis.pid_swing_set(ez::LEFT_SWING, -10, 90,0);
	chassis.pid_wait_quick();
	
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms,							500_ms);

	//Clearing Park
	// chassis.pid_drive_set(39_in,127,43,false,false);
	chassis.drive_set(82,82);
	pros::delay(380);
	loaderPiston.set(false);
	pros::delay(500);
	// chassis.pid_wait();
	chassis.drive_set(0,0);
	intake.set(Intake::INTAKE,127);
	chassis.pid_turn_set(-30,127);
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


