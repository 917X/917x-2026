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
	chassis.pid_wait_quick_chain();
	intake.set(Intake::IntakeState::SCORE, 105);
	pros::delay(1300);

	intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_odom_set({{40_in, -44_in}, fwd, 110}, true);
	chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::RIGHT_SWING, 91_deg, 90, 0, true);
	chassis.pid_wait_quick_chain();
	loaderPiston.set(true);
	pros::delay(50);

	chassis.pid_drive_set(7_in, 80, true);
	chassis.pid_wait_quick_chain();
	pros::delay(800);
	chassis.pid_drive_set(-31_in, 90, true);
	chassis.pid_wait_quick_chain();
	
	intake.set(Intake::IntakeState::SCORE, 127);
	pros::delay(1500);
    loaderPiston.set(false);


	intake.set(Intake::IntakeState::INTAKE, 110);
	chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 80, 20, true);
	chassis.pid_wait_quick_chain();
	chassis.pid_odom_set({{17.2_in, 24.5_in, -15_deg}, fwd, 100});
	chassis.pid_wait_quick_chain();
	intake.set(Intake::IntakeState::STOP);

	chassis.pid_turn_set(45_deg, 100); // 50
    pros::delay(600);
    intake.set(Intake::IntakeState::SCORE, 127);
	chassis.pid_wait_quick();

	chassis.pid_drive_set(-15.5_in, 40, true);
	
    chassis.pid_wait_quick();

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
	chassis.pid_wait_quick_chain();
	intake.set(Intake::IntakeState::SCORE, 105);
	pros::delay(1300);

	intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_odom_set({{40_in, -44_in}, fwd, 110}, true);
	chassis.pid_wait_quick_chain();
	chassis.pid_swing_set(ez::RIGHT_SWING, 91_deg, 90, 0, true);
	chassis.pid_wait_quick_chain();
	loaderPiston.set(true);
	pros::delay(50);

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
    pros::delay(100);

	intake.set(Intake::IntakeState::INTAKE, 127);

	chassis.odom_xyt_set(45_in, 6_in, -90_deg);

	// Mirrored: 22_in,-21_in,-160_deg becomes 22_in,21_in,160_deg
	chassis.pid_odom_set({{22_in, 21_in, -20_deg}, fwd, 90}, true);
	chassis.pid_wait_quick_chain();

	chassis.pid_odom_set({{{17_in, 25.3_in}, fwd, 60}}, true);
	chassis.pid_wait_quick();


    chassis.pid_swing_set(ez::RIGHT_SWING, 45, 90, -23, true); 
    intake.set(Intake::STOP);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(-9_in, 80, true);
	chassis.pid_wait_quick_chain();
	intake.set(Intake::IntakeState::SCORE, 100);
	pros::delay(650);



	// go matchload
    intake.set(Intake::IntakeState::INTAKE, 127);
	chassis.pid_odom_set({{40_in, 44_in}, fwd, 110}, true);
	chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::LEFT_SWING, 89_deg, 90, 0, true);
	chassis.pid_wait_quick_chain();
	loaderPiston.set(true);
	pros::delay(50);

    // score high
    chassis.pid_drive_set(7_in, 80, true);
	chassis.pid_wait_quick_chain();
	pros::delay(800);
	chassis.pid_drive_set(-31_in, 90, true);
	chassis.pid_wait_quick_chain();
	
	intake.set(Intake::IntakeState::SCORE, 127);
	pros::delay(1500);
    intake.set(Intake::IntakeState::STOP);
    loaderPiston.set(false);
    chassis.pid_swing_set(ez::LEFT_SWING, 175, 100);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::RIGHT_SWING, 90, 100);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-31.5_in, 50, true);
    flapperPiston.set(false);
    chassis.pid_wait_quick();
    flapperPiston.set(true);

}


void skills(){
//   intake.set(Intake::IntakeState::INTAKE,127);
//   chassis.odom_xyt_set(-45_in,6_in,90_deg);
//   chassis.pid_odom_set({{{-22_in,22_in,40_deg}, fwd, 80},},true);
//   chassis.pid_wait_quick();
//   // chassis.pid_turn_set({50_in,-45_in},fwd,90);
//   chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, 100, true);//90,40
//   chassis.pid_wait_quick();
//   chassis.pid_odom_set({{-37,47}, fwd, 110}, true);
//   chassis.pid_wait_quick();
//   intake.set(Intake::IntakeState::STOP);
//   chassis.pid_turn_set(-90_deg, 90);
//   chassis.pid_wait_quick();


  intake.set(Intake::IntakeState::INTAKE,127);
  chassis.odom_xyt_set(-53_in,16_in,0_deg);
  chassis.pid_turn_set({-46,48},fwd,90);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-46_in,48_in},fwd,120},false); //-46,48
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90,90);
  chassis.pid_wait_quick();

  loaderPiston.set(true);
  chassis.pid_drive_set(11,120,true); //8.7
  chassis.pid_wait_quick();
  pros::delay(2000);
  chassis.pid_turn_set(-92,90);
  intake.set(Intake::IntakeState::STOP);
  chassis.pid_wait_quick();
  pros::delay(300);
  intake.set(Intake::IntakeState::INTAKE,127);
  chassis.pid_drive_set(-27_in,50,true);
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::SCORE);
  pros::delay(1000);
  intake.set(Intake::IntakeState::STOP);
  pros::delay(200);
  intake.set(Intake::IntakeState::SCORE,127);
  pros::delay(750);
  loaderPiston.set(false);
  pros::delay(500);


  chassis.pid_drive_set(2_in,100,false,false);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING,90_deg,90,-22,counterclockwise,true);
  chassis.pid_wait_quick();
  intake.set(Intake::IntakeState::OUTTAKE);
  


//PUSHING TO MIDDLE
  flapperPiston.set(true);
  chassis.pid_drive_set(18,50,true,true);
  chassis.pid_wait_quick(); 
  flapperPiston.set(false);
  chassis.pid_turn_set(92,90);
  chassis.pid_wait_quick();
  



  //ROBOT GOING TO SECOND LOADBAR
  chassis.pid_drive_set(50_in,90,true);
  intake.set(Intake::IntakeState::INTAKE,127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{50,51.2},fwd,90},true); //48,49.5
  chassis.pid_wait();
  chassis.pid_turn_set(90,100);
  chassis.pid_wait();
  loaderPiston.set(true);
  pros::delay(500); //lower if necessary
  chassis.pid_drive_set(9.2_in,120,true); //11
  chassis.pid_wait_quick();
  pros::delay(2000);

  chassis.pid_turn_set(90,90);
  intake.set(Intake::IntakeState::STOP);
  chassis.pid_wait_quick();
  intake.set(Intake::IntakeState::INTAKE,127);
  chassis.pid_drive_set(-29_in,70,true,true);
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::SCORE);
  pros::delay(1500);
  loaderPiston.set(false);
  pros::delay(500);

  
  //SCORING TO CENTER 
  intake.set(Intake::IntakeState::INTAKE,127);
  chassis.pid_drive_set(2,100,false);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(200,90,true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(30_in,80,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{19,-17},fwd,50},true); //18,-19
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING,138,90,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in,70,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-35_in,70,false);
  chassis.pid_wait_quick();
  intake.set(Intake::OUTTAKE,127);
  pros::delay(300);
  intake.set(Intake::IntakeState::SCORE,127);
  pros::delay(1000);





// GOING TO THIRD LOADBAR
  chassis.pid_odom_set({{42,-41.5},fwd,100},true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90,90);
  chassis.pid_wait();

  //include code to intake from load bar
  intake.set(Intake::IntakeState::INTAKE,127);
  loaderPiston.set(true);
  pros::delay(500); //lower if necessary
  chassis.pid_drive_set(9,50,true);
  chassis.pid_wait_quick();

  pros::delay(2000); //remove once intake coded
  intake.set(Intake::IntakeState::STOP);
  chassis.pid_drive_set(-29_in,60,true);
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::SCORE);
  pros::delay(1000);
  loaderPiston.set(false);
  pros::delay(750);
  








  //SCORED THIRD LOADBAR. GOING TO FOURTH LOADBAR
  chassis.pid_drive_set(5_in,100,false,false);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::LEFT_SWING,269_deg,90,-10,clockwise,true);
  chassis.pid_wait_quick();
  intake.set(Intake::IntakeState::SCORE,127);
  chassis.pid_drive_set(60,100,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-51,-38.2},fwd,110},true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-92,90);
  chassis.pid_wait_quick();
  
  loaderPiston.set(true);
  chassis.pid_drive_set(15.5_in,120,true);
  chassis.pid_wait_quick();
  pros::delay(2000); //remove once intake coded

  intake.set(Intake::IntakeState::STOP);
  chassis.pid_turn_set(-91,90);
  chassis.pid_wait();
  chassis.pid_drive_set(-29_in,50,true);
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::SCORE);
  pros::delay(1000);
  loaderPiston.set(false);
  pros::delay(1000);

  //Pushing balls into center
  chassis.pid_drive_set(10_in,110,false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-135,110,true);
  chassis.pid_wait_quick_chain();
  flapperPiston.set(true);
  chassis.pid_drive_set(-9.6_in,110,false); //10
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING,-90,110,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-9.5_in,70,false);  //-9.8 -10 -12     <-110
  chassis.pid_wait_quick();
  flapperPiston.set(false);

  //going to fourball
  chassis.pid_turn_set(-50,90,true); //120.    <-90
  chassis.pid_wait_quick();
  intake.set(Intake::IntakeState::INTAKE,127);
  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 63,22, clockwise,true); //-135,60,20
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5,80,true);
  chassis.pid_wait_quick();
  intake.set(Intake::IntakeState::OUTTAKE);
  pros::delay(1000);

  chassis.pid_drive_set(-55,1000,false);
  chassis.pid_wait_quick_chain();
  intake.set(Intake::IntakeState::SCORE);
  chassis.pid_turn_set(0,90);
  chassis.pid_wait();
  
}
