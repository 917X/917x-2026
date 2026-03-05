#include "devices.hpp"
#include "EZ-Template/tracking_wheel.hpp"

// device ports
constexpr int LEFT_F = -20;
constexpr int LEFT_M = 19;
constexpr int LEFT_B = -18;

constexpr int RIGHT_F = 11;
constexpr int RIGHT_M = -12;
constexpr int RIGHT_B = 13;

constexpr int VERT_POD = 15;

constexpr int INDEXER = -3;
constexpr int ROLLER = 14;

constexpr int LEFT_DISTANCE = 17;
constexpr int RIGHT_DISTANCE = 2;
constexpr int FRONT_DISTANCE = 16;

constexpr char LOADER_PISTON = 'A';
constexpr char FLAPPER_PISTON = 'C';
constexpr char LIFT_PISTON = 'B';

constexpr char COLORSORT = 10;

constexpr char IMU = 1;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// pistons
ez::Piston loaderPiston(LOADER_PISTON);
ez::Piston flapperPiston(FLAPPER_PISTON);
ez::Piston liftPiston(LIFT_PISTON);

// intake motors
pros::Motor indexerMotor(INDEXER);
pros::Motor rollerMotor(ROLLER);

// distance sensors
pros::Distance leftDistance(LEFT_DISTANCE);
pros::Distance rightDistance(RIGHT_DISTANCE);
pros::Distance frontDistance(FRONT_DISTANCE);

// optical sensor
pros::Optical colorSensor(COLORSORT);

// intake
Intake intake(rollerMotor, indexerMotor, &colorSensor);

// drive chassis
ez::Drive chassis({LEFT_F, LEFT_M, LEFT_B},	   // Left Chassis Ports
				  {RIGHT_F, RIGHT_M, RIGHT_B}, // Right Chassis Ports
				  IMU,						   // IMU Port
				  2.75,						   // Wheel Diameter
				  600);						   // Drive RPM

ez::tracking_wheel vertical_pod(VERT_POD, 2,0.25);

Localizer localizer(&leftDistance, &rightDistance, nullptr, &frontDistance, 4.875, 4.875, 0, 7);

// default chassis constants
void default_constants() {

	chassis.odom_tracker_right_set(&vertical_pod);
    chassis.drive_width_set(10.5);

	// lateral constants
	chassis.pid_drive_constants_forward_set(6, 0.0, 11.5);
	chassis.pid_drive_constants_backward_set(5.8, 0.0, 14);

	// heading correction constants
	chassis.pid_heading_constants_set(4, 0.0, 9); // 5 0 9

	// angular constants
	chassis.pid_turn_constants_set(2, 0.2, 11.1, 5);
	chassis.pid_swing_constants_set(3.8, 0.3, 21.25, 5);
	chassis.pid_odom_angular_constants_set(1.8, 0.0, 11, 5);
	chassis.pid_odom_boomerang_constants_set(6, 0.0, 24);

	// exit conditions
	chassis.pid_turn_exit_condition_set(90_ms, 1.75_deg, 300_ms, 5_deg, 250_ms,
										250_ms);
	chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 7_deg, 500_ms,
										 500_ms);
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 200_ms,
										 250_ms);
	chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg,
											 250_ms, 250_ms);
	chassis.pid_odom_drive_exit_condition_set(50_ms, 1.5_in, 100_ms, 5_in,
											  250_ms, 500_ms);
	chassis.pid_turn_chain_constant_set(3_deg);
	chassis.pid_swing_chain_constant_set(5_deg);
	chassis.pid_drive_chain_constant_set(3_in);

	// slewrate controller constants
	chassis.slew_turn_constants_set(15_deg, 40);
	chassis.slew_drive_constants_forward_set(3_in, 50);
	chassis.slew_drive_constants_backward_set(3_in, 50);
	chassis.slew_swing_constants_set(5_in, 80);

	// odometry heading vs lateral driving priority
	chassis.odom_turn_bias_set(0.9);
	
	// odometry drive boost (boost weak output when far from target)
	chassis.odom_drive_boost_set(30, 1.5); 
    // carrot points
	chassis.odom_look_ahead_set(7_in);
	chassis.odom_boomerang_distance_set(16_in);
	chassis.odom_boomerang_dlead_set(0.625);

	// turn behavior
	chassis.pid_angle_behavior_set(ez::shortest);

}
