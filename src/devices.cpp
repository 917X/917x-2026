#include "devices.hpp"

// device ports
constexpr int RIGHT_F = -18;
constexpr int RIGHT_M = 19;
constexpr int RIGHT_B = -17;

constexpr int LEFT_F = 15;
constexpr int LEFT_M = -14;
constexpr int LEFT_B = 13;

constexpr int INDEXER = -10;
constexpr int MIDDLE_ROLLER = 7;
constexpr int BOTTOM_ROLLER = -8;
constexpr int FRONT_ROLLER = 16;

constexpr int DISTANCE = 1;

constexpr char INTAKE_PISTON = 'H';
constexpr char FLAPPER_PISTON = 'G';
constexpr char COLORSORT = 6;

constexpr char IMU = 21;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// optical
pros::Optical colorSort(COLORSORT);

// pistons
ez::Piston intakePiston(INTAKE_PISTON);
ez::Piston flapperPiston(FLAPPER_PISTON);

// intake motors
pros::Motor indexerMotor(INDEXER);
pros::Motor bottomRoller(BOTTOM_ROLLER);
pros::Motor middleRoller(MIDDLE_ROLLER);
pros::Motor frontRoller(FRONT_ROLLER);

// intake
Intake intake(indexerMotor, colorSort, frontRoller, middleRoller, bottomRoller);

// drive chassis
ez::Drive chassis({15, -14, 13},  // Left Chassis Ports
				  {-18, 19, -17}, // Right Chassis Ports
				  21,			  // IMU Port
				  2.75,			  // Wheel Diameter
				  600);			  // Drive RPM

// default chassis constants
void default_constants() {
	// lateral constants
	chassis.pid_drive_constants_forward_set(11.2, 0, 56);
	chassis.pid_drive_constants_backward_set(5.7, 0.0, 9);

	// angular constants
	chassis.pid_heading_constants_set(11, 0, 50.0);
	chassis.pid_turn_constants_set(3.2, 0, 25, 16.0);
	chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
	chassis.pid_odom_angular_constants_set(6.5, 0.0, 60.5);
	chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);

	// exit conditions
	chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,
										500_ms);
	chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,
										 500_ms);
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms,
										 500_ms);
	chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg,
											 500_ms, 750_ms);
	chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 5_in, 500_ms,
											  750_ms);
	chassis.pid_turn_chain_constant_set(3_deg);
	chassis.pid_swing_chain_constant_set(5_deg);
	chassis.pid_drive_chain_constant_set(3_in);

	// slewrate controller constants
	chassis.slew_turn_constants_set(3_deg, 70);
	chassis.slew_drive_constants_forward_set(3.5_in, 40);
	chassis.slew_drive_constants_backward_set(0.5_in, 110);
	chassis.slew_swing_constants_set(3_in, 80);

	// odometry heading vs lateral driving priority
	chassis.odom_turn_bias_set(0.9);

	// carrot points
	chassis.odom_look_ahead_set(7_in);
	chassis.odom_boomerang_distance_set(16_in);
	chassis.odom_boomerang_dlead_set(0.625);

	// turn behavior
	chassis.pid_angle_behavior_set(ez::shortest);
}
