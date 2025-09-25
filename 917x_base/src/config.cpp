#include "config.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"

// ports
constexpr int RIGHT_F = -18;
constexpr int RIGHT_M = 19;
constexpr int RIGHT_B = -17;

constexpr int LEFT_F = 15;
constexpr int LEFT_M = -14;
constexpr int LEFT_B = 13;


constexpr int INDEXER = -10;
constexpr int BOTTOM_ROLLER = 7;
constexpr int MIDDLE_ROLLER = -8;
constexpr int FRONT_ROLLER = 16;

constexpr int DISTANCE = 1;


constexpr char CLAMP = 'A';
constexpr char HOOK = 'B';
constexpr char HANG = 'G';
constexpr char TOP_SORT = 8;

constexpr char IMU = 21;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// optical
pros::Optical colorSort(TOP_SORT);

pros::Motor indexerMotor(INDEXER);
pros::Motor bottomRoller(BOTTOM_ROLLER);
pros::Motor middleRoller(MIDDLE_ROLLER);
pros::Motor frontRoller(FRONT_ROLLER);

Intake intake(indexerMotor, colorSort, frontRoller, middleRoller, bottomRoller);

// drivetrain
pros::MotorGroup rightMotors({RIGHT_F, RIGHT_M, RIGHT_B}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({LEFT_F, LEFT_M, LEFT_B}, pros::MotorGearset::blue);

