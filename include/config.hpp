#pragma once
#include "main.h"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/imu.hpp"
#include "subsystem/intake.hpp"

extern pros::Controller controller;

extern pros::adi::DigitalOut intakePiston;
extern pros::Motor indexerMotor;
extern pros::Motor frontRoller;
extern pros::Motor middleRoller;
extern pros::Motor bottomRoller;
extern Intake intake;

extern pros::adi::DigitalOut flapperPiston;

extern pros::Optical colorSort;
extern pros::Imu imu;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;
extern pros::Distance distance;