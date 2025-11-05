#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "subsystems/intake.hpp"

// externs devices 
extern Drive chassis;
extern pros::Controller controller;

extern pros::Motor indexerMotor;
extern pros::Motor frontRoller;
extern pros::Motor middleRoller;
extern pros::Motor bottomRoller;
extern Intake intake;

extern ez::Piston intakePiston;
extern ez::Piston flapperPiston;

extern pros::Optical colorSort;
extern pros::Imu imu;
extern pros::Distance distance;

/**
 * @brief Sets the default PID constants for the chassis
 * 
 */
void default_constants();