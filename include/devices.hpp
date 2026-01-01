#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "subsystems/intake.hpp"
#include "subsystems//localizer.hpp"

// externs devices
extern Drive chassis;
extern pros::Controller controller;

extern pros::Motor indexerMotor;
extern pros::Motor rollerMotor;;
extern Intake intake;

extern ez::Piston loaderPiston;
extern ez::Piston flapperPiston;
extern ez::Piston liftPiston;

extern pros::Imu imu;
extern pros::Distance leftDistance;
extern pros::Distance rightDistance;

extern Localizer localizer;

/**
 * @brief Sets the default PID constants for the chassis
 *
 */
void default_constants();