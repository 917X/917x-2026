#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/rotation.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/localizer.hpp"
#include "subsystems/profiler.hpp"

// externs devices
extern Drive chassis;
extern pros::Controller controller;
extern MotionProfiler leverProfiler;

extern pros::Motor indexerMotor;
extern pros::Motor intakeMotor;
extern pros::Motor leverMotor;
extern Intake intake;

extern ez::Piston loaderPiston;
extern ez::Piston flapperPiston;
extern ez::Piston liftPiston;

extern pros::Imu imu;
extern pros::Distance leftDistance;
extern pros::Distance rightDistance;
extern pros::Optical colorSensor;
extern pros::Rotation leverRotation;

extern Localizer localizer;

/**
 * @brief Sets the default PID constants for the chassis
 *
 */
void default_constants();