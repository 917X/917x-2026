#pragma once

#include "EZ-Template/piston.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "subsystems/profiler.hpp"
#include <cmath>
#include <cstdint>

class Intake {
  public:
	/**
	 * @brief Construct a new Intake object
	 *
	 * @param intakeMotor
	 * @param indexerMotor
     * @param leverProfiler
	 * @param hoodPiston
	 * @param colorSensor
	 */
	Intake(pros::Motor &intakeMotor, pros::Motor &indexerMotor, MotionProfiler &leverProfiler, ez::Piston &hoodPiston, pros::Optical* colorSensor = nullptr);

	/**
	 * @brief Enum for the different states of the intake system
	 *
	 */
	enum IntakeState { STOP, INTAKE, OUTTAKE, SCORE, AUTON_SCORING, AUTON_SCORING_HOLD };

	/**
	 * @brief Main control loop for the intake system
	 */
	void intakeControl();

	/**
	 * @brief Set the state and overall speed of the intake system
	 *
	 * @param state
	 * @param speed
	 */
	void set(IntakeState state, int speed = 127);

    /**
     * @brief Set the state, top speed, and bottom speed of the intake system
     * 
     * @param state 
     * @param topSpeed 
     * @param bottomSpeed 
     */
    void set(IntakeState state, int topSpeed, int bottomSpeed);

    void waitUntilColor(int hue1, int hue2, double saturation, int proximity, int timeout, int delay = 50);

     pros::Optical *colorSensor;
  private:
	// motors and sensors
	pros::Motor &intakeMotor;
	pros::Motor &indexerMotor;
    MotionProfiler &leverProfiler;
    ez::Piston &hoodPiston;

	// intake state variables
	IntakeState state = STOP;
    double leverTarget = 0.0;
	int topSpeed = 127;
	int bottomSpeed = 127;
    bool isScoring = false;
	bool IGNORE_HOOD = false; // Used to ignore hood piston control during auton scoring sequence
    uint32_t scoreSettleStartTime = 0;
	uint32_t intakeStartTime = 0;
};