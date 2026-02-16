#include "subsystems/intake.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <ctime>

Intake::Intake(pros::Motor &rollerMotor, pros::Motor &indexerMotor)
	: rollerMotor(rollerMotor), indexerMotor(indexerMotor) {}



void Intake::intakeControl() {
	while (true) {
		switch (state) {
		case STOP:
			indexerMotor.move(-10);
			rollerMotor.move(0);
			break;
		case INTAKE:
			indexerMotor.move(-45);
			rollerMotor.move(bottomSpeed);
			break;
		case OUTTAKE:
			indexerMotor.move(-topSpeed);
			rollerMotor.move(-bottomSpeed);
			break;
		case SCORE:
            indexerMotor.move(topSpeed);
            rollerMotor.move(bottomSpeed);
            break;
		}
        pros::delay(10);
	}
}

void Intake::set(IntakeState state, int speed) {
	// state setting
	this->state = state;
	this->topSpeed = speed;
    this->bottomSpeed = speed;
}

void Intake::set(IntakeState state, int topSpeed, int bottomSpeed) {
    // state setting
    this->state = state;
    this->topSpeed = topSpeed;
    this->bottomSpeed = bottomSpeed;
}