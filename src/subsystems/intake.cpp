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
			indexerMotor.move(0);
			rollerMotor.move(0);
			break;
		case INTAKE:
			indexerMotor.move(0);
			rollerMotor.move(speed);
			break;
			break;
		case OUTTAKE:
			indexerMotor.move(-speed);
			rollerMotor.move(-speed);
			break;
		case SCORE:
            indexerMotor.move(speed);
            rollerMotor.move(speed);
            break;
		}
        pros::delay(10);
	}
}

void Intake::set(IntakeState state, int speed) {
	// state setting
	this->state = state;
	this->speed = speed;
}