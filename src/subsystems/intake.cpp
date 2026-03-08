#include "subsystems/intake.hpp"
#include "okapi/impl/util/timer.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <ctime>

Intake::Intake(pros::Motor &rollerMotor, pros::Motor &indexerMotor, pros::Optical* colorSensor)
	: rollerMotor(rollerMotor), indexerMotor(indexerMotor), colorSensor(colorSensor) {}

void Intake::intakeControl() {
	while (true) {
		switch (state) {
		case STOP:
			indexerMotor.move(-50);
			rollerMotor.move(0);
			break;
		case INTAKE:
			indexerMotor.move(-50);
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

void Intake::waitUntilColor(int hue1, int hue2, double saturation, int proximity, int timeout) {
    if (colorSensor == nullptr) { return; }
    okapi::Timer timer;
    while (timer.getDtFromStart() < timeout * okapi::millisecond) {
        int hue = colorSensor->get_hue();
        double sat = colorSensor->get_saturation();  // Changed to double
        int prox = colorSensor->get_proximity();
        if (sat >= saturation && (hue >= hue1 && hue <= hue2) && prox >= proximity) {
            return;  // All conditions met
        }
        pros::delay(50);
    }
}