#include "subsystems/intake.hpp"
#include "EZ-Template/piston.hpp"
#include "okapi/impl/util/timer.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <ctime>

Intake::Intake(pros::Motor &intakeMotor, pros::Motor &indexerMotor, MotionProfiler &leverProfiler, ez::Piston &hoodPiston, pros::Optical* colorSensor)
	: intakeMotor(intakeMotor), indexerMotor(indexerMotor), leverProfiler(leverProfiler), hoodPiston(hoodPiston), colorSensor(colorSensor) {}

void Intake::intakeControl() {
	while (true) {
		switch (state) {
			case STOP:
				if(isScoring && leverTarget == 0.0){intakeMotor.move(-40);}
				else{intakeMotor.move(0);}
				// rollerMotor.move(0); --- IGNORE ---
				break;
			case INTAKE:
				// indexerMotor.move(-50); --- IGNORE ---
				intakeMotor.move(topSpeed);
				isScoring = false;
				leverTarget = 0.0;
				// Only spin roller if lever is fully lowered (within 5 degrees of 0)
				if (leverProfiler.getRotation() <= 5.0) {
					intakeMotor.move(topSpeed);
				} else {
					if(isScoring){intakeMotor.move(topSpeed);}else{intakeMotor.move(0);}  // Roller stops until lever is down
				}
				break;
			case OUTTAKE:
				// indexerMotor.move(-topSpeed); --- IGNORE ---
				intakeMotor.move(-topSpeed);
				isScoring = false;
				leverTarget = 0.0;
				break;
			case SCORE:
				// indexerMotor.move(topSpeed); --- IGNORE ---

				// intakeMotor.move(80);
				if (!isScoring) {
					isScoring = true;
					leverTarget = 117.0;
					scoreSettleStartTime = 0;
					hoodPiston.set(true);
				}
				break;
		}
        if (isScoring) {
            if (leverTarget == 117.0 && leverProfiler.getSettled()) {
                if (state == SCORE) {
                    scoreSettleStartTime = pros::millis(); // wait until the user releases the button
                } else if (scoreSettleStartTime == 0) {
                    scoreSettleStartTime = pros::millis();
                } else if (pros::millis() - scoreSettleStartTime > 50) {
                    leverTarget = 0.0;
                    leverProfiler.forceResume();
                    this->state = OUTTAKE;
                    this->topSpeed = 50;
                }
            }
            if (leverTarget == 0.0) {
                if (scoreSettleStartTime != 0 && pros::millis() - scoreSettleStartTime > 100) {
                    hoodPiston.set(false);
                }
                if (leverProfiler.getSettled()) {
                    isScoring = false;
                    this->state = STOP;
                    this->topSpeed = 127;
                }
            }
        }


        bool activeHold = (state == SCORE);
        leverProfiler.stepTo(leverTarget, activeHold);
        pros::delay(10);
    }
}

void Intake::set(IntakeState state, int speed) {
    // state setting
    if (this->state != state) { // If changing states, force the profiler to reset its settled state
        if (state == SCORE) isScoring = false;
        leverProfiler.forceResume();
    }
    this->state = state;
    this->topSpeed = speed;
    this->bottomSpeed = speed;
}

void Intake::set(IntakeState state, int topSpeed, int bottomSpeed) {
    // state setting
    if (this->state != state) {
        if (state == SCORE) isScoring = false;
        leverProfiler.forceResume();
    }
    this->state = state;
    this->topSpeed = topSpeed;
    this->bottomSpeed = bottomSpeed;
}

void Intake::waitUntilColor(int hue1, int hue2, double saturation, int proximity, int timeout,int delay) {
    if (colorSensor == nullptr) { return; }
    okapi::Timer timer;
    while (timer.getDtFromStart() < timeout * okapi::millisecond) {
        int hue = colorSensor->get_hue();
        double sat = colorSensor->get_saturation();  // Changed to double
        int prox = colorSensor->get_proximity();
        if (sat >= saturation && (hue >= hue1 && hue <= hue2) && prox >= proximity) {
            return;  // All conditions met
        }
        pros::delay(delay);
    }
}