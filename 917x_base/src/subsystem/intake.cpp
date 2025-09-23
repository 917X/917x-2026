#include "subsystem/intake.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

Intake::Intake(pros::Motor& indexerMotor, pros::Optical& colorSort, pros::Motor& topRoller, pros::Motor& middleRoller, pros::Motor& bottomRoller)
    : indexerMotor(indexerMotor), colorSort(colorSort), frontRoller(topRoller), middleRoller(middleRoller), bottomRoller(bottomRoller) {
    this->state = IntakeState::STOPPED;
    this->ball = Ball::NONE;
    this->speed = 100;
    this->sort = false;
    this->INITIAL_POSITION = 0;
}

void Intake::setSeparation(Ball ball) {
    this->ball = ball;
}

void Intake::checkForSort() {
    if (ball == Ball::NONE) {
        sort = false;
    } else if (ball == Ball::BLUE) {
        if (colorSort.get_hue() > 200 && colorSort.get_hue() < 270) {
            sort = true;
            INITIAL_POSITION = frontRoller.get_position();
        }
    } else if (ball == Ball::RED) {
        if (colorSort.get_hue() > 0 && colorSort.get_hue() < 40) {
            sort = true;
            INITIAL_POSITION = frontRoller.get_position();
        }
    }


}

void Intake::intakeControl() {
    while (true) {
        if (!sort) {
            checkForSort();
        }
        bool COMPLETED_MOVEMENT = (INITIAL_POSITION + SEPARATION_MOVEMENT) < frontRoller.get_position();

        if (sort && !COMPLETED_MOVEMENT) {
            set(IntakeState::SEPARATE, speed);
            continue;
        } else if (sort && COMPLETED_MOVEMENT) {
            set(IntakeState::STOPPED);
            sort = false;
            continue;
        }


        switch (state) {
            case STOPPED:
                indexerMotor.move(0);
                frontRoller.move(0);
                middleRoller.move(0);
                bottomRoller.move(0);
                break;
            case INTAKING:
                indexerMotor.move(0);
                frontRoller.move(speed);
                middleRoller.move(speed);
                bottomRoller.move(speed);
                break;
            case OUTTAKE:
                indexerMotor.move(0);
                frontRoller.move(-speed);
                middleRoller.move(-speed);
                bottomRoller.move(-speed);
                break;
            case SEPARATE:
                indexerMotor.move(0);
                frontRoller.move(speed);
                middleRoller.move(-speed);
                bottomRoller.move(speed);
                break;
            case TOPSCORING:
                indexerMotor.move(speed);
                frontRoller.move(speed);
                middleRoller.move(speed);
                bottomRoller.move(speed);
                break;
            case LOWSCORING:
                indexerMotor.move(-speed);
                frontRoller.move(speed);
                middleRoller.move(speed);
                bottomRoller.move(speed);
                break;
        }
    }
    
}

void Intake::set(IntakeState state, int speed) {
    this->state = state;
    this->speed = speed;
}