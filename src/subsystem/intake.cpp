#include "subsystem/intake.hpp"
#include <ctime>
#include "pros/misc.hpp"
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

// DISABLED: Color sorting is not currently used
// void Intake::checkForSort() {
//     if (ball == Ball::NONE) {
//         sort = false;
//     } else if (ball == Ball::BLUE) {
//         if (colorSort.get_hue() > 200 && colorSort.get_hue() < 270) {
//             sort = true;
//             INITIAL_POSITION = middleRoller.get_position();
//         }
//     } else if (ball == Ball::RED) {
//         if (colorSort.get_hue() > 0 && colorSort.get_hue() < 20) {
//             sort = true;
//             INITIAL_POSITION = middleRoller.get_position();
//         }
//     }
// }

bool Intake::checkForDelay() {
    if (waitTime >= 50 && delaying == 0) {
        return true;
    }
    return false;
}

bool Intake::checkIfTopFull() {
    // Check if optical sensor detects a ball stuck near the middle roller
    // Proximity value above threshold indicates a ball is present
    return this->colorSort.get_proximity() > 150;
}
void Intake::intakeControl() {
    while (true) {
        pros::delay(10); 
        
        // DISABLED: Color sorting logic
        // if (!sort) {
        //     checkForSort();
        // }
        // bool COMPLETED_MOVEMENT = (INITIAL_POSITION + SEPARATION_MOVEMENT) < middleRoller.get_position();

        // if (sort && !COMPLETED_MOVEMENT) {
        //     set(IntakeState::SEPARATE, speed);
        //     continue;
        // } else if (sort && COMPLETED_MOVEMENT) {
        //     set(IntakeState::STOPPED);
        //     sort = false;
        //     continue;
        // }


        //LOWSCORE DELAY LOGIC
        if (state != LOWSCORING && state != LOWSCORE_DELAY) {
            waitTime += 1;
            delaying = 0;
        }
        
        if (state == LOWSCORE_DELAY) {
            delaying += 1;
            // 75*10 cuz 10 ms per loop iteration
            if (delaying >= 75) {
                state = LOWSCORING;
                delaying = 1;
                waitTime = 0;
            }
        }
        
        if (state == LOWSCORING) {
            waitTime = 0;
        }

        
        // Check if top intake is full during intaking
        bool topFull = (state == INTAKING) && checkIfTopFull();



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
                // Stop middle roller if top is full to prevent jamming
                middleRoller.move(topFull ? 0 : speed);
                bottomRoller.move(speed);
                break;
            case OUTTAKE:
                indexerMotor.move(-speed);
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
            case LOWSCORE_DELAY:
                indexerMotor.move(-speed);
                frontRoller.move(speed);
                middleRoller.move(-speed);
                bottomRoller.move(speed);
                break;
            case FULLTOP:
                indexerMotor.move(0);
                frontRoller.move(speed);
                middleRoller.move(0);
                bottomRoller.move(speed);
                break;
            case ROLLERONLY:
                frontRoller.move(speed);
                middleRoller.move(0);
                bottomRoller.move(0);
                indexerMotor.move(0);
                break;
        }
    }

    
}

void Intake::set(IntakeState state, int speed) {
    // Special handling for LOWSCORING state
    if (state == LOWSCORING) {
        // If we're currently in delay, don't override it
        if (this->state == LOWSCORE_DELAY) {
            this->speed = speed; // Update speed but keep the delay state
            return;
        }
        // If delay is needed (500ms+ gap), start with LOWSCORE_DELAY
        if (checkForDelay()) {
            this->state = LOWSCORE_DELAY;
            this->delaying = 1;
            this->speed = speed;
            return;
        }
    }
    
    // Normal state setting
    this->state = state;
    this->speed = speed;
}