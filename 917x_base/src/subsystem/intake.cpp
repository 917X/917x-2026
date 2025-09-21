// #include "subsystem/intake.hpp"
// #include "pros/motors.hpp"
// #include "pros/optical.hpp"

// Intake::Intake(pros::Motor& indexerMotor, pros::Optical& colorSort, pros::Motor& topRoller, pros::Motor& middleRoller, pros::Motor& bottomRoller)
//     : IndexerMotor(indexerMotor), colorSort(colorSort), topRoller(topRoller), middleRoller(middleRoller), bottomRoller(bottomRoller) {
//     this->state = IntakeState::STOPPED;
//     this->ball = Ball::NONE;
//     this->speed = 100;
//     this->sort = false;
//     this->INITIAL_POSITION = 0;
// }

// void Intake::setSeparation(Ball ball) {
//     this->ball = ball;
// }

// void Intake::checkForSort() {
//     if (ball == Ball::NONE) {
//         sort = false;
//     } else if (ball == Ball::BLUE) {
//         if (colorSort.get_hue() > 200 && colorSort.get_hue() < 270) {
//             sort = true;
//             INITIAL_POSITION = topRoller.get_position();
//         }
//     }
// }

// void Intake::intakeControl() {
//     while (true) {
//         if (!sort) {
//             checkForSort();
//         }
//         bool COMPLETED_MOVEMENT = (INITIAL_POSITION + SEPARATION_MOVEMENT) < topRoller.get_position();

//         if (sort && !COMPLETED_MOVEMENT) {
//             set(IntakeState::SEPARATE, speed);
//         } else if (sort && COMPLETED_MOVEMENT) {
//             set(IntakeState::STOPPED);
//             sort = false;
//         }
//     }

//     if (this->state == STOPPED) {
//         this->IndexerMotor.move(0);
//         this->bottomRoller.move(0);
//         this->middleRoller.move(0);
//         this->topRoller.move(0);
//     }
// }

// void Intake::set(IntakeState state, int speed) {
//     this->state = state;
//     this->speed = speed;
    
//     switch (state) {
//         case STOPPED:
//             IndexerMotor.move(0);
//             topRoller.move(0);
//             middleRoller.move(0);
//             bottomRoller.move(0);
//             break;
//         case INTAKING:
//             IndexerMotor.move(speed);
//             topRoller.move(speed);
//             middleRoller.move(speed);
//             bottomRoller.move(speed);
//             break;
//         case OUTTAKE:
//             IndexerMotor.move(-speed);
//             topRoller.move(-speed);
//             middleRoller.move(-speed);
//             bottomRoller.move(-speed);
//             break;
//         case SEPARATE:
//             IndexerMotor.move(0);
//             topRoller.move(speed);
//             middleRoller.move(0);
//             bottomRoller.move(0);
//             break;
//         case TOPSCORING:
//             IndexerMotor.move(speed);
//             topRoller.move(0);
//             middleRoller.move(0);
//             bottomRoller.move(0);
//             break;
//         case LOWSCORING:
//             IndexerMotor.move(0);
//             topRoller.move(0);
//             middleRoller.move(0);
//             bottomRoller.move(speed);
//             break;
//     }
// }