#pragma once
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <cmath>

class Intake {
    public:

        Intake(pros::Motor& indexerMotor, pros::Optical& colorSort , pros::Motor& topRoller, pros::Motor& middleRoller, pros::Motor& bottomRoller);
        enum IntakeState{ STOPPED , INTAKING , OUTTAKE, SEPARATE, TOPSCORING, LOWSCORING , LOWSCORE_DELAY, FULLTOP, ROLLERONLY};
        enum Ball { BLUE , RED , NONE };

        void intakeControl();
        void lowScoring();
        void set(IntakeState state, int speed = 127);
        void setSeparation(Ball ball);
        // void checkForSort();  // DISABLED: Color sorting not used
        bool checkForDelay();
        bool checkIfTopFull();  // Check if top intake is full via optical sensor

        pros::Optical& colorSort;
        pros::Motor& frontRoller;
        pros::Motor& middleRoller;
        pros::Motor& bottomRoller;
        pros::Motor& indexerMotor;
        IntakeState state = STOPPED;
        Ball ball = NONE;

        bool sort  = false;  // DISABLED: Color sorting not currently used
        int speed = 127;

        double INITIAL_POSITION = 0;
        double SEPARATION_MOVEMENT = 30;
        double SEPARATION_WAIT = 0;
        double TIME_TO_COMPLETE_SEP = 100; //45 before


        int waitTime = 0;
        int delaying = 0;
        int separationTimeout = 0; // Timeout counter for separation
};