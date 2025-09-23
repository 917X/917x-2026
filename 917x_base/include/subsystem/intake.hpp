#pragma once
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <cmath>

class Intake {
    public:

        Intake(pros::Motor& indexerMotor, pros::Optical& colorSort , pros::Motor& topRoller, pros::Motor& middleRoller, pros::Motor& bottomRoller);
        enum IntakeState{ STOPPED , INTAKING , OUTTAKE, SEPARATE, TOPSCORING, LOWSCORING };
        enum Ball { BLUE , RED , NONE };

        void intakeControl();
        void set(IntakeState state, int speed = 127);
        void setSeparation(Ball ball);
        void checkForSort();

    
        pros::Optical& colorSort;
        pros::Motor& frontRoller;
        pros::Motor& middleRoller;
        pros::Motor& bottomRoller;
        pros::Motor& indexerMotor;
        IntakeState state = STOPPED;
        Ball ball = NONE;

        bool sort  = false;
        int speed = 127;

        double INITIAL_POSITION = 0;
        double SEPARATION_MOVEMENT = 300;
        double SEPARATION_WAIT = 0;
        double TIME_TO_COMPLETE_SEP = 100; //45 before
};