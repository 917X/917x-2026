#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <cmath>

class Intake {
public:
  /**
   * @brief Construct a new Intake object
   *
   * @param indexerMotor
   * @param colorSort
   * @param topRoller
   * @param middleRoller
   * @param bottomRoller
   */
  Intake(pros::Motor &indexerMotor, pros::Optical &colorSort,
         pros::Motor &topRoller, pros::Motor &middleRoller,
         pros::Motor &bottomRoller);

  /**
   * @brief Enum for the different states of the intake system
   *
   */
  enum IntakeState {
    STOPPED,
    INTAKING,
    OUTTAKE,
    TOPSCORING,
    LOWSCORING,
    LOWSCORE_DELAY,
    ROLLERONLY
  };

  /**
   * @brief Enum for the different ball colors
   *
   */
  enum Ball { BLUE, RED, NONE };

  /**
   * @brief Main control loop for the intake system
   */
  void intakeControl();

  /**
   * @brief Set the state and speed of the intake system
   *
   * @param state
   * @param speed
   */
  void set(IntakeState state, int speed = 127);

  /**
   * @brief Set the color separation state
   *
   * @param ball
   */
  void setSeparation(Ball ball);

  /**
   * @brief Check for necessary intake delay
   *
   * @return true
   * @return false
   */
  bool checkForDelay();

  /**
   * @brief Check if theintake is full to prevent jamming via optical sensor
   *
   * @return true
   * @return false
   */
  bool checkIfTopFull();

private:
  // motors and sensors
  pros::Optical &colorSort;
  pros::Motor &frontRoller;
  pros::Motor &middleRoller;
  pros::Motor &bottomRoller;
  pros::Motor &indexerMotor;

  // intake state variables
  IntakeState state = STOPPED;
  Ball ball = NONE;
  bool sort = false;
  int speed = 127;

  // color separation variables
  double initialPosition = 0;
  double separationMovement = 30;
  double separationDelay = 0;
  double separationTime = 100;
  int waitTime = 0;
  int delaying = 0;
  int separationTimeout = 0;
};