#pragma once
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/rotation.hpp"

/**
 * @brief Motion profiling for one motor. 
 * Uses a piecewise linear motion profile.
 */

class MotionProfiler {
  public:
    /**
     * @brief Constructs a new MotionProfiler object
     * 
     */
    MotionProfiler(pros::Motor *motor, pros::Rotation *rotation, double minPosition, double maxPosition);

    /**
     * @brief Sets the motion profile for the motor. The profile is a vector of pairs, where each pair consists of a position (in degrees) and a velocity (in percentage).
     * 
     */
    void setProfile(std::vector<std::pair<double, double>> profile);

    /**
     * @brief Steps the motor towards the target position using the profile
     * 
     * @param targetPosition 
     */
    void stepTo(double targetPosition);

    /**
     * @brief gets the rotation sensor value
     * 
     */
    double getRotation();

  private:
    pros::Motor *motor = nullptr;
    pros::Rotation *rotation;
    double minPosition;
    double maxPosition;
    std::vector<std::pair<double, double>> profile; // pair of (position, velocity)
    double currentTarget = -9999.0;
    bool isSettled = false;
    double previousError = 0.0;
};