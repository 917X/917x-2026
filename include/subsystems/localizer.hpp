#pragma once
#include "pros/distance.hpp"
#include <cmath>

class Localizer {
  public:
	/**
	 * @brief Construct a new Localizer object
	 *
	 * @param leftRangefinder The left rangefinder sensor
	 * @param rightRangefinder The right rangefinder sensor
	 * @param backRangefinder The back rangefinder sensor
	 * @param frontRangefinder The front rangefinder sensor
	 * @param leftOffset The offset of the left rangefinder
	 * @param rightOffset The offset of the right rangefinder
	 * @param backOffset The offset of the back rangefinder
	 * @param frontOffset The offset of the front rangefinder
	 */
	Localizer(pros::Distance *leftRangefinder, pros::Distance *rightRangefinder,
			  pros::Distance *backRangefinder, pros::Distance *frontRangefinder,
			  double leftOffset, double rightOffset, double backOffset,
			  double frontOffset);
	/**
	 * @brief Enum representing the field corners for localization
	 *
	 */
	enum class Corner { TL, TR, BL, BR };

	/**
	 * @brief Localizes the robot based on the specified wall
	 *
	 * @param corner The corner that robot is loading from
	 * @return int The localized position value. Always Y value. 
	 */
	int localize(Corner corner);


    /**
     * @brief Converts millimeters to inches
     * 
     * @param mm 
     * @return double
     */
    double mmToInches(double mm);

  private:
	pros::Distance *leftRangefinder = nullptr;
	pros::Distance *rightRangefinder = nullptr;
	pros::Distance *backRangefinder = nullptr;
	pros::Distance *frontRangefinder = nullptr;
	double leftOffset = std::nan("");
	double rightOffset = std::nan("");
	double backOffset = std::nan("");
	double frontOffset = std::nan("");
};