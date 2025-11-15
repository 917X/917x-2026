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
	 * @brief Enum representing the field walls for localization
	 *
	 */
	enum class Corner { TL, TR, BL, BR };

    /**
     * @brief Enum representing the robot's heading for localization
     * 
     */
    enum class Heading { TOP, BOTTOM, LEFT, RIGHT };

	/**
	 * @brief Localizes the robot based on the specified wall
	 *
	 * @param wall The wall to localize against
	 * @return std::pair<double, double> The (x, y) coordinates of the robot
	 * w.r.t the field
	 */
	std::pair<double, double> localize(Corner corner, Heading heading);

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