#include "subsystems/localizer.hpp"

Localizer::Localizer(pros::Distance *leftRangefinder,
					 pros::Distance *rightRangefinder,
					 pros::Distance *backRangefinder,
					 pros::Distance *frontRangefinder, double leftOffset,
					 double rightOffset, double backOffset, double frontOffset)
	: leftRangefinder(leftRangefinder), rightRangefinder(rightRangefinder),
	  backRangefinder(backRangefinder), frontRangefinder(frontRangefinder),
	  leftOffset(leftOffset), rightOffset(rightOffset), backOffset(backOffset),
	  frontOffset(frontOffset) {}

double Localizer::mmToInches(double mm) {
    return mm / 25.4;
}

int Localizer::localize(Corner corner) {
    switch (corner) {
        case Corner::TL:
            return 71 - mmToInches(rightRangefinder->get()) - leftOffset;
            break;
        case Corner::TR:
            return 71 - mmToInches(leftRangefinder->get()) - rightOffset;
            break;
        case Corner::BL:
            return -71 + mmToInches(leftRangefinder->get()) + leftOffset;
            break;
        case Corner::BR:
            return -71 + mmToInches(rightRangefinder->get()) + rightOffset;
            break;
    }
    return 0; 
}

            