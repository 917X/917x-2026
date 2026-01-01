#include "subsystems/localizer.hpp"
#include "okapi/api/units/QLength.hpp"
#include <numeric>
using namespace okapi::literals;

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

okapi::QLength Localizer::localize(Corner corner) {
    switch (corner) {
        case Corner::TL:
            return (71 - mmToInches(rightRangefinder->get()) - leftOffset)*1_in;
            break;
        case Corner::TR:
            return (71 - mmToInches(leftRangefinder->get()) - rightOffset)*1_in;
            break;
        case Corner::BL:
            return (-71 + mmToInches(leftRangefinder->get()) + leftOffset)*1_in;
            break;
        case Corner::BR:
            return (-71 + mmToInches(rightRangefinder->get()) + rightOffset)*1_in;
            break;
    }
    return 0_in; 
}

okapi::QLength Localizer::get_localized_coordinate(Corner corner){
    std::vector<okapi::QLength> localizer_values;
    for (int i = 0; i < 10; i++) {
        localizer_values.push_back(localize(corner));
        pros::delay(50);
    }
    okapi::QLength sum = std::accumulate(localizer_values.begin(), localizer_values.end(),0.0_in);
    return sum/localizer_values.size();
}
            