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

std::pair<double, double> Localizer::localize(Corner Corner, Heading heading) {
    double x = std::nan("");
    double y = std::nan("");
    // switch (Corner) {
    //     case Corner::TOP:
    //         switch (heading) {
    //             case Heading::TOP:
    //                 y = 71 - frontRangefinder->get() - frontOffset;
    //                 break;
    //             case Heading::BOTTOM:
    //                 y = 71 - backRangefinder->get() - backOffset;
    //                 break;
    //             case Heading::LEFT:
    //                 y = 71 - rightRangefinder->get() - leftOffset;
    //                 break;
    //             case Heading::RIGHT:
    //                 y = 71 - leftRangefinder->get() - rightOffset;
    //                 break;
    //             default:
    //                 break;
    //         }
    //         break;
    //     case Corner::BOTTOM:
    //         switch (heading) {
    //             case Heading::TOP:
    //                 y = -71 + backRangefinder->get() + backOffset;
    //                 break;
    //             case Heading::BOTTOM:
    //                 y = -71 + frontRangefinder->get() + frontOffset;
    //                 break;
    //             case Heading::LEFT:
    //                 y = -71 + leftRangefinder->get() + rightOffset;
    //                 break;
    //             case Heading::RIGHT:
    //                 y = -71 + rightRangefinder->get() + leftOffset;
    //                 break;
    //             default:
    //                 break;
    //         };
    //         break;
    //     }
    return std::make_pair(0,0);
}

            