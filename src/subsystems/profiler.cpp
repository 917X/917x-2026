#include "subsystems/profiler.hpp"

MotionProfiler::MotionProfiler(pros::Motor *motor, pros::Rotation *rotation, double minPosition, double maxPosition) {
    this->motor = motor;
    this->rotation = rotation;
    this->minPosition = minPosition;
    this->maxPosition = maxPosition;
    this->motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void MotionProfiler::setProfile(std::vector<std::pair<double, double>> profile) {
    this->profile = profile;
}

double MotionProfiler::getRotation(){
    return this->rotation->get_position() / 100.0;
}

void MotionProfiler::stepTo(double targetPosition){
    if (targetPosition != currentTarget) {
        currentTarget = targetPosition;
        isSettled = false;
        previousError = targetPosition - this->getRotation();
    }

    if (isSettled) {
        motor->move(0);
        return;
    }

    double currentPosition = this->getRotation();
    
    if (targetPosition < minPosition) targetPosition = minPosition;
    if (targetPosition > maxPosition) targetPosition = maxPosition;

    double error = targetPosition - currentPosition;
    std::cout<<"Current Position: " << currentPosition << " | Target Position: " << targetPosition << " | Error: " << error << std::endl;
    
    bool crossedTarget = (error > 0 && previousError < 0) || (error < 0 && previousError > 0);

    if (std::abs(error) < 5.0 || crossedTarget) {
        std::cout<<"Target reached or crossed. Stopping motor."<<std::endl;
        isSettled = true;
        motor->move(0);
        return; 
    }
    
    previousError = error;

    double absTargetVelocity = 0.0;
    if (profile.empty()) {
        absTargetVelocity = 127.0;
    } else if (currentPosition <= profile.front().first) {
        absTargetVelocity = profile.front().second;
    } else if (currentPosition >= profile.back().first) {
        absTargetVelocity = profile.back().second;
    } else {
        for (size_t i = 0; i < profile.size() - 1; ++i) {
            if (currentPosition >= profile[i].first && currentPosition <= profile[i + 1].first) {
                double x1 = profile[i].first;
                double y1 = profile[i].second;
                double x2 = profile[i + 1].first;
                double y2 = profile[i + 1].second;
                absTargetVelocity = y1 + (y2 - y1) * (currentPosition - x1) / (x2 - x1);
                break;
            }
        }
    }
    
    double velocity = absTargetVelocity * (error > 0 ? 1.0 : -1.0);
    motor->move(velocity);
}
