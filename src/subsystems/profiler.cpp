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

double MotionProfiler::compute(double targetPosition){
    double currentPosition = this->getRotation();
    if(currentPosition > targetPosition){
        return 0.0; 
    }

    // perform binary search to determine profile sgmt
    int left = 0;
    int right = profile.size() - 1;
    double targetVelocity = 0.0;
    while(left <= right){
        int mid = left + (right - left) / 2;
        if(profile[mid].first < targetPosition){
            targetVelocity = profile[mid].second;
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    // perform linearization
    if(left < profile.size() - 1){
        double x1 = profile[left].first;
        double y1 = profile[left].second;
        double x2 = profile[left + 1].first;
        double y2 = profile[left + 1].second;

        // linear interpolation
        targetVelocity = y1 + (y2 - y1) * (currentPosition - x1) / (x2 - x1);
    }
    
    return targetVelocity;
}
