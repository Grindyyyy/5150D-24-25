#pragma once
#include <iostream>
#include <cmath>

namespace dlib {

struct Setpoint {
    double position;
    double velocity;

    Setpoint(double position, double velocity) : 
    position(position), velocity(velocity) {};
};

class TrapMotionProfile {
    private:

        double maxAccel;
        double maxVelo;
        double totalDistance;

        double totalTime;
        
        double accel_time;
        double accel_distance;

        double coast_distance = totalDistance - (0.5*std::pow(accel_time,2));
        double coast_time = coast_distance / maxVelo;

        Setpoint setpoint = {0,0};
    public:
        // pass Max accel & Max velo for constructor
        TrapMotionProfile(double maxAcceleration, double maxVelocity, double totalDistance);

        // trap Velocity profile
        Setpoint calculate(double curTime);
        // should return position eventually

};
}