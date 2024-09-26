#pragma once
#include "api.h"
#include "pros/motors.hpp"

namespace dlib {

struct Position {
    double x = 0;
    double y = 0;
    double theta = 0;
};

class Odom {
    private:
        double wheel_diameter;
        double gear_ratio;
 
        double previous_forward = 0;
        double previous_theta = 0;
        Position position{};
        
    public:
        Odom(
            double wheel_diameter,
            double gear_ratio
        );

        void update(double current_forward, double current_theta);

        Position get_position();
        void set_position(Position new_position);

        double get_x();
        double get_y();
        std::unique_ptr<pros::Task> odom_updater;
        bool odom_started = false;
};

}