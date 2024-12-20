#include "api.h"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include <initializer_list>

namespace dlib {
struct Chassis {
    pros::MotorGroup left;
    pros::MotorGroup right;
    double wheel_diameter;
    double rpm;
    double maxVelo;
    double maxAccel;


    Chassis(std::initializer_list<int8_t> left_ports,
            std::initializer_list<int8_t> right_ports,
            double wheel_diameter,
            double rpm,
            double maxVelo,
            double maxAccel
    ) : left(left_ports), right(right_ports), wheel_diameter(wheel_diameter), rpm(rpm) , maxAccel(maxAccel), maxVelo(maxVelo) {};
};
}