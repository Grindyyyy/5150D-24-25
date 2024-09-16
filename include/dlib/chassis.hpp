#include "api.h"
#include <initializer_list>

namespace dlib {
struct Chassis {
    pros::MotorGroup left;
    pros::MotorGroup right;
    double wheel_diameter;
    double gear_ratio;

    Chassis(std::initializer_list<int8_t> left_ports,
            std::initializer_list<int8_t> right_ports,
            double wheel_diameter,
            double gear_ratio
    ) : left(left_ports), right(right_ports), wheel_diameter(wheel_diameter), gear_ratio(gear_ratio) {};
};
}