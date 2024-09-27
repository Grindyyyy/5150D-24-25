#include "api.h"
#include <initializer_list>

namespace dlib {
struct Lift {
    pros::Motor lift;
    pros::Rotation lift_rot;

    Lift(std::int8_t lift_port,
        std::int8_t lift_rot_port)
     : lift(lift_port), lift_rot(lift_rot_port) {};
};
}