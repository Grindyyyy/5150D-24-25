#include "api.h"
#include <initializer_list>

namespace dlib {
struct Intake {
    pros::Motor intake;
    Intake(std::int8_t intake_port)
     : intake(intake_port) {};
};
}