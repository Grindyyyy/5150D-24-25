#include "api.h"
#include <initializer_list>

namespace dlib {
struct Sensor {
    pros::Optical ring_sensor;
    pros::c::optical_rgb_s_t rgb_value;

    Sensor(std::int8_t ring_sensor_port)
     : ring_sensor(ring_sensor_port) {};
};
}