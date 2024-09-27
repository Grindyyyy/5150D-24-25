#include "api.h"
#include "pros/optical.h"
#include <initializer_list>

namespace dlib {
struct Sensor {
    pros::Optical ring_sensor;
    pros::c::optical_rgb_s_t ring_sensor_rgb_value;
    pros::Optical lift_sensor;
    pros::c::optical_rgb_s_t lift_sensor_rgb_value;

    Sensor(std::int8_t ring_sensor_port,
        std::int8_t lift_sensor_port)
     : ring_sensor(ring_sensor_port), lift_sensor(lift_sensor_port) {};
};
}