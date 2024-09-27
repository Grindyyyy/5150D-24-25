#include "api.h"
#include <initializer_list>

namespace dlib {
struct IMU {
    pros::Imu imu;

    IMU(std::int8_t imu_port)
     : imu(imu_port) {};
};
}