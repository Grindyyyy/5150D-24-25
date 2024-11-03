#include "api.h"
#include <initializer_list>

namespace dlib {
struct Lift {
    pros::Motor lift;
    pros::Rotation lift_rot;
    bool lift_task_started = false;
    pros::Mutex lift_mutex;
    std::unique_ptr<pros::Task> lift_updater;
    double current_pos;
    bool task_toggle = false;
    double range = 228;

    

    double pid_error;
    Lift(std::int8_t lift_port,
        std::int8_t lift_rot_port)
     : lift(lift_port), lift_rot(lift_rot_port) {};
};
}