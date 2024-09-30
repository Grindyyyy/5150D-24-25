#include "api.h"
#include <initializer_list>

namespace dlib {
struct Intake {
    pros::Motor intake;
    bool is_red_alliance;
    bool is_blue_alliance;
    bool intake_task_started = false;
    bool ring_detected;
    bool ring_flung = true;
    double initial_position;
    double current_position;
    std::unique_ptr<pros::Task> intake_updater;
    Intake(std::int8_t intake_port)
     : intake(intake_port) {};
};
}