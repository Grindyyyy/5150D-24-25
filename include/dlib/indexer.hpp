#include "api.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include <initializer_list>

namespace dlib {
struct Indexer {
    pros::adi::DigitalOut piston_1;
    bool init_state;
    bool toggle_mode = false;
    Indexer(char piston_1_port,
        bool init_state
    ) : piston_1(piston_1_port), init_state(init_state) {};
};
}