#include "api.h"
#include "dlib/odom.hpp"
#include "dlib/chassis.hpp"
#include <mutex>

namespace dlib {

// Initializer class
Odom::Odom() {
};

// Update global x, y, and theta
void Odom::update(double current_forward, double current_theta) {
    std::lock_guard<pros::Mutex> guard(mutex);
    
    double delta_forward = current_forward - previous_forward;
    double delta_theta = current_theta - previous_theta;

    double local_x = delta_forward;
    double local_y = 0; // no horizontal tracking wheel so this will always be 0

    // if delta_theta is 0, the robot moved in a straight line
    if (delta_theta != 0) {
        // arc_to_line * radius = chord_length
        double arc_to_line = 2 * std::sin(delta_theta / 2);
        local_x = arc_to_line * (local_x / delta_theta);
        local_y = arc_to_line * (local_y / delta_theta);
    }

    // the average heading of the movement, simplified from (self.theta + (self.theta + delta_theta)) / 2
    double average_theta = position.theta + delta_theta / 2;

    // rotate the local movement into the global frame
    double global_x = position.x + (local_x * std::cos(average_theta) - local_y * std::sin(average_theta));
    double global_y = position.y + (local_y * std::cos(average_theta) + local_x * std::sin(average_theta));
    double global_theta = position.theta + delta_theta;

    // update odometry state
    previous_forward = current_forward;
    previous_theta = current_theta;

    position = Position {global_x,global_y,global_theta};
}

Position Odom::get_position() {
    return position;
}



void Odom::set_position(Position new_position) {
    position = new_position;
}

// Return the x position
double Odom::get_x() {
    return position.x;
}

// Return the y position
double Odom::get_y() {
    return position.y;
}

}