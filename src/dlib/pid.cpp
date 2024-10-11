#include "api.h"
#include "dlib/pid.hpp"
#include <algorithm>

namespace dlib {

// Construct a PID controller
PID::PID(
    PIDGains gains_settings, 
    uint32_t interval_setting) {
    double interval_in_sec = interval_setting / 1000.0;
    
    PIDGains real_gains({
        gains_settings.kp, 
        gains_settings.ki * interval_in_sec, 
        gains_settings.kd / interval_in_sec
    });
    
    gains = real_gains;
    interval = interval_setting;
}

// Reset anything that needs to be reset between movements
void PID::reset() {
    p = 0;
    i = 0;
    d = 0;

    last_error = 0;
    last_time     = 0;
    last_output   = 0;
}

// Get the PID output in millivolts
double PID::update(double error) {
    uint32_t time = pros::millis();

    // return the previous output if any amount of time less than the interval has elapsed
    if (time - last_time < interval) {
        return last_output;
    }

    // calculate PID terms
    // no need to multiply/divide by delta time for kI and kD becayse they are already 
    // adjusted based on interval in the PID constructor
    p = error * gains.kp;
    i = i + error * gains.ki;
    d = (error - last_error) * gains.kd;

    // scale the output and limit it to [-12000, 12000] millivolts
    double output_scale = 100;
    double output = std::clamp((p + i + d) * output_scale, -12000.0, 12000.0);

    // update PID state
    last_error    = error;
    last_time     = time;
    last_output   = output;

    return output;
}

// Get the gains
PIDGains PID::get_gains() {
    return gains;
}

// Set the gains
void PID::set_gains(PIDGains new_gains) {
    gains = new_gains;
}

// Get the interval in ms
uint32_t PID::get_interval() {
    return interval;
}

// Get the current error
double PID::get_error() {
    return last_error;
}

}