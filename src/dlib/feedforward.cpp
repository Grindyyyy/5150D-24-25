#include "dlib/feedforward.hpp"
#include "api.h"
#include <cmath>

namespace dlib {

// Construct a FeedForward controller
FeedForward::FeedForward(
    FFGains gains_settings) {
    
    FFGains real_gains({
        gains_settings.ka, 
        gains_settings.ks,
        gains_settings.kv
    });
    
    gains = real_gains;
}

double FeedForward::calculate(double current_velocity){
    double current_time = pros::millis();
    double acceleration = (current_velocity - last_velocity) / (current_time - last_time);

    last_velocity = current_velocity;
    last_time = current_time;

    return(
        std::copysign(gains.ks, current_velocity) + 
        gains.kv * current_velocity + 
        gains.ka * acceleration
    );
}

void FeedForward::reset(){
    last_velocity = 0;
    last_time = 0;
}

}