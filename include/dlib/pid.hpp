#pragma once
#include <iostream>

namespace dlib {

struct PIDGains {
    double kp = 0;
    double ki = 0;
    double kd = 0;
};

class PID {
    private:
        PIDGains gains;
        double setpoint;
        uint32_t interval;

        double p;
        double i;
        double d;

        double last_error = 0;
        uint32_t last_time = 0;
        double last_output = 0;

    public:
        PID(PIDGains gains_settings, uint32_t interval_setting);

        double update(double error);
        void reset();

        PIDGains get_gains();
        void set_gains(PIDGains new_gains);

        uint32_t get_interval();

        double get_error();
};




// for options
struct Options {
    // the threshold for the error
    double error_threshold;
    uint32_t settle_ms;
    uint32_t max_ms;

    Options with_error_threshold(double threshold) {
        return { threshold, settle_ms, max_ms };
    }
    
    Options with_settle_ms(uint32_t time) {
        return { error_threshold, time, max_ms };
    }

    Options with_max_ms(uint32_t time) {
        return { error_threshold, settle_ms, time };
    }
};

}