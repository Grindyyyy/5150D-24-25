#pragma once
#include <iostream>

namespace dlib {

struct FFGains {
    double ka = 0;
    double ks = 0;
    double kv = 0;
};

class FeedForward {
    private:
        FFGains gains;

        double last_velocity;
        double last_time;

    public:
        FeedForward(FFGains gains_settings);

        //feedforward
        double calculate(double velocity);

        FFGains get_gains();
        void set_gains(FFGains new_gains);
        void reset();

};
}