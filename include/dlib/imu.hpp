#include "api.h"
#include <initializer_list>

namespace dlib {
struct IMU {
    pros::Imu imu;
    std::unique_ptr<pros::Task> imu_task;
    double scalorMulx; //

    IMU(std::int8_t imu_port, double scalar)
     : imu(imu_port), scalorMulx(scalar) {};

    private:
        double driftRate = 0.0;
        double startAngle = 0.0;
         // might be spelled wrong x1 is default
        double driftAccum = 0.0; // total drift acculmation over run
        double correctedAngle = 0.0; //drift + scale corrected angle
    
    public:
        void setScalorMulx(double num) {
            scalorMulx = num;
        };

        double getCorrectedAngle() {
            return correctedAngle;
        };

        double getdriftAccum() {
            return driftAccum;
        };

        void main() {
            int prevTime = pros::millis();
            int millsDelay = 10;
            while (true) {
                // gets time when the loop starts
                prevTime = pros::millis();

                // adds driftRate to driftAccum as IMU drifts by pretty constant amount
                driftAccum += driftRate;

                // sets corrected angle with drift and scalor corrections. Note: accuracy is within 3 decimal places
                correctedAngle = static_cast<int>((imu.get_rotation() - driftAccum + startAngle) * scalorMulx * 1000) / 1000.0;

                // Makes sure the delay is exactly 10 ms bc calculations can take some time.
                millsDelay = (prevTime + 10) - pros::millis();
                pros::delay(millsDelay);
            }
        };

        void calibrateDrift() {
            int msCalTime = 1000; // Time to calculate drift = 1 sec
            int prevTime = 0;
            double curImuVal = 0;
            startAngle = imu.get_rotation();
            double prevImuVal = 0;
            double driftAcc = 0;
            int millsDelay = 10;
            std::vector<double> imuDifValues = {};

            // Loop keeps track of the drift within msCalTime
            while ((prevTime + 10) < msCalTime) { // added by 10 to account for delay
                // gets time when the loop starts
                prevTime = pros::millis();

                // gets IMU value and calculates difference between last IMU value
                // REQUIRED: Bot needs to be stable and non moving
                curImuVal = imu.get_rotation() - startAngle;
                imuDifValues.push_back(curImuVal - prevImuVal);

                // sets prevImuVal for drift correction
                prevImuVal = curImuVal;

                // Makes sure the delay is exactly 10 ms bc operations can take some time.
                millsDelay = (prevTime + 10) - pros::millis();
                pros::delay(millsDelay);
            }

            // adds all the drift values together
            for(int i = 0; i < imuDifValues.size(); i++) {
                driftAcc += imuDifValues[i];
            }

            // calculates driftRate from it & sets an inital correctedAngle
            driftRate = driftAcc/imuDifValues.size();
            correctedAngle = imu.get_rotation() - driftRate - startAngle;

            // starts the task for the Core Drift Correction Algo (START TASK HERE)
        };
};
}