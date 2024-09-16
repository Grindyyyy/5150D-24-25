#include "api.h"
#include "dlib/chassis.hpp"
#include "dlib/intake.hpp"
#include "dlib/pid.hpp"
#include "dlib/odom.hpp"
#include <concepts>

namespace dlib {

template<typename Robot>
void brake_motors(Robot& robot)  {
    robot.get_chassis().left->brake();
    robot.get_chassis().right->brake();
}

template<typename Robot>
void move_voltage(Robot& robot, std::int32_t power) {
    robot.get_chassis().left->move(power);
    robot.get_chassis().right->move(power);
}

// Turn at a given voltage
template<typename Robot>
void turn_voltage(Robot& robot, std::int32_t power) {
    robot.get_chassis().left->move(-power);
    robot.get_chassis().right->move(power);
}

// Drive using the arcade scheme
template<typename Robot>
void arcade(Robot& robot, std::int8_t power, std::int8_t turn) {
    robot.get_chassis().left->move(power + turn);
    robot.get_chassis().right->move(power - turn);
}

// Constrain the angle to -180 to 180 for efficient turns
inline double angle_within_180(double degrees){
    degrees = std::fmod(degrees, 360);

    if(degrees > 180){
        degrees -= 360;
    }

    return degrees;
}

// Get the current heading from -180 to 180 degrees
template<typename Robot>
double get_imu_heading(Robot& robot) {
    // Convert heading to counterclockwise
    double ccw = std::fmod(360.0 - robot.get_imu().get_heading(), 360.0);
    return angle_within_180(ccw);
}


// Get the average inches the drivebase motors have moved
template<typename Robot>
double get_motor_inches(Robot& robot) {
    double sum = 0;

    auto left_positions = robot.get_chassis().left->get_position_all();
    auto right_positions = robot.get_chassis().right->get_position_all();

    for(int i = 0; i < left_positions.size(); i++) {
        sum += left_positions.at(i);
    }

    for(int i = 0; i < right_positions.size(); i++) {
        sum += right_positions.at(i);
    }

    double degrees = sum / (left_positions.size() + right_positions.size());
    double rotations = degrees / 360.0;
    double inches = rotations * robot.get_chassis().wheel_diameter * robot.get_chassis().gear_ratio * M_PI;
    return inches;
};

template<typename Robot>
void move_inches(Robot& robot, double inches, Options options) {
    long interval = robot.get_drive_pid().get_interval();

    double starting_inches = get_motor_inches(robot);
    double target_inches = starting_inches + inches;
    robot.get_drive_pid().reset();

    uint32_t starting_time = pros::millis();
    
    bool is_settling = false;
    uint32_t settle_start = 0;
    
    while (true) {
        uint32_t current_time = pros::millis();

        if (current_time - starting_time > options.max_ms) {
            break;
        }

        if (!is_settling && std::abs(robot.get_drive_pid().get_error()) < options.error_threshold) {
            is_settling = true;
            settle_start = pros::millis();
        }

        if (is_settling) {
            if (std::abs(robot.get_drive_pid().get_error()) < options.error_threshold) {
                if(current_time - settle_start > options.settle_ms) {
                    break;
                }
            } else {
                is_settling = false;
            }
        }

        double current_inches = get_motor_inches(robot);
        double output_voltage = robot.get_drive_pid().update(target_inches - current_inches);

        move_voltage(robot, output_voltage);

        pros::delay(interval);
    }

    brake_motors(robot);
}

// Turn to a given absolute angle using PID
template<typename Robot>
void turn_degrees(Robot& robot, double angle, Options options) {
    long interval = robot.get_turn_pid().get_interval();
    double target_angle = angle;

    uint32_t starting_time = pros::millis();
    
    bool is_settling = false;
    uint32_t settle_start = 0;

    while (true) {
        uint32_t current_time = pros::millis();
        uint32_t running_time = current_time - starting_time;

        pros::lcd::print(6, "Time (ms): %lu", running_time);
        if (current_time - starting_time > options.max_ms) {
            break;
        }

        if (!is_settling && std::abs(robot.get_turn_pid().get_error()) < options.error_threshold) {
            is_settling = true;
            settle_start = pros::millis();
        }
        
        if (is_settling) {
            if (std::abs(robot.get_turn_pid().get_error()) < options.error_threshold) {
                if(current_time - settle_start > options.settle_ms) {
                    break;
                }
            } else {
                is_settling = false;
            }
        }

        double current_angle = get_imu_heading(robot);
        double output_voltage = robot.get_turn_pid().update(std::remainder(target_angle - current_angle, 360));

        turn_voltage(robot, output_voltage);

        pros::delay(interval);
    }

    brake(robot);
}

template<typename Robot>
void intake_fwd(Robot& robot, int8_t volts){
    robot.get_intake().intake->move(volts);
}

template<typename Robot>
void intake_bwd(Robot& robot, int8_t volts){
    robot.get_intake().intake->move(-volts);
}

}