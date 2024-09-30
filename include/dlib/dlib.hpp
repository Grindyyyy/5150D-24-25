#include "api.h"
#include "dlib/chassis.hpp"
#include "dlib/intake.hpp"
#include "dlib/pid.hpp"
#include "dlib/odom.hpp"
#include "dlib/mogo.hpp"
#include "dlib/indexer.hpp"
#include "dlib/imu.hpp"
#include "pros/motors.h"
#include "dlib/sensor.hpp"
#include "dlib/lift.hpp"
#include <concepts>

namespace dlib {

// ------------------------------ //
// Chassis
// ------------------------------ //

template<typename Robot>
void set_mode_brake(Robot& robot){
    robot.get_chassis().left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    robot.get_chassis().right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

template<typename Robot>
void set_mode_coast(Robot& robot){
    robot.get_chassis().left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    robot.get_chassis().right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

template<typename Robot>
void set_mode_hold(Robot& robot){
    robot.get_chassis().left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    robot.get_chassis().right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

template<typename Robot>
void calibrate(Robot& robot){
    robot.get_chassis().left.set_gearing(pros::E_MOTOR_GEAR_BLUE);
    robot.get_chassis().right.set_gearing(pros::E_MOTOR_GEAR_BLUE);

    robot.get_chassis().left.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    robot.get_chassis().right.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    robot.get_chassis().left.tare_position();
    robot.get_chassis().right.tare_position();

    robot.get_imu().imu.reset(true);
}

template<typename Robot>
void tare_position(Robot& robot){
    robot.get_chassis().left.tare_position();
    robot.get_chassis().right.tare_position();
}

template<typename Robot>
void brake_motors(Robot& robot)  {
    robot.get_chassis().left.brake();
    robot.get_chassis().right.brake();
}

template<typename Robot>
void move_voltage(Robot& robot, std::int32_t power) {
    robot.get_chassis().left.move(power);
    robot.get_chassis().right.move(power);
}
template<typename Robot>
void move_voltage(Robot& robot, std::int32_t left_volts, std::int32_t right_volts) {
    robot.get_chassis().left.move(left_volts);
    robot.get_chassis().right.move(right_volts);
}

// Turn at a given voltage
template<typename Robot>
void turn_voltage(Robot& robot, std::int32_t power) {
    robot.get_chassis().left.move(-power);
    robot.get_chassis().right.move(power);
}

// Drive using the arcade scheme
template<typename Robot>
void arcade(Robot& robot, std::int8_t power, std::int8_t turn) {
        robot.get_chassis().left.move((power - turn));
        robot.get_chassis().right.move(power + turn);
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
    double ccw = std::fmod(360.0 - robot.get_imu().imu.get_heading(), 360.0);
    return angle_within_180(ccw);
}

// Get the average inches the drivebase motors have moved
template<typename Robot>
double get_motor_inches(Robot& robot) {
    double sum = 0;

    auto left_positions = robot.get_chassis().left.get_position_all();
    auto right_positions = robot.get_chassis().right.get_position_all();

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
void update_odom(Robot& robot){
    while(true){
        robot.get_odom().update(get_motor_inches(robot), get_imu_heading(robot) * (M_PI / 180.00)); 
        pros::delay(10);
    }
}



template<typename Robot>
void start_odom_update_loop(Robot& robot) {
    // Capture the current class instance and call update_odom in a seperate task

    if (!robot.get_odom().odom_started) {
        robot.get_odom().odom_updater = std::make_unique<pros::Task>([&] { update_odom(robot); });
        robot.get_odom().odom_started = true;
    }
}



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
// Calculate the angle to turn from the current position to a point
double angle_to(Robot& robot, double x, double y, bool reverse){
    double target_x = x;
    double target_y = y;

    Position new_position = robot.get_odom().odom.get_position();
    double current_x = new_position.x;
    double current_y = new_position.y;

    double angle = std::atan2(target_y - current_y, target_x - current_x) * (180.0 / M_PI);

    // If reverse is true add 180 so that the bot faces the opposite direction
    if(reverse){
        angle += 180;
    }

    return angle_within_180(angle);
}

template<typename Robot>
// Calculate the distance from the current position to a given point
double dist_to(Robot& robot, double x, double y, bool reverse){
    double target_x = x;
    double target_y = y;

    Position new_position = robot.get_odom().odom.get_position();
    double current_x = new_position.x;
    double current_y = new_position.y;
    
    double dist = std::hypot(target_x - current_x, target_y - current_y);
    // The bot will move in the opposite direction
    if(reverse){
        dist = -dist;
    }

    return dist;
}

// Get the robot's current x, y and theta
template<typename Robot>
Position get_position(Robot& robot, bool radians) {
    Position position = robot.get_odom().get_position();

    if (!radians) {
        position.theta *= 180.0 / M_PI;
    }

    return position;
}

// Set the robot's current x, y and theta
template<typename Robot>
void set_position(Robot& robot, Position new_position, bool radians) {
    if (radians) {
        new_position.theta *= M_PI / 180.0;
    }

    robot.get_odom().odom.set_position(new_position);
}

template<typename Robot>
// Turn to a coordinate point using Odometry and PID
void turn_to(Robot& robot, double x, double y, bool reverse){
    double target_angle = angle_to(robot, x, y, reverse);
    turn_degrees(robot, target_angle);
}

template<typename Robot>
// Move to a coordinate point using Odometry and PID
void move_to(Robot& robot, double x, double y, bool reverse) {
    turn_to(robot, x, y, reverse);
    double dist = dist_to(robot, x, y, reverse);
    move_inches(robot, dist);
}

// ------------------------------ //
// Ring Sensor
// ------------------------------ //

template<typename Robot>
pros::c::optical_rgb_s_t intake_get_rgb_values(Robot& robot){
    return(robot.get_sensor().ring_sensor.get_rgb());
}

template<typename Robot>
double intake_get_red(Robot& robot){
    robot.get_sensor().ring_sensor_rgb_value = intake_get_rgb_values(robot);
    return(robot.get_sensor().ring_sensor_rgb_value.red);
}

template<typename Robot>
double intake_get_green(Robot& robot){
    robot.get_sensor().ring_sensor_rgb_value = intake_get_rgb_values(robot);
    return(robot.get_sensor().ring_sensor_rgb_value.green);
}

template<typename Robot>
double intake_get_blue(Robot& robot){
    robot.get_sensor().ring_sensor_rgb_value = intake_get_rgb_values(robot);
    return(robot.get_sensor().ring_sensor_rgb_value.blue);
}

template<typename Robot>
void intake_activate_led(Robot& robot, int power){
    robot.get_sensor().ring_sensor.set_led_pwm(power);
}

// ------------------------------ //
// Intake 
// ------------------------------ //

// kinda experimental and maybe overthought
template<typename Robot>
void intake_filter_task(Robot& robot){
    while(true){
            if(robot.get_intake().is_red_alliance){
                    if(intake_get_blue(robot) > intake_get_red(robot)*1.2){
                        robot.get_intake().ring_detected = true;
                        pros::delay(170);
                        robot.get_intake().intake.move(127);
                        pros::delay(300);
                    }
                    else{
                        robot.get_intake().ring_detected = false;
                    }
                }
            else if(robot.get_intake().is_blue_alliance){
                if(intake_get_red(robot) > intake_get_blue(robot)*1.5){
                    robot.get_intake().ring_detected = true;
                    pros::delay(170);
                    robot.get_intake().intake.move(127);
                    pros::delay(300);
                }
                else{
                    robot.get_intake().ring_detected = false;
                }
            }
                
    }
}

template<typename Robot>
double get_intake_position(Robot& robot){
    return(robot.get_intake().intake.get_position());
}

template<typename Robot>
void experimental_intake_task(Robot& robot){
    while(true){
            if(robot.get_intake().is_red_alliance){
                    if(intake_get_blue(robot) > intake_get_red(robot)*1.2 && robot.get_intake().ring_flung != false){
                        robot.get_intake().ring_detected = true;
                        robot.get_intake().ring_flung = false;
                        robot.get_intake().initial_position = robot.get_intake().intake.get_position();
                    }
                    else if(intake_get_blue(robot) > intake_get_red(robot)*1.2){
                        robot.get_intake().current_position = robot.get_intake().intake.get_position();
                        if(robot.get_intake().current_position - robot.get_intake().initial_position > 125){
                            robot.get_intake().intake.move(127);
                            pros::delay(200);
                            robot.get_intake().ring_flung = true;
                        }
                    }
                    else{
                        robot.get_intake().ring_detected = false;
                    }
                }
            else if(robot.get_intake().is_blue_alliance){
                if(intake_get_red(robot) > intake_get_blue(robot)*1.5){
                    robot.get_intake().ring_detected = true;
                    pros::delay(170);
                    robot.get_intake().intake.move(127);
                    pros::delay(300);
                }
                else{
                    robot.get_intake().ring_detected = false;
                }
            }
                
    }
}

template<typename Robot>
void start_intake_update_loop(Robot& robot) {
    if(!robot.get_intake().intake_task_started) {
        robot.get_intake().intake_updater = std::make_unique<pros::Task>([&] { experimental_intake_task(robot); });
        robot.get_intake().intake_task_started = true;
    }
}

template<typename Robot>
bool get_ring_detected(Robot& robot){
    return(robot.get_intake().ring_detected);
}

template<typename Robot>
double get_torque_intake(Robot& robot){
    return robot.get_intake().intake.get_torque();
}

template<typename Robot>
void intake_move(Robot& robot, int volts){
    robot.get_intake().intake.move(volts);
}

template<typename Robot>
void intake_move_torque(Robot& robot, int volts){
    if(robot.get_intake.intake.get_torque()){
        robot.get_intake().intake.move(volts);
    }
}

template<typename Robot>
bool get_red_alliance(Robot& robot){
    robot.get_intake().is_red_alliance = true;
    robot.get_intake().is_blue_alliance = false;
    return(robot.get_intake().is_red_alliance);
}
template<typename Robot>
bool get_blue_alliance(Robot& robot){
    robot.get_intake().is_blue_alliance = true;
    robot.get_intake().is_red_alliance = false;
    return(robot.get_intake().is_blue_alliance);
}

template<typename Robot>
void intake_stop(Robot& robot){
    robot.get_intake().intake.brake();
}

// ------------------------------ //
// Mogo
// ------------------------------ //

template<typename Robot>
void mogo(Robot& robot, bool state){
    robot.get_mogo().piston_1.set_value(state);
}

template<typename Robot>
void toggle_mogo(Robot& robot){
    robot.get_mogo().toggle_mode = !robot.get_mogo().toggle_mode;
}

template<typename Robot>
bool get_mogo_mode(Robot& robot){
    return(robot.get_mogo().toggle_mode);
}

// ------------------------------ //
// Indexer
// ------------------------------ //

template<typename Robot>
void indexer(Robot& robot, bool state){
    robot.get_indexer().piston_1.set_value(state);
}

template<typename Robot>
void toggle_indexer(Robot& robot){
    robot.get_indexer().toggle_mode = !robot.get_indexer().toggle_mode;
}

template<typename Robot>
bool get_indexer_mode(Robot& robot){
    return(robot.get_indexer().toggle_mode);
}

// ------------------------------ //
// Lift Sensor
// ------------------------------ //

template<typename Robot>
pros::c::optical_rgb_s_t lift_get_rgb_values(Robot& robot){
    return(robot.get_sensor().ring_sensor.get_rgb());
}

template<typename Robot>
double lift_get_red(Robot& robot){
    robot.get_sensor().ring_sensor_rgb_value = lift_get_rgb_values(robot);
    return(robot.get_sensor().ring_sensor_rgb_value.red);
}

template<typename Robot>
double lift_get_green(Robot& robot){
    robot.get_sensor().ring_sensor_rgb_value = lift_get_rgb_values(robot);
    return(robot.get_sensor().ring_sensor_rgb_value.green);
}

template<typename Robot>
double lift_get_blue(Robot& robot){
    robot.get_sensor().ring_sensor_rgb_value = lift_get_rgb_values(robot);
    return(robot.get_sensor().ring_sensor_rgb_value.blue);
}

template<typename Robot>
void lift_activate_led(Robot& robot, uint8_t power){
    robot.get_sensor().ring_sensor.set_led_pwm(power);
}

// ------------------------------ //
// Lift
// ------------------------------ //

template<typename Robot>
void lift_move(Robot& robot, int8_t volts){
    robot.get_lift().lift.move(volts);
}

template<typename Robot>
double get_lift_rot(Robot& robot){
    return(robot.get_lift().lift_rot.get_position());
}

// ------------------------------ //
// Controls 
// ------------------------------ //

template<typename Robot>
pros::Controller get_controller(Robot& robot){
    return(robot.get_master().master);
}


}