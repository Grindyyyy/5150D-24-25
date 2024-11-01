#include "api.h"
#include "dlib/chassis.hpp"
#include "dlib/intake.hpp"
#include "dlib/pid.hpp"
#include "dlib/odom.hpp"
#include "dlib/mogo.hpp"
#include "dlib/indexer.hpp"
#include "dlib/imu.hpp"
#include "dlib/trapMP.hpp"
#include "feedforward.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "dlib/sensor.hpp"
#include "dlib/lift.hpp"
#include "pros/motors.hpp"
#include "robodash/views/console.hpp"
#include <cmath>
#include <concepts>
#include <string>
#include <vector>

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
    robot.get_chassis().left.set_gearing_all(pros::E_MOTOR_GEAR_BLUE);
    robot.get_chassis().right.set_gearing_all(pros::E_MOTOR_GEAR_BLUE);

    robot.get_chassis().left.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    robot.get_chassis().right.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);


    robot.get_imu().imu.reset(true);

    robot.get_imu().calibrateDrift();
    robot.get_imu().imu_task = std::make_unique<pros::Task>([&] { robot.get_imu().main(); });

}


   // robot.get_chassis().left.tare_position_all();
   // robot.get_chassis().right.tare_position_all();

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
    robot.get_chassis().left.move_voltage(power);
    robot.get_chassis().right.move_voltage(power);
}

// Turn at a given voltage
template<typename Robot>
void turn_voltage(Robot& robot, std::int32_t power) {
    robot.get_chassis().left.move_voltage(-power);
    robot.get_chassis().right.move_voltage(power);
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

    double angle = std::fmod(robot.get_imu().imu.get_heading(), 360.0);
    return angle;
}

template<typename Robot>
double get_imu_heading_absolute(Robot& robot) {
    double angle = robot.get_imu().imu.get_heading();
    return angle;
}

template<typename Robot>
double get_imu_rotation(Robot& robot) {
    return robot.get_imu().imu.get_rotation();
}

inline double average(std::vector<double> values) {
    double sum = 0;

    for(auto value : values) {
        sum += value;
    }

    return sum / values.size();
}

// Get the average inches the drivebase motors have moved
template<typename Robot>
double get_motor_inches(Robot& robot) {
    auto diameter = robot.get_chassis().wheel_diameter;
    auto rpm = robot.get_chassis().rpm;

    std::vector<double> distances;

    auto left_gearsets = robot.get_chassis().left.get_gearing_all();
    auto left_positions = robot.get_chassis().left.get_position_all();
    
    for(int i = 0; i < left_positions.size(); i++) {
        double in;

        switch (left_gearsets[i]) {
            case pros::MotorGears::red: in = 100; break;
            case pros::MotorGears::green: in = 200; break;
            case pros::MotorGears::blue: in = 600; break;
            default: in = 200; break;
        }

        distances.push_back(left_positions[i] * (diameter * M_PI) * (rpm / in));
    }

    auto right_gearsets = robot.get_chassis().right.get_gearing_all();
    auto right_positions = robot.get_chassis().right.get_position_all();

    for(int i = 0; i < right_positions.size(); i++) {
        double in;

        switch (right_gearsets[i]) {
            case pros::MotorGears::red: in = 100; break;
            case pros::MotorGears::green: in = 200; break;
            case pros::MotorGears::blue: in = 600; break;
            default: in = 200; break;
        }

        distances.push_back(right_positions[i] * (diameter * M_PI) * (rpm / in));
    }

    return average(distances);
};

template<typename Robot>
void update_odom(Robot& robot){
    int prevTime = pros::millis();
    int millsDelay = 10;
    while(true){
        // gets time when the loop starts
        prevTime = pros::millis();

        // runs odom function
        robot.get_odom().update(get_motor_inches(robot), robot.get_imu().getCorrectedAngle() * (M_PI / 180.00)); 

        // Makes sure the delay is exactly 10 ms bc calculations can take some time.
        millsDelay = (prevTime + 10) - pros::millis();
        pros::delay(millsDelay);
    }
}

template<typename Robot>
void update_odom_alt(Robot& robot){
    while(true){
        robot.get_odom().update(get_motor_inches(robot), get_imu_heading_absolute(robot) * (M_PI / 180.00)); 
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
void start_odom_update_loop_alt(Robot& robot) {
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
            } 
            else {
                is_settling = false;
            }
        }

        double current_inches = get_motor_inches(robot);
        double error = target_inches - current_inches;
        double output_voltage = robot.get_drive_pid().update(error);
        
        if(std::abs(output_voltage) > options.max_voltage){
            if(output_voltage > 0){
                output_voltage = options.max_voltage;
            }
            else{
                output_voltage = -options.max_voltage;
            }
        }
        output_voltage += std::copysign(error, 900);

        move_voltage(robot, output_voltage);

        pros::delay(interval);
    }

    brake_motors(robot);
}

template<typename Robot>
void move_inches_ffwd(Robot& robot, double inches, Options options) {
    TrapMotionProfile move_profile = TrapMotionProfile(robot.get_chassis().maxAccel,robot.get_chassis().maxVelo, inches);
    long interval = robot.get_drive_pid().get_interval();

    double starting_inches = get_motor_inches(robot);
    double target_inches = starting_inches + inches;
    robot.get_drive_pid().reset();

    uint32_t starting_time = pros::millis();
    
    bool is_settling = false;
    uint32_t settle_start = 0;
    double error;
    
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
            } 
            else {
                is_settling = false;
            }
        }

        double current_inches = get_motor_inches(robot);
        error = target_inches - current_inches;
        double output_voltage = robot.get_drive_pid().update(error);


        //ffwd calc
        double cur_time = pros::millis();
        double elapsed_time = (cur_time - starting_time) / 1000;

        Setpoint setpoint = move_profile.calculate(elapsed_time / 1000);


        output_voltage += robot.get_drive_feed_forward().calculate(setpoint.velocity); // placeholder number
        // output voltage stuff
        if(std::abs(output_voltage) > options.max_voltage){
            if(output_voltage > 0){
                output_voltage = options.max_voltage;
            }
            else{
                output_voltage = -options.max_voltage;
            }
        }

        move_voltage(robot, output_voltage);

        pros::delay(interval);
    }

    std::cout << error << std::endl;
    brake_motors(robot);
}

// Turn to a given absolute angle using PID
template<typename Robot>
void turn_degrees(Robot& robot, double angle, const Options options) {
   long interval = robot.get_turn_pid().get_interval();

    double target_angle = angle;
    robot.get_turn_pid().reset();

    uint32_t starting_time = pros::millis();
    
    bool is_settling = false;
    uint32_t settle_start = 0;
    double error;
    while (true) {
        
        uint32_t current_time = pros::millis();

        if (current_time - starting_time > options.max_ms) {
            break;
        }

        if (!is_settling && std::abs(robot.get_turn_pid().get_error()) < options.error_threshold) {
            is_settling = true;

            settle_start = pros::millis();
        }

        double current_angle = robot.get_imu().getCorrectedAngle();
        error = target_angle - current_angle;

        if (is_settling) {
            if (std::abs(robot.get_turn_pid().get_error()) < options.error_threshold) {
                if(current_time - settle_start > options.settle_ms) {
                    break;
                }
            } 
            else {
                is_settling = false;
            }
        } 

        double output_voltage = robot.get_turn_pid().update(error);
        
        // output voltage stuff
        if(std::abs(output_voltage) > options.max_voltage){
            if(output_voltage > 0){
                output_voltage = options.max_voltage;
            }
            else{
                output_voltage = -options.max_voltage;
            }
        }

        turn_voltage(robot, output_voltage);

        pros::delay(interval);
    }

    brake_motors(robot);

    std::cout << error << std::endl;

    double current_angle = robot.get_imu().getCorrectedAngle();

}

// Turn to a given absolute angle using PID
template<typename Robot>
void turn_degrees_ffwd(Robot& robot, double angle, const Options options) {
//TrapMotionProfile move_profile = TrapMotionProfile(robot.get_chassis().maxAccel,robot.get_chassis().maxVelo, angle);
   long interval = robot.get_turn_pid().get_interval();

    double target_angle = angle;
    robot.get_turn_pid().reset();

    uint32_t starting_time = pros::millis();
    
    bool is_settling = false;
    uint32_t settle_start = 0;

    while (true) {
        
        uint32_t current_time = pros::millis();

        if (current_time - starting_time > options.max_ms) {
            break;
        }

        if (!is_settling && std::abs(robot.get_turn_pid().get_error()) < options.error_threshold) {
            is_settling = true;

            settle_start = pros::millis();
        }

        double current_angle = robot.get_imu().getCorrectedAngle();
        double error = target_angle - current_angle;

        if (is_settling) {
            if (std::abs(robot.get_turn_pid().get_error()) < options.error_threshold) {
                if(current_time - settle_start > options.settle_ms) {
                    break;
                }
            } 
            else {
                is_settling = false;
            }
        } 

        double output_voltage = robot.get_turn_pid().update(error);

        // ffwd calc
        //ffwd calc
        double cur_time = pros::millis();
        double elapsed_time = (cur_time - starting_time) / 1000;

        //Setpoint setpoint = move_profile.calculate(elapsed_time /1000);

        //output_voltage += robot.get_drive_feed_forward().calculate(setpoint.velocity); // placeholder number

        // output voltage stuff
        if(std::abs(output_voltage) > options.max_voltage){
            if(output_voltage > 0){
                output_voltage = options.max_voltage;
            }
            else{
                output_voltage = -options.max_voltage;
            }
        }

        turn_voltage(robot, output_voltage);

        pros::delay(interval);
    }

    brake_motors(robot);

    double current_angle = robot.get_imu().getCorrectedAngle();
}


template<typename Robot>
// Calculate the angle to turn from the current position to a point
double angle_to(Robot& robot, double x, double y, bool reverse){
    double target_x = x;
    double target_y = y;

    Position new_position = robot.get_odom().get_position();
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

    Position new_position = robot.get_odom().get_position();
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
void turn_to(Robot& robot, double x, double y, bool reverse, Options options){
    double target_angle = angle_to(robot, x, y, reverse);
    turn_degrees(robot, target_angle,  options);
}

template<typename Robot>
// Turn to a coordinate point using Odometry and PID
void turn_to_ffwd(Robot& robot, double x, double y, bool reverse, Options options){
    double target_angle = angle_to(robot, x, y, reverse);
    turn_degrees(robot, target_angle,  options);
}

template<typename Robot>
// Move to a coordinate point using Odometry and PID
void move_to(Robot& robot, double x, double y, bool reverse, Options move_options, Options turn_options) {
    turn_to(robot, x, y, reverse, turn_options);
    double dist = dist_to(robot, x, y, reverse);
    move_inches_ffwd(robot, dist, move_options);
}

template<typename Robot>
// Move to a coordinate point using Odometry and PID
void move_to_ffwd(Robot& robot, double x, double y, bool reverse, Options move_options, Options turn_options) {
    turn_to_ffwd(robot, x, y, reverse, turn_options);
    double dist = dist_to(robot, x, y, reverse);
    move_inches_ffwd(robot, dist, move_options);
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

// intake calibrate
template<typename Robot>
void intake_calibrate(Robot& robot){
    robot.get_intake().intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    
    robot.get_intake().intake.tare_position();
}

// kinda experimental and maybe overthought
template<typename Robot>
void intake_filter_task(Robot& robot){
    while(true){
        robot.get_intake().intake_mutex.lock();  
            if(robot.get_intake().is_red_alliance){
                    if(intake_get_blue(robot) > intake_get_red(robot)*1.8){
                        robot.get_intake().ring_detected = true;
                        pros::delay(145);
                        robot.get_intake().intake.move(-127);
                        pros::delay(300);
                    }
                    else if(intake_get_red(robot) > intake_get_blue(robot) * 2 && robot.get_intake().lift_reverse == true){
                        robot.get_intake().lift_ring_detected = true;
                        robot.get_intake().intake.move(50);
                        pros::delay(50);
                        robot.get_intake().intake.move(-90);
                        pros::delay(300);
                    }
                    else{
                        robot.get_intake().ring_detected = false;
                        robot.get_intake().lift_ring_detected = false;
                        if(robot.get_intake().auto_intake_run){
                        robot.get_intake().intake.move(127);
                        }
                        else{
                        if(!robot.get_intake().driver_intake){
                            robot.get_intake().intake.move(0);
                        }
                    }
                    }
                }
            else if(robot.get_intake().is_blue_alliance){
                if(intake_get_red(robot) > intake_get_blue(robot)*1.8){
                        robot.get_intake().ring_detected = true;
                        pros::delay(145);
                        robot.get_intake().intake.move(-127);
                        pros::delay(300);
                    }
                else if(intake_get_blue(robot) > intake_get_red(robot) * 2 && robot.get_intake().lift_reverse == true){
                        robot.get_intake().lift_ring_detected = true;
                        robot.get_intake().intake.move(50);
                        pros::delay(50);
                        robot.get_intake().intake.move(-90);
                        pros::delay(300);
                    }
                else{
                        robot.get_intake().ring_detected = false;
                        robot.get_intake().lift_ring_detected = false;
                        if(robot.get_intake().auto_intake_run){
                        robot.get_intake().intake.move(127);
                        }
                        else{
                        if(!robot.get_intake().driver_intake){
                            robot.get_intake().intake.move(0);
                        }
                }
                 }
            }
        pros::delay(10);   
        robot.get_intake().intake_mutex.unlock();  
    }
}

template<typename Robot>
double get_intake_position(Robot& robot){
    return(robot.get_intake().intake.get_position());
}

template<typename Robot>
void start_intake_update_loop(Robot& robot) {
    if(!robot.get_intake().intake_task_started) {
        robot.get_intake().intake_updater = std::make_unique<pros::Task>([&] { intake_filter_task(robot); });
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
    if(robot.get_intake.intake.get_torque() > 1){
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

template<typename Robot>
void auto_intake(Robot& robot, int volts, bool intake_run, bool lift_reverse){
    if(!dlib::get_ring_detected(robot) && !robot.get_intake().lift_ring_detected){
                robot.get_intake().auto_intake_run = intake_run;
                robot.get_intake().lift_reverse = lift_reverse;
                dlib::intake_move(robot,volts);
            }
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
void lift_move_voltage(Robot& robot, int volts){
    robot.get_lift().lift.move_voltage(volts);
}

template<typename Robot>
double get_lift_rot(Robot& robot){
    // returns in degrees
    return(std::abs(robot.get_lift().lift_rot.get_position() / 100));
}

template<typename Robot>
void reset_lift_rot(Robot& robot){
    robot.get_lift().lift_rot.reset_position();
}

/*// needs to be adjusted to be inside of a task probably!
template<typename Robot>
void lift_task(Robot& robot, double angle, rd::Console& console, Options options){
    long interval = robot.get_lift_pid().get_interval();
    double target_angle = angle;
    robot.get_lift_pid().reset();
    
    while (true) {
        robot.get_lift().lift_mutex.lock();
        if(robot.get_lift().task_toggle){

        double current_angle = robot.get_lift().lift_rot.get_position() / 100;
        double error = std::remainder(target_angle - current_angle,360);

        double output_voltage = robot.get_lift_pid().update(error);

        robot.get_lift().pid_error = error;

        lift_move_voltage(robot, output_voltage);

        pros::delay(interval);
        }
        else{
            robot.get_lift().lift.brake();
        }
        robot.get_lift().lift_mutex.unlock();
    }
}*/

template<typename Robot>
void lift_task(Robot& robot){
    while(true){
    robot.get_lift().lift_mutex.lock();
    if(robot.get_lift().task_toggle){
        if(std::abs(robot.get_lift().lift_rot.get_position() / 100) > 160){
            robot.get_lift().lift.move(90);
        }
        else{
            robot.get_lift().lift.move(30);
        }
    }
    else{
        if(std::abs(robot.get_lift().lift_rot.get_position() / 100) < 165){
            robot.get_lift().lift.move(-90);
        }
        else{
            robot.get_lift().lift.move(0);
        }
    }
    robot.get_lift().lift_mutex.unlock();
    pros::delay(10);
    }
}

template<typename Robot>
void start_lift_update_loop(Robot& robot){
    if(!robot.get_lift().lift_task_started) {
        robot.get_lift().lift_updater = std::make_unique<pros::Task>([&] {lift_task(robot);});
        robot.get_lift().lift_task_started = true;
    }
}

// ------------------------------ //
// Controls 
// ------------------------------ //

template<typename Robot>
pros::Controller get_controller(Robot& robot){
    return(robot.get_master().master);
}


}