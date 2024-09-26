#include "main.h"
#include "dlib/dlib.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "robodash/api.h"
#include "robodash/views/selector.hpp"
#include <initializer_list>
#include <memory>
#include <utility>

/*
TODO:

**INTAKE FILTRATION MECH TASK
**ODOM TASK VERIFICATION

Find a way to change background colors of the UI (THIS IS DEFINITELY POSSIBLE)
^ Try looking through robodash/lvgl stuff
*/

// example UI stuff, adjust for the autos that we're actually gonna use

bool toggle_mogo = false;
bool toggle_mode = false;

// use this to actually print stuff to the console
rd::Console console;

// initialize

pros::Controller master(pros::E_CONTROLLER_MASTER);

// dLib
//Initializing values using dLib
struct Robot {
    dlib::Chassis chassis = dlib::Chassis(
	    {18,19,17},
        {-14,-16,-11},
    	3.25,
    	1
    );

    dlib::IMU imu = dlib::IMU(
        15
    );

    dlib::PID drive_pid = dlib::PID(
        {},
        1
    );

    dlib::PID turn_pid = dlib::PID(
        {},
        1
    );

    dlib::Odom odom = dlib::Odom(
        1,
        1
    );

    dlib::Intake intake = dlib::Intake(
        8
    );

    dlib::Mogo mogo = dlib::Mogo(
        'H',
        toggle_mogo
    );

    dlib::Indexer indexer = dlib::Indexer(
        'G',
        true
    );

    dlib::Sensor sensor = dlib::Sensor(
        3
    );

    dlib::Chassis& get_chassis() {
        return chassis;
    }
    
    dlib::IMU& get_imu(){
        return imu;
    }

    dlib::PID& get_drive_pid() {
        return drive_pid;
    }

    dlib::PID& get_turn_pid() {
        return turn_pid;
    }

    dlib::Odom& get_odom() {
        return odom;
    }

    dlib::Intake& get_intake() {
        return intake;
    }

    dlib::Mogo& get_mogo() {
        return mogo;
    }

    dlib::Indexer& get_indexer(){
        return indexer;
    }
    
    dlib::Sensor& get_sensor(){
        return sensor;
    }
};

// instantiate Robot object
Robot robot = Robot();

//declaring autons for both sides
void red_awp(){
    robot.get_intake().is_red_alliance = true;
}
void red_6_ring(){
    robot.get_intake().is_red_alliance = true;
}
void blue_awp(){
    robot.get_intake().is_blue_alliance = true;
}
void blue_6_ring(){
    robot.get_intake().is_blue_alliance = true;
}
void skills(){}

// robo dash works modularly, so you can add more autos into this constructor
// Creating ui using robo dash
rd::Selector selector({
    {"Red AWP", &red_awp},
    {"Red 6 Ring", &red_6_ring},
    {"Blue AWP", &blue_awp},
    {"Blue 6 Ring", &blue_6_ring},
    {"Skills", &skills},
});

void initialize() {}

void disabled() {}

void competition_initialize() {
    // focus on selector screen
    selector.focus();
}
 
void autonomous() {
    selector.run_auton();
}
//Getting RGB values for color sensor
void opcontrol() {
    selector.focus();
    dlib::set_mode_brake(robot);
    robot.get_intake().is_red_alliance = true;
    dlib::activate_led(robot, 100);
    while(true){
        dlib::get_rgb_values(robot);
        console.clear();
        console.printf("Red Value: %lf \n", dlib::intake_get_red(robot));
        console.printf("Green Value: %lf \n", dlib::intake_get_green(robot));
        console.printf("Blue Value: %lf \n", dlib::intake_get_blue(robot));
        console.printf("Motor Torque: %d \n", dlib::get_torque_intake(robot));
        if(dlib::intake_get_blue(robot) > dlib::intake_get_red(robot)*1.3){
                    console.printf("Blue Ring Detected, Flinging");
        }
        // arcade
        // Type of driving format the controller uses
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        dlib::arcade(robot,power,turn);
        

        // intake binds
        //Color senser if statements
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            if(robot.get_intake().is_red_alliance){
                if(dlib::intake_get_blue(robot) > dlib::intake_get_red(robot)*1.3){
                    console.printf("Blue Ring Detected, Flinging");
                    dlib::intake_move(robot,-127);
                    pros::delay(200);
                    dlib::intake_move(robot,127);
                }
                else if(dlib::intake_get_red(robot) > dlib::intake_get_blue(robot)*1.5){
                    console.printf("Red Ring Detected");
                    dlib::intake_move(robot,127);
                }
                else{
                    dlib::intake_move(robot,127);
                }
            }
            else if(robot.get_intake().is_blue_alliance){
                if(dlib::intake_get_red(robot) > dlib::intake_get_blue(robot)*1.5){
                    console.printf("Red Ring Detected, Flinging");
                    dlib::intake_stop(robot);
                }
                else if(dlib::intake_get_blue(robot) > dlib::intake_get_red(robot)*1.3){
                    console.printf("Blue Ring Detected");
                    dlib::intake_move(robot,127);
                }
                else{
                    dlib::intake_move(robot,127);
                }
            }
        }
            
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            dlib::intake_move(robot,-127);
        }
        else{
            dlib::intake_stop(robot);
        }

        // mogo binds
        // Uses pnuematics to grab mogo
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            toggle_mogo = !toggle_mogo;
        }

        if(toggle_mogo == true){
            dlib::mogo(robot, true);
        }
        if(toggle_mogo == false){
            dlib::mogo(robot,false);
        }
        //Uses pnuematics to change height of intake
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            dlib::indexer(robot,true);
        }
        else{
            dlib::indexer(robot,false);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            toggle_mode = !toggle_mode;
        }
        //alternates between coast and brake deceleration
        if(toggle_mode == true){
            dlib::set_mode_coast(robot);
        }
        if(toggle_mode == false){
            dlib::set_mode_brake(robot);
        }
        
        
        

        // arm binds
        // Empty!
        pros::delay(20);
    }
}