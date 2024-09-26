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
Find a way to change background colors of the UI (THIS IS DEFINITELY POSSIBLE)
^ Try looking through robodash/lvgl stuff

See if you can make a second tab for selector (one for red autos, one for blue autos)

Look through dLib, make sure it all makes sense roughly
*/

// example UI stuff, adjust for the autos that we're actually gonna use

bool toggle_mogo = false;
bool toggle_mode = false;

bool is_red_alliance;
bool is_blue_alliance;

void red_awp(){
    is_red_alliance = true;
}
void red_6_ring(){
    is_red_alliance = true;
}
void blue_awp(){
    is_blue_alliance = true;
}
void blue_6_ring(){
    is_blue_alliance = true;
}
void skills(){}

// robo dash works modularly, so you can add more autos into this constructor
rd::Selector Red_Selector({
    {"Red AWP", &red_awp},
    {"Red 6 Ring", &red_6_ring},
});

rd::Selector Blue_Selector({
    {"Blue AWP", &blue_awp},
    {"Blue 6 Ring", &blue_6_ring},
});

rd::Selector Skills_SXselector({
    {"Skills", &skills},
});

// use this to actually print stuff to the console
rd::Console console;

// initialize

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Optical optical(14);

pros::c::optical_rgb_s_t rgb_value;

// dLib
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
};

// instantiate Robot object
Robot robot = Robot();

void initialize() {}

void disabled() {}

void competition_initialize() {
    // focus on selector screen
    Red_Selector.focus();
}
 
void autonomous() {
    Red_Selector.run_auton();
}

void opcontrol() {
    dlib::set_mode_brake(robot);
    optical.set_led_pwm(100);
    is_red_alliance = true;
    while(true){
        rgb_value = optical.get_rgb();
        console.clear();
        console.printf("Red Value: %lf \n", rgb_value.red);
        console.printf("Green Value: %lf \n", rgb_value.green);
        console.printf("Blue Value: %lf \n", rgb_value.blue);
  
        // arcade
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        dlib::arcade(robot,power,turn);
        

        // intake binds
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            if(is_red_alliance){
                if(rgb_value.blue > rgb_value.red*1.3){
                    console.printf("Blue Ring Detected, Flinging");
                    dlib::intake_stop(robot);
                }
                else if(rgb_value.red > rgb_value.blue*1.5){
                    console.printf("Red Ring Detected");
                }
                else{
                    dlib::intake_move(robot,127);
                }
            }
            else if(is_blue_alliance){
                if(rgb_value.red > rgb_value.blue*1.5){
                    console.printf("Red Ring Detected, Flinging");
                    dlib::intake_stop(robot);
                }
                else if(rgb_value.blue > rgb_value.red*1.3){
                    console.printf("Blue Ring Detected");
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
        
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            toggle_mogo = !toggle_mogo;
        }

        if(toggle_mogo == true){
            dlib::mogo(robot, true);
        }
        if(toggle_mogo == false){
            dlib::mogo(robot,false);
        }
        
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            dlib::indexer(robot,true);
        }
        else{
            dlib::indexer(robot,false);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            toggle_mode = !toggle_mode;
        }

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