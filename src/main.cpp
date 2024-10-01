#include "main.h"
#include "dlib/dlib.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "robodash/api.h"
#include "robodash/views/selector.hpp"
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>

// thonk award

/*
TODO:

** CREATE AUTONS FOR MATCH
* intake torque macro
* make macros based off of position for accuracy
* lift mech methods
* Create autons for skills (not super important)
* reformat

(backburner) Find a way to change background colors of the UI (THIS IS DEFINITELY POSSIBLE)
^ Try looking through robodash/lvgl stuff
*/

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
        false
    );

    dlib::Indexer indexer = dlib::Indexer(
        'G',
        true
    );

    dlib::Lift lift = dlib::Lift(
        20,
        6
    );

    dlib::Sensor sensor = dlib::Sensor(
        3,
        4
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

    dlib::Lift& get_lift(){
        return lift;
    }
    
    dlib::Sensor& get_sensor(){
        return sensor;
    }

};

// instantiate Robot object
Robot robot = Robot();

// Initialize position struct
dlib::Position position = dlib::get_position(robot, false);

// use this to actually print stuff to the console
rd::Console console;

// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

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

void initialize() {
    // Reset IMU + Tare motors
    robot.get_imu().imu.reset(true);
    dlib::tare_position(robot);

    // Start the UI focusing on auto selector for easy access
    selector.focus();

    // Set drive mode to brake
    // Better than coast for autos in my opinion
    dlib::set_mode_brake(robot);
    
    // Activate Color Sensor
    dlib::intake_activate_led(robot, 100);
    robot.get_intake().is_red_alliance = true;

    // Begin tasks
    dlib::start_intake_update_loop(robot);
    dlib::start_odom_update_loop(robot);
    
}

// Run while robot is disabled on the field.
// Usually leave this blank
void disabled() {}

void competition_initialize() {
    // focus on selector screen
    selector.focus();
}
 
void autonomous() {
    // run chosen auto
    selector.run_auton();
}
//Getting RGB values for color sensor
void opcontrol() {
    while(true){
        // get a new coordinate position
        position = dlib::get_position(robot, false);
        // get current optical sensor values
        dlib::intake_get_rgb_values(robot);

        // ------------------------------------------------- //
        // Binds
        // ------------------------------------------------- //

        // clear console at the start of each while loop
        console.clear();

        // ring sensor rgb (for testing purposes)
        console.printf("Red Value: %lf \n", dlib::intake_get_red(robot));
        console.printf("Green Value: %lf \n", dlib::intake_get_green(robot));
        console.printf("Blue Value: %lf \n", dlib::intake_get_blue(robot));

        // odometry
        console.print("X: ");
        console.println(std::to_string(position.x));
        console.print("Y: ");
        console.println(std::to_string(position.y));
        console.print("Theta: ");
        console.println(std::to_string(position.theta));

        // intake 
        console.print("Intake Torque: ");
        console.println(std::to_string(dlib::get_torque_intake(robot)));
        console.print("Intake Position: ");
        console.printf(std::to_string(dlib::get_intake_position(robot)));


        // ------------------------------------------------- //
        // Binds
        // ------------------------------------------------- //

        // arcade
        // Type of driving format the controller uses
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        dlib::arcade(robot,power,turn);
        

        // intake binds
        //Color senser if statements
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            if(!dlib::get_ring_detected(robot) && !robot.get_intake().lift_ring_detected){
                dlib::intake_move(robot,127);
            }
            
        }
            
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            if(!dlib::get_ring_detected(robot) && !robot.get_intake().lift_ring_detected){
                dlib::intake_move(robot,-127);
            }
        }
        else if(!dlib::get_ring_detected(robot) && !robot.get_intake().lift_ring_detected){
            dlib::intake_stop(robot);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            robot.get_intake().lift_reverse = !robot.get_intake().lift_reverse;
        }

        // mogo binds
        // Uses pnuematics to grab mogo
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            dlib::toggle_mogo(robot);
        }

        if(dlib::get_mogo_mode(robot) == true){
            dlib::mogo(robot, true);
        }
        if(dlib::get_mogo_mode(robot) == false){
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
            dlib::toggle_indexer(robot);
        }
        //alternates between coast and brake deceleration
        if(dlib::get_indexer_mode(robot) == true){
            dlib::set_mode_coast(robot);
        }
        if(dlib::get_indexer_mode(robot) == false){
            dlib::set_mode_brake(robot);
        }

        // arm binds
        // Empty!
        
        // delay to prevent brain damage
        pros::delay(20);
    }
}