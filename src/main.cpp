#include "main.h"
#include "dlib/dlib.hpp"
#include "dlib/pid.hpp"
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

*/

// dLib

struct Robot {
    dlib::Chassis chassis = dlib::Chassis(
	    {18,19,17},
        {-14,-16,-11},
    	3.25,
    	450
    );

    dlib::IMU imu = dlib::IMU(
        15
    );

    dlib::PID drive_pid = dlib::PID(
        {15, 0, 1},
        10
    );

    dlib::PID turn_pid = dlib::PID(
        {5.75,0,.39},
        10
    );

    dlib::PID lift_pid = dlib::PID(
        {1,0,0},
        10
    );

    dlib::FeedForward feed_forward = dlib::FeedForward(
        {0,900,0}
    );

    dlib::Odom odom = dlib::Odom();

    dlib::Intake intake = dlib::Intake(
        -8
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
        -1,
        10
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

    dlib::PID& get_lift_pid() {
        return lift_pid;
    }

    dlib::FeedForward& get_feedforward() {
        return feed_forward;
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
    // Robot& robot, double x, double y, rd::Console& console, double static_offset, bool reverse, const Options options
  robot.get_chassis().left.tare_position_all();
    robot.get_chassis().right.tare_position_all();
    robot.get_intake().is_red_alliance = true;
    robot.get_intake().is_blue_alliance = false;
        // Robot& robot, double x, double y, rd::Console& console, double static_offset, bool reverse, const Options options
    dlib::move_to(robot,console,-15.75,0,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 8000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 200,
        .max_voltage = 12000
        });


        // go to mogo
        dlib::move_to(robot,console,-15.75,-5,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 500,
        .max_voltage = 8000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 500,
        .max_voltage = 12000
        });

        dlib::auto_intake(robot, 127, true, false);
        pros::delay(500);
        dlib::auto_intake(robot, 0, false, false);
        // y = 7
        dlib::move_to(robot,console,-15.75,3,false,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 400,
        .max_voltage = 12000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 0,
        .max_voltage = 12000
        });

        dlib::move_to(robot,console,9,30,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1500,
        .max_voltage = 5500
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 12000
        });
        dlib::mogo(robot, true);
        pros::delay(250);

        dlib::auto_intake(robot, 127, true, false);

        dlib::move_to(robot,console,27,30,false,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 10000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 12000
        });
    
        //31,41,false
        /*dlib::move_to(robot,console,31,41,false,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 10000
        },{
        .error_threshold = 1,
        .settle_ms = 400,
        .max_ms = 1000,
        .max_voltage = 12000});

        //28,34.6,true
        dlib::move_to(robot,console,28,34.6,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 10000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 750,
        .max_voltage = 12000});
        //37,46.5,false
        dlib::move_to(robot,console,37,46.5,false,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 10000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 750,
        .max_voltage = 12000});
        //31,35,true
        dlib::move_to(robot,console,31,35,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 10000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 750,
        .max_voltage = 12000});
        //1,31,false*/

    //29.5,35.9
    // 34.8 43.4
    // 0.5, 45
    //-48,11,true
    pros::delay(5000);
    dlib::move_to(robot,console,-48,11,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 5000,
        .max_voltage = 12000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 12000});
    dlib::auto_intake(robot, 0, false,false);
}

void red_6_ring(){
    console.focus();
    robot.get_intake().is_red_alliance = true;
}
void blue_awp(){
    robot.get_chassis().left.tare_position_all();
    robot.get_chassis().right.tare_position_all();
    robot.get_intake().is_red_alliance = false;
    robot.get_intake().is_blue_alliance = true;
        // Robot& robot, double x, double y, rd::Console& console, double static_offset, bool reverse, const Options options
        
       dlib::move_to(robot,console,-15.75,0,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 8000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 200,
        .max_voltage = 12000
        });


        // go to mogo
        dlib::move_to(robot,console,-15.75, 5,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 500,
        .max_voltage = 8000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 500,
        .max_voltage = 12000
        });

        dlib::auto_intake(robot, 127, true, false);
        pros::delay(500);
        dlib::auto_intake(robot, 0, false, false);
        // y = 7
        dlib::move_to(robot,console,-15.75,-3,false,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 400,
        .max_voltage = 12000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 0,
        .max_voltage = 12000
        });

        dlib::move_to(robot,console,12,-30,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1500,
        .max_voltage = 5500
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 12000
        });
        dlib::mogo(robot, true);
        pros::delay(250);

        dlib::auto_intake(robot, 127, true, false);

        dlib::move_to(robot,console,27,-30,false,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 10000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 12000
        });
    
        
        dlib::move_to(robot,console,-48,-11,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 5000,
        .max_voltage = 12000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 750,
        .max_voltage = 12000});
    dlib::auto_intake(robot, 0, false,false);


     
}
void blue_6_ring(){
    robot.get_intake().is_blue_alliance = true;
}
void skills(){
    dlib::auto_intake(robot, 127, true, false); 
    pros::delay(750);
    dlib::auto_intake(robot, 0, false, false); 

    dlib::move_to(robot,console,15,0,false,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 12000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 200,
        .max_voltage = 12000});

        // move to mogo
    dlib::move_to(robot,console,16,-27,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 3000,
        .max_voltage = 6000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 9000});
    dlib::mogo(robot,true);

    dlib::move_to(robot,console,4.3,-57.1,true,{
        .error_threshold = 0.3,
        .settle_ms = 200,
        .max_ms = 3000,
        .max_voltage = 6000
        },{
        .error_threshold = 1,
        .settle_ms = 200,
        .max_ms = 1000,
        .max_voltage = 9000});
    dlib::mogo(robot,false);
    
}
//15,0,false
//16,-27,true,clamp
//4.3,-57.1,true,unclamp
//
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
    // Calibrate chassis + intake
    dlib::calibrate(robot);
    dlib::intake_calibrate(robot);

    robot.get_chassis().left.tare_position_all();
    robot.get_chassis().right.tare_position_all();

    // Start the UI focusing on auto selector for easy access
    selector.focus();
    
    // Activate Color Sensor
    dlib::intake_activate_led(robot, 20);
    robot.get_intake().is_red_alliance = true;

    // Begin tasks
    dlib::start_intake_update_loop(robot);
    dlib::start_odom_update_loop(robot);

    // try this instead?:
    //dlib::start_odom_update_loop_alt(robot);

    dlib::start_lift_update_loop(robot);
}

// Run while robot is disabled on the field.
// Usually leave this blank
void disabled() {}

void competition_initialize() {
    // focus on selector screen
    robot.get_chassis().left.tare_position_all();
    robot.get_chassis().right.tare_position_all();
    selector.focus();
}
 
void autonomous() {
    ///blue_awp();
    selector.run_auton();
}

//Getting RGB values for color sensor
void opcontrol() {
    // Set drive mode to brake
    // Better than coast for autos in my opinion
    dlib::auto_intake(robot, 0, false, false);
    dlib::set_mode_brake(robot);
    robot.get_intake().driver_intake = true;
    dlib::mogo(robot,true);
    //robot.get_lift().lift_rot.reset_position();
    //robot.get_lift().lift_rot.set_reversed(true);
    
    while(true){
        // get a new coordinate position
        position = dlib::get_position(robot, false);
        // get current optical sensor values
        dlib::intake_get_rgb_values(robot);

        // ------------------------------------------------- //
        // Logs
        // ------------------------------------------------- //
        
        // clear console at the start of each while loop
        console.clear();

        // ring sensor rgb (for testing purposes)
        // returns rgba values (not 0-255)
        console.printf("Red Value: %lf \n", dlib::intake_get_red(robot));
        console.printf("Green Value: %lf \n", dlib::intake_get_green(robot));
        console.printf("Blue Value: %lf \n", dlib::intake_get_blue(robot));

        // odometry
        // position: inches
        // theta: degrees
        console.print("X: ");
        console.println(std::to_string(position.x));
        console.print("Y: ");
        console.println(std::to_string(position.y));
        console.print("Theta: ");
        console.println(std::to_string(position.theta));

        // intake 
        // torque (in newton-meters)
        // position (rotations)
        console.print("Intake Torque: ");
        console.println(std::to_string(dlib::get_torque_intake(robot)));
        console.print("Intake Position: ");
        console.println(std::to_string(dlib::get_intake_position(robot)));

        robot.get_lift().lift_mutex.lock();
        console.print("Rot Theta: ");
        console.println(std::to_string(dlib::get_lift_rot(robot)));
        console.print("pid error: ");
        console.println(std::to_string(robot.get_lift().pid_error));
        robot.get_lift().lift_mutex.unlock();

    
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
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            if(!dlib::get_ring_detected(robot) && !robot.get_intake().lift_ring_detected){
                dlib::intake_move(robot,127);
            }
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            if(!dlib::get_ring_detected(robot) && !robot.get_intake().lift_ring_detected){
                dlib::intake_move(robot,-127);
            }
        }
        else if(!dlib::get_ring_detected(robot) && !robot.get_intake().lift_ring_detected){
            dlib::intake_stop(robot);}
        

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            robot.get_intake().lift_reverse = !robot.get_intake().lift_reverse;
        }

        // mogo binds
        // Uses pnuematics to grab mogo
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            dlib::toggle_mogo(robot);
        }

        if(dlib::get_mogo_mode(robot) == true){
            dlib::mogo(robot, true);
        }
        if(dlib::get_mogo_mode(robot) == false){
            dlib::mogo(robot,false);
        }

        //Uses pnuematics to change height of intake
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            dlib::indexer(robot,true);
        }
        else{
            dlib::indexer(robot,false);
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
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            robot.get_lift().task_toggle = true;
        }
        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            robot.get_lift().task_toggle = false;
        }
        
        // delay to prevent brain damage
        pros::delay(20);
    }
}