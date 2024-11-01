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
    	450,
        1.96,
        8.8
    );

    dlib::IMU imu = dlib::IMU(
        15,
        // IMU return scalar
        1.01322825781
    );

    dlib::PID drive_pid = dlib::PID(
        {15, 0, 0.75}, // 15
        5
    );

    dlib::PID turn_pid = dlib::PID(
        {8,0,.69},
        10
    );

    dlib::PID lift_pid = dlib::PID(
        {8,0,0},
        10
    );

    dlib::FeedForward drive_feed_forward = dlib::FeedForward(
        {0,0.8449287665,5.70181872}
    );

    dlib::FeedForward turn_feed_forward = dlib::FeedForward(
        {0,0,0}
    );

    dlib::Odom odom = dlib::Odom();

    dlib::Intake intake = dlib::Intake(
        -8
    );

    dlib::Mogo mogo = dlib::Mogo(
        'H',
        true
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

    dlib::FeedForward& get_drive_feed_forward() {
        return drive_feed_forward;
    }
    
    dlib::FeedForward& get_turn_feed_forward() {
        return turn_feed_forward;
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

pros::adi::DigitalOut doinker('E', false);

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
    double start_time = pros::millis();
    robot.get_chassis().left.tare_position_all();
    robot.get_chassis().right.tare_position_all();
    robot.get_intake().is_red_alliance = false;
    robot.get_intake().is_blue_alliance = true;
    dlib::move_to(robot,-15,0,true,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 200,
    .max_voltage = 12000
    });


    // go to mogo
    dlib::turn_degrees(robot,90,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 10000
    });

    dlib::move_to_ffwd(robot, -14,-5, true, {
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 500,
    .max_voltage = 12000
    },{
    .error_threshold = 0,
    .settle_ms = 0,
    .max_ms = 0,
    .max_voltage = 0
    });

    // alliance stake
    dlib::auto_intake(robot, 127, true, false);
    pros::delay(600);
    dlib::auto_intake(robot, 0, false, false);
    // y = 7
    dlib::move_to_ffwd(robot,-17.75,7,false,{
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

    // clip mogo
    dlib::move_to_ffwd(robot,9,30,true,{
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
    // intake safe ring
    dlib::move_to_ffwd(robot,27,31.5,false,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    });
    
    //31,41,false
    dlib::move_to_ffwd(robot,16,31.5,true,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 400,
    .max_ms = 0,
    .max_voltage = 12000});

    
    dlib::move_to_ffwd(robot,23,45,false,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 750,
    .max_voltage = 12000});
    
    dlib::move_to_ffwd(robot,34,45,false,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 750,
    .max_voltage = 12000});

    // move to ladder
    dlib::move_to_ffwd(robot,26,34,true,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 750,
    .max_voltage = 12000});

    robot.get_lift().task_toggle = true;

    dlib::move_to_ffwd(robot,10,41.5,false,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000});
 
    robot.get_lift().task_toggle = false;

    
    double end_time = pros::millis();
    double elapsed = end_time - start_time;
    std::cout << elapsed << std::endl;    
}

void red_6_ring(){
    console.focus();
    robot.get_intake().is_red_alliance = true;
}
void blue_awp(){
    double start_time = pros::millis();
    robot.get_chassis().left.tare_position_all();
    robot.get_chassis().right.tare_position_all();
    robot.get_intake().is_red_alliance = false;
    robot.get_intake().is_blue_alliance = true;
    dlib::move_to(robot,-15,0,true,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 200,
    .max_voltage = 12000
    });


    // go to mogo
    dlib::turn_degrees(robot,-90,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 10000
    });

    dlib::move_to_ffwd(robot, -14,5, true, {
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 500,
    .max_voltage = 12000
    },{
    .error_threshold = 0,
    .settle_ms = 0,
    .max_ms = 0,
    .max_voltage = 0
    });

    // alliance stake
    dlib::auto_intake(robot, 127, true, false);
    pros::delay(600);
    dlib::auto_intake(robot, 0, false, false);
    // y = 7
    dlib::move_to_ffwd(robot,-17.75,-7,false,{
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

    // clip mogo
    dlib::move_to_ffwd(robot,9,-30,true,{
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
    // intake safe ring
    dlib::move_to_ffwd(robot,27,-31.5,false,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    });
    
    //31,41,false
    dlib::move_to_ffwd(robot,16,-31.5,true,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 400,
    .max_ms = 0,
    .max_voltage = 12000});

    
    dlib::move_to_ffwd(robot,23,-45,false,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 750,
    .max_voltage = 12000});
    
    dlib::move_to_ffwd(robot,34,-45,false,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 750,
    .max_voltage = 12000});

    // move to ladder
    dlib::move_to_ffwd(robot,26,-34,true,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 750,
    .max_voltage = 12000});

    robot.get_lift().task_toggle = true;

    dlib::move_to_ffwd(robot,10,-41.5,false,{
    .error_threshold = 0.3,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000
    },{
    .error_threshold = 1,
    .settle_ms = 200,
    .max_ms = 1000,
    .max_voltage = 12000});
 
    robot.get_lift().task_toggle = false;

    
    double end_time = pros::millis();
    double elapsed = end_time - start_time;
    std::cout << elapsed << std::endl;    
    //26,34
    // lift
    //15,41.5
    // lift stop
}
void blue_6_ring(){
    robot.get_intake().is_blue_alliance = true;
}
void skills(){
    dlib::auto_intake(robot, 127, true, false); 
    pros::delay(750);
    dlib::auto_intake(robot, 0, false, false); 

    dlib::move_to(robot,15,0,false,{
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
    dlib::move_to(robot,16,-27,true,{
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

    dlib::move_to(robot,4.3,-57.1,true,{
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
    selector.run_auton();

}

//Getting RGB values for color sensor
void opcontrol() {
    // Set drive mode to brake
    // Better than coast for autos in my opinion
    dlib::auto_intake(robot, 0, false, false);
    dlib::set_mode_brake(robot);
    robot.get_intake().driver_intake = true;
    //robot.get_lift().lift_rot.reset_position();
    //robot.get_lift().lift_rot.set_reversed(true);
    
    while(true){
        // get a new coordinate position
        position = dlib::get_position(robot, false);
        // get current optical sensor values
        dlib::intake_get_rgb_values(robot);
;
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
        auto celsius = robot.get_intake().intake.get_temperature();
        console.print("Intake Temperature: ");
        console.println(std::to_string((celsius * (9/5)) + 32));
        console.print("Celsius: ");
        console.println(std::to_string(celsius));

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
            robot.get_lift().task_toggle = !robot.get_lift().task_toggle;
        }
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            doinker.set_value(true);
        }
        else{
            doinker.set_value(false);
        }
        
        // delay to prevent brain damage
        pros::delay(20);
    }
}