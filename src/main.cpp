#include "main.h"
#include "dlib/dlib.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "robodash/api.h"
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
void red_awp(){}
void red_6_ring(){}
void blue_awp(){}
void blue_6_ring(){}
void skills(){}

// robo dash works modularly, so you can add more autos into this constructor
rd::Selector selector({
    {"Red AWP", &red_awp},
    {"Red 6 Ring", &red_6_ring},
    {"Blue AWP", &blue_awp},
    {"Blue 6 Ring", &blue_6_ring},
    {"Skills", &skills},
});

// use this to actually print stuff to the console
rd::Console console;

// initialize

pros::Controller master(pros::E_CONTROLLER_MASTER);

// dLib
struct Robot {
    dlib::Chassis chassis = dlib::Chassis(
	    {1,2,3},
        {4,5,6},
    	3.25,
    	1
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
        7
    );

    dlib::Chassis& get_chassis() {
        return chassis;
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
};

void initialize() {
	
}

void disabled() {}

void competition_initialize() {
    selector.focus();
}

void autonomous() {
    selector.run_auton();
    // make new robot
    Robot robot = Robot();

    dlib::intake_fwd(robot, 127);
}

void opcontrol() {
    for (int i = 0; i < 100; i++) {
		console.printf("Hello %d\n", i);
		pros::delay(200);
	}
    while(true){
        pros::delay(20);   
    }
}