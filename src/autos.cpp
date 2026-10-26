#include "lemlib/configs.h"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

ASSET(auto_curve_1_txt)

void lift_until_deg(int theta, int timeout, int velocity=300) {
    int time=0;
    lift_motor_group.set_brake_mode(pros::MotorBrake::hold);
    if (lift_encoder.get_position()/100 < theta) {
        while (lift_encoder.get_position()/100 < theta) {
            lift_motor_group.move_velocity(velocity);
            pros::delay(20);
            time += 20;
            if(time > timeout) {
                break;
            }

        }} else if (lift_encoder.get_position()/100 > theta){
        while (lift_encoder.get_position()/100 > theta) {
            lift_motor_group.move_velocity(-velocity);
            pros::delay(20);
            time += 20;
            if(time > timeout) {
                break;
            }
        }}
    lift_motor_group.move_velocity(0);
    
}

void claw_close() {
    flipper_motor.move_relative(225, 75);
    pros::delay(350); //changed from 700
    flipper_motor.move_relative(-225, 75);
}

void claw_open() {
    claw_motor.move_velocity(-400);
    pros::delay(700);
    claw_motor.move_velocity(0);
}

void tune_pid(){
    claw_open();
}

void match_autonomous() {
    chassis.setPose(0,0,0);
    lift_motor_group.move_velocity(-100);
    pros::c::delay(400);
    lift_encoder.set_position(0);
    lift_motor_group.move_velocity(0);

    //reset pose, turn toggle, and take the preload
    chassis.setPose(0,0,0);
    toggle_motor.move_relative(1800, 400);
    claw_close();
    pros::delay(200);

    //follow the preset curve path and lift the lift
    chassis.moveToPose(20.667, 13.667,90, 1600, {.minSpeed=65});
    lift_until_deg(3, 500);
    chassis.waitUntilDone();
    lift_until_deg(1, 700, 80);
    pros::delay(200);
    claw_open();

    auto pose = chassis.getPose();

    chassis.setPose(-2.81,0,pose.theta);

    chassis.moveToPoint(-10,0,500, {.forwards=false});
    lift_until_deg(-2, 700, 80);
    chassis.turnToPoint(0, 24+2.81, 700);
    chassis.waitUntilDone();

    chassis.moveToPoint(0, 24+2.81, 800, {.minSpeed=80});
    chassis.waitUntilDone();
    claw_close();
    

    chassis.turnToPoint(0, 0, 700);
    lift_until_deg(15, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(0,0,180,1200);
    lift_until_deg(8, 500);
    claw_open();

    pose = chassis.getPose();
    chassis.setPose(0,2.81,pose.theta);

    chassis.moveToPoint(0,10,500, {.forwards=false});
    lift_until_deg(-2, 700, 80);
    chassis.turnToPoint(24+2.81, 0, 700);
    chassis.waitUntilDone();

    chassis.moveToPoint(24+2.81, 0, 700);
    chassis.waitUntilDone();
    claw_close();

    chassis.turnToPoint(0, 0, 700);
    lift_until_deg(35, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(0,0,180,1200);
    lift_until_deg(25, 500);
    claw_open();
}