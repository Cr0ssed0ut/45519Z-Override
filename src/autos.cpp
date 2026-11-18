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
    flipper_motor.move_velocity(225);
    pros::delay(350); //changed from 700
    flipper_motor.move_velocity(-225);
}

void claw_open() {
    claw_motor.move_velocity(-400);
    pros::delay(1000);
}

void match_autonomous() {
    lift_motor_group.move_velocity(-100);
    pros::c::delay(400);
    lift_encoder.set_position(0);
    lift_motor_group.move_velocity(0);

    //reset pose, turn toggle, and take the preload
    chassis.setPose(0,0,0);
    toggle_motor.move_relative(1800, 400);
    pros::delay(500);

    //follow the preset curve path and lift the lift
    chassis.moveToPoint(0, 13.667, 1000, {.minSpeed=75});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 700);
    lift_until_deg(4, 500);
    chassis.waitUntilDone();
    chassis.moveToPose(21.667, 13.667, 90, 1000, {.minSpeed=75});
    chassis.waitUntilDone();
    lift_until_deg(3, 300);
    claw_open();

    //set pose origin to (0-offset,0,90)
    chassis.setPose(-2.81,0,chassis.getPose().theta);

    chassis.moveToPoint(-10,0,700, {.forwards=false});
    lift_until_deg(-2, 400, 80);
    claw_open();
    chassis.turnToPoint(0, 24, 1000);
    chassis.waitUntilDone();
    claw_motor.move_velocity(400);

    chassis.moveToPoint(0, 24, 1500, {.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.setPose(0,24,chassis.getPose().theta);
    claw_close();
    claw_motor.move_velocity(0);

    lift_until_deg(15, 700);
    chassis.turnToPoint(0,0,800);
    chassis.waitUntilDone();
    chassis.moveToPoint(0,0,1600, {.maxSpeed=60});
    chassis.waitUntilDone();
    lift_until_deg(10, 800);
    chassis.setPose(0,2.81,chassis.getPose().theta);
    claw_open();

    chassis.moveToPoint(0,10,800, {.forwards=false});
    lift_until_deg(-2, 1000, 80);
    claw_motor.move_velocity(400);
    chassis.waitUntilDone();
    chassis.turnToPoint(24, 0, 1500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.waitUntilDone();

    chassis.moveToPoint(24, 0, 1000, {.minSpeed=60});
    chassis.waitUntilDone();
    chassis.setPose(24,0,chassis.getPose().theta);
    claw_close();
    claw_motor.move_velocity(0);

    chassis.turnToPoint(0, 0, 1000);
    lift_until_deg(35, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 0, 1000, {.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.setPose(2.81,0,chassis.getPose().theta);
    lift_until_deg(25, 1000);
    claw_open();

    chassis.moveToPoint(35,0,1200, {.forwards=false, .maxSpeed=70});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 700);
}

 void tune_pid() {
    chassis.setPose(0,0,0);
    
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 2000);
    chassis.waitUntilDone();
 }