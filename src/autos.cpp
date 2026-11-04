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
    pros::delay(500);
    claw_motor.move_velocity(0);
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
    pros::delay(500);

    //follow the preset curve path and lift the lift
    chassis.moveToPose(20.667, 13.667,90, 1600, {.minSpeed=65});
    lift_until_deg(3, 500);
    chassis.waitUntilDone();
    lift_until_deg(2, 300);
    claw_open();

    //set pose origin to (0-offset,0,90)
    chassis.setPose(-2.81,0,90);

    chassis.moveToPoint(-10,0,500, {.forwards=false});
    lift_until_deg(-2, 400, 80);
    claw_open();
    claw_motor.move_velocity(400);
    chassis.turnToPoint(0, 23, 1000);
    chassis.waitUntilDone();

    chassis.moveToPoint(0,23, 1000, {.maxSpeed=60});
    chassis.waitUntilDone();
    claw_close();
    claw_motor.move_velocity(0);

    lift_until_deg(13, 700);
    chassis.turnToHeading(45, 700);
    chassis.waitUntilDone();
    chassis.setPose(0,24,0);
    chassis.moveToPose(0,2.81,180,1600, {.minSpeed=60});
    chassis.waitUntilDone();
    lift_until_deg(8, 800);
    chassis.setPose(0,2.81,180);
    claw_open();

    

    chassis.moveToPoint(0,10,800, {.forwards=false});
    lift_until_deg(-2, 1000, 80);
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.waitUntilDone();
    chassis.setPose(0,12.67,135);

    chassis.moveToPoint(18, 0, 1000, {.minSpeed=40});
    chassis.moveToPoint(18, 0, 400, {.maxSpeed=40});
    chassis.waitUntilDone();
    claw_close();

    chassis.setPose(24,0,0);
    lift_until_deg(15, 1000);
    chassis.moveToPoint(0, 0, 800);
    chassis.waitUntilDone();
    lift_until_deg(8, 1000);
    claw_open();
 }

 void tune_pid() {
    chassis.setPose(0,0,0);
    lift_motor_group.move_velocity(-100);
    pros::c::delay(400);
    lift_encoder.set_position(0);
    lift_motor_group.move_velocity(0);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 24, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 2000);
    chassis.waitUntilDone();
    chassis.setPose(0,24,180);
    chassis.moveToPoint(0, 0, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 2000);
    chassis.waitUntilDone();
 }