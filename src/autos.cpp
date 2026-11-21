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
    pros::delay(400); //changed from 700
    flipper_motor.move_velocity(-225);
    pros::delay(200);
    flipper_motor.move_velocity(0);
}

void claw_open() {
    claw_motor.move_velocity(-400);
    pros::delay(600);
    claw_motor.move_velocity(0);
}


void match_autonomous() {
    lift_motor_group.set_brake_mode(pros::MotorBrake::hold);
    
    
    toggle_motor.move_relative(1800, 400);
    lift_motor_group.move_velocity(-100);
    lift_encoder.set_position(0);
    lift_motor_group.move_velocity(0);

    //reset pose, turn toggle, and take the preload
    chassis.setPose(0,0,0);
    pros::delay(600);

    //follow the preset curve path and lift the lift
    chassis.moveToPose(21.967, 14.467, 90, 1600, {.minSpeed=65});
    lift_until_deg(2, 400);
    chassis.waitUntilDone();
    lift_until_deg(1, 300);
    claw_open();

    //set pose origin to (0-offset,0,90)
    chassis.setPose(-2.81,0,chassis.getPose().theta);

    chassis.moveToPoint(-13,0,800, {.forwards=false});
    lift_until_deg(-2, 400, 80);
    claw_open();
    chassis.turnToPoint(0, 24, 800);
    chassis.waitUntilDone();
    claw_motor.move_velocity(400);
    chassis.moveToPoint(0, 24, 1200, {.maxSpeed=60});
    chassis.waitUntilDone();
    claw_close();
    claw_motor.move_velocity(0);

    
    chassis.turnToHeading(180,800);
    lift_until_deg(18, 700);
    chassis.waitUntilDone();
    chassis.setPose(0,24,180);
    chassis.moveToPoint(0,0,450, {.minSpeed=80});
    chassis.waitUntilDone();
    chassis.moveToPoint(0,0,500, {.maxSpeed=80});
    chassis.waitUntilDone();
    lift_until_deg(10, 800);
    claw_open();
    chassis.setPose(0,2.81,chassis.getPose().theta);

    chassis.moveToPoint(0,13,800, {.forwards=false});
    
    lift_until_deg(-2, 1000, 80);
    chassis.waitUntilDone();
    
    chassis.waitUntilDone();
    chassis.turnToPoint(24, 0, 900, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    claw_open();
    claw_motor.move_velocity(400);
    chassis.waitUntilDone();

    chassis.moveToPoint(24, 0, 1200, {.maxSpeed=60});
    chassis.waitUntilDone();
    claw_close();
    claw_motor.move_velocity(0);
    
    
    chassis.turnToHeading(270, 800);
    lift_until_deg(40,1000);
    chassis.waitUntilDone();
    chassis.setPose(26,0,chassis.getPose().theta);
    
    chassis.moveToPoint(0, 0, 1200, {.maxSpeed=80});
    chassis.waitUntilDone();
    
    lift_until_deg(25, 1000, 400);
    claw_open();
    

    int positions[6] = {3, 15, 18, 40, 60, 90};
    for (int i = 4; i < 6; i++) { 
        // loop match loading, starting at 4 because we already scored 3
        chassis.setPose(2.81, 0, chassis.getPose().theta);
        chassis.moveToPose(30,0,270,1200, {.forwards=false, .minSpeed=60});
        lift_until_deg(-2, 1000);
        chassis.waitUntilDone();
        chassis.setPose(30,0,270);
        chassis.turnToHeading(180, 1000);
        
        claw_open();
        claw_motor.move_velocity(400);

        chassis.setPose(0,0,180);

        chassis.moveToPoint(0,-15,1000, {.maxSpeed=60});
        chassis.waitUntilDone();
        chassis.setPose(0,-13,chassis.getPose().theta);
        claw_close();
        

        chassis.moveToPoint(0,0,1000, {.forwards=false, .maxSpeed=60});
        lift_until_deg(2,500);
        chassis.waitUntilDone();
        
        chassis.turnToHeading(267, 1200);
        claw_motor.move_velocity(0);
        lift_until_deg(positions[i], 1500);
        chassis.waitUntilDone();
        
        chassis.setPose(0,0,270);
        chassis.moveToPoint(-34,0,1000, {.maxSpeed=60});
        chassis.waitUntilDone();
        lift_until_deg(positions[i]-5, 500);
        claw_open();
    }

    chassis.setPose(2.81, 0, chassis.getPose().theta);
    
    lift_until_deg(-2, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(30,0,270,1200, {.forwards=false, .minSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    
    claw_open();
    claw_motor.move_velocity(400);

    chassis.setPose(0,0,180);

    int positions2[6] = {12, 27, 40, 55, 70, 110};
    for (int i = 0; i < 3; i++) {
        chassis.moveToPoint(0,-16,700, {.maxSpeed=60});
        chassis.waitUntilDone();
        claw_close();
        chassis.moveToPoint(0,-16,300, {.maxSpeed=60});
        
        chassis.setPose(0,-12,chassis.getPose().theta);
        
        chassis.moveToPoint(0,0,1000, {.forwards=false, .maxSpeed=60});
        pros::delay(200);
        claw_motor.move_velocity(0);
        chassis.waitUntilDone();
        chassis.turnToHeading(-24.33, 1000);
        auto lastpose = chassis.getPose();
        lift_until_deg(positions2[i], 800);
        chassis.waitUntilDone();
        chassis.moveToPoint(-12,24,1500, {.maxSpeed=60});
        
        chassis.waitUntilDone();
        lift_until_deg(positions2[i]-5, 500);
        claw_open();
        claw_motor.move_velocity(400);

        chassis.moveToPoint(lastpose.x, lastpose.y, 700, {.forwards=false, .maxSpeed=60});
        lift_until_deg(-2, 1000);
        chassis.waitUntilDone();
        chassis.turnToPoint(0,-16, 1000);
    }

    chassis.moveToPoint(0,-16,700, {.maxSpeed=60});
    chassis.waitUntilDone();
    claw_close();
    chassis.moveToPoint(0,-16,300, {.maxSpeed=60});
    
    chassis.setPose(0,-12,chassis.getPose().theta);
    
    chassis.moveToPoint(73,-99,1000, {.forwards=false, .minSpeed=90});
    pros::delay(200);
    claw_motor.move_velocity(0);
    lift_until_deg(1,200);
    chassis.waitUntilDone();

    chassis.moveToPose(-18,-160,90,1500, {.forwards=false, .minSpeed=90});
    chassis.waitUntilDone();

    lift_until_deg(-2, 500);
    toggle_motor.move_relative(-1800, 400);
    chassis.moveToPoint(140, -160, 1500);
    lift_until_deg(positions2[2], 1200);
    chassis.waitUntilDone();
    lift_until_deg(positions2[2]-10, 1200);
    claw_open();
}

 void tune_pid() {
    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, 24, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(0,0,1000, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 24, 1000, {.forwards=false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0,0,1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(24, 0, 1000);
 }

 void test_claw() {
    static int positions[5] = {3, 15, 40, 50, 80};
    chassis.setPose(2.81, 0, 270);
    chassis.moveToPoint(30,0,1200, {.forwards=false, .maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    lift_motor_group.move_velocity(-100);
    pros::delay(500);
    claw_open();
    claw_motor.move_velocity(400);

    chassis.setPose(0,0,0);

    chassis.moveToPoint(0,12,1000, {.maxSpeed=60});
    chassis.waitUntilDone();
    claw_close();
    chassis.moveToPoint(0,0,1000, {.forwards=false, .maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToHeading(145.67, 1000);
    lift_until_deg(positions[1], 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(18,-24,1000, {.maxSpeed=60});
    
    chassis.waitUntilDone();
    lift_until_deg(positions[1]-5, 500);
    claw_open();
 }

