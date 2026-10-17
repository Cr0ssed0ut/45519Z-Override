#include "lemlib/configs.h"
#include "pros/motor_group.hpp"

ASSET(auto_curve_1_txt)

void lift_until_deg(int theta) {
    while (lift_encoder.get_position()/100 < theta) {
        lift_motor_group.move_velocity(300);
        pros::delay(20);
    }
    while (lift_encoder.get_position()/100 > theta) {
        lift_motor_group.move_velocity(-300);
        pros::delay(20);
    }
    lift_motor_group.move_velocity(0);
}

void claw_close() {
    flipper_motor.move_relative(225, 75);
    pros::delay(500);
    flipper_motor.move_relative(-225, 75);
}

void claw_open() {
    claw_motor.move_relative(90, 100);
    pros::delay(300);
}

void test_autonomous() {
    //reset pose, turn toggle, and take the preload
    chassis.setPose(0,0,0);
    toggle_motor.move_relative(1600, 400);
    claw_close();

    //follow the preset curve path and lift the lift
    chassis.follow(auto_curve_1_txt, 5, 3000);
    lift_until_deg(6);
    chassis.waitUntilDone();

    //reset position after following the path, lower the lift, and release the preload
    chassis.setPose(11.52, 16,90); 
    lift_until_deg(2);
    claw_motor.move_relative(90, 100);

    //move backwards 4in
    chassis.moveToPoint(7.52, 16, 
                        2000, 
                        {.forwards=false}); 
    lift_until_deg(0);
    claw_motor.move_velocity(400);

    //move to target pose. motion chain so that it is fast right until it approaches the stack.
    chassis.turnToPoint(14.8, 35.3, 1000);
    chassis.moveToPoint(13.459,31.197,
                        5000, 
                        {.minSpeed=40});
    chassis.moveToPose(14.8, 35.3, 
                       30, 
                       1000, 
                       {.maxSpeed=40});
    chassis.waitUntilDone();
    claw_close();

    //set up pose accurately, then lift the lift and go to the goal
    chassis.moveToPose(17.5, 35.3, 90, 700);
    chassis.turnToHeading(180, 1000);
    lift_until_deg(15);
    chassis.moveToPose(17.5, 22, 180,2000);
    chassis.waitUntilDone();
    
    //reset pose again and lower the lift to score
    chassis.setPose(17.5,22,180);
    lift_until_deg(13);
    claw_open();
    claw_motor.move_velocity(400);

    //move backwards 4in and reset lift pose
    chassis.moveToPoint(17.5, 32, 
                        1000, 
                        {.forwards=false});
    lift_until_deg(0);

    //move to target pose. motion chain so that it is fast right until it approaches the stack.
    chassis.turnToPoint(36.1, 18.5, 1000);
    chassis.moveToPoint(32.61,19.9,
                        5000, 
                        {.minSpeed=40});
    chassis.moveToPose(36.1, 18.5, 
                       115, 
                       1000, 
                       {.maxSpeed=40});
    chassis.waitUntilDone();
    claw_close();

    //set up pose accurately, then lift the lift and go to the goal
    chassis.moveToPose(36.1, 16, 90, 700);
    chassis.turnToHeading(270, 1000);
    lift_until_deg(20);
    chassis.moveToPose(25,16,270,2000);

    //lower the lift to score
    lift_until_deg(18);
    claw_open();
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, -4, 1000);
    claw_motor.move_velocity(400);
}