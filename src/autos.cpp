#include "lemlib/configs.h"
#include "pros/motor_group.hpp"

void test_autonomous() {
    float lat_offset = 5+(21/25.4);
    float left = left_dist.get();
    chassis.setPose(0,0, 0); // reset robot position
    toggle_motor.move_relative(1600, 400);
    
    flipper_motor.move_relative(225,75);
    pros::delay(500);
    flipper_motor.move_relative(-225, 75);

    chassis.turnToHeading(63,1000);
    while (lift_encoder.get_position()/100 < 6) {
        lift_motor_group.move_velocity(300);
        pros::delay(20);
    }
    lift_motor_group.move_velocity(0);
    chassis.moveToPoint(22, 72, 3000, {}, false);
    lift_motor_group.move_velocity(-200);
    pros::delay(300);
    lift_motor_group.move_velocity(0);
    claw_motor.move_relative(-90, 100);
}