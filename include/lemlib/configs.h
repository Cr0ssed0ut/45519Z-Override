#ifndef _PROS_CONFIGS_H_
#define _PROS_CONFIGS_H_

#include "api.h"
#include "lemlib/api.hpp"
#include "main.h"
#include "pros/motors.hpp"

extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::MotorGroup lift_motor_group;
extern lemlib::Chassis chassis;

extern pros::Motor claw_motor;
extern pros::Motor toggle_motor;
extern pros::Motor flipper_motor;
extern pros::adi::Pneumatics piston;
extern pros::Rotation lift_encoder;

extern pros::Distance left_dist;
extern pros::Distance front_distance;
extern pros::Distance right_distance;
extern pros::Distance back_distance;


#endif 