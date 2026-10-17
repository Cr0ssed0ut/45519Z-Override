#include "lemlib/configs.h"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"

// left motor group
pros::MotorGroup left_motor_group({-12,-7}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({8,19}, pros::MotorGears::blue);

pros::MotorGroup lift_motor_group({4,-9}, pros::MotorGears::red);

pros::Motor toggle_motor(18, pros::MotorGears::green);
pros::Motor claw_motor(3, pros::MotorGears::green);
pros::Motor flipper_motor(11, pros::MotorGears::green);

pros::adi::Pneumatics piston('a', false, true);
pros::Rotation lift_encoder(16);

pros::Distance left_dist(13);
pros::Distance front_dist(2);
pros::Distance right_dist(20);



// drivetrain settings
lemlib::ExpoDriveCurve throttle_curve(3,10,1.02);
lemlib::ExpoDriveCurve steer_curve(3,10,1.02);

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.5, // 10.5 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(17);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(99);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(12, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              35, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, // throttle curve
                        &steer_curve // steer curve
);
