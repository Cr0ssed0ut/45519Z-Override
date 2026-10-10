#include "main.h"
#include "lemlib/configs.h"
#include "pros/motor_group.hpp"
#include "lemlib/autos.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    lift_encoder.reset_position(); // reset lift encoder position to 0

    toggle_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set toggle motor to hold position when not moving
    
    lift_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set lift motors to hold position when not moving
    lift_motor_group.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES); // set lift motors to use degrees for encoder units
    lift_motor_group.tare_position(); // reset lift motors position to 0
    
    claw_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES); // set claw motor to use degrees for encoder units
    claw_motor.tare_position(); // reset claw motor position to 0

    flipper_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES); // set flipper motor to use degrees for encoder units
    flipper_motor.tare_position(); // reset flipper motor position to 0

    // print position to brain screen
    pros::Task screen_task([=]() {
        while (true) {
            // print robot location to the brain screen
            auto pose = chassis.getPose();

            pros::lcd::print(0, "X: %f  Y: %f", pose.x, pose.y);
            pros::lcd::print(1, "Theta: %f", pose.theta); // heading
            pros::lcd::print(2, "Lift Pose (cDeg): %d", lift_encoder.get_position()); // lift position
            pros::lcd::print(4, "Lift Draw (mA): %d", lift_motor_group.get_current_draw()); // lift current draw

            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    test_autonomous();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER);
int time_elapsed = 0;
bool already_pressed = false;
int target_time = 0; //ms

void opcontrol() {
    claw_motor.move_velocity(400);
    // loop forever
    while (true) {

        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // control the lift motors with buttons R1 and R2
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift_motor_group.move_velocity(400);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift_motor_group.move_velocity(-75);
        } else {
            lift_motor_group.move_velocity(0);
        }

        //spin claw inwards with L2
        static bool yPressedLast = false;

        bool yPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        if (yPressed && !already_pressed) {
            target_time = 400;
            already_pressed = true;
        }
        if (yPressed && !yPressedLast) {
            claw_motor.move_relative(-80, 100);
            claw_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        } 
        
        if (time_elapsed >= target_time && !yPressed) {
            claw_motor.move_velocity(400);
            time_elapsed = 0;
            target_time = 0;
            already_pressed = false;
        }

        yPressedLast = yPressed;

        //flip flipper with Right, hold, else return to neutral
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            // put flipper up
            flipper_motor.move_absolute(225,75);
            flipper_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            pros::delay(20);    
        } else {
            // return flipper to neutral position
            flipper_motor.move_absolute(0,55);
            flipper_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }

        // delay to save resources
        pros::delay(50);
        time_elapsed += 50;
    }
}

