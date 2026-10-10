#include "main.h"
#include "lemlib/configs.h"
#include "pros/motor_group.hpp"

void test_autonomous() {
    chassis.setPose(0, 0, 0); // reset robot position to (0, 0) with heading 0 degrees
    chassis.turnToHeading(-180,1000000);
}