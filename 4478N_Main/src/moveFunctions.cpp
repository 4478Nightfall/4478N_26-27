#include "main.h"
#include "api.h"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "moveFunctions.h"
#include <string>
#include <iostream>
#include <thread>
#include <algorithm>
#include <atomic>
#include <memory>
#include <cmath>
#include "pros/rtos.hpp"

using namespace pros;
using namespace lemlib;

// ---------- drive / intake / pid utils ----------


double xPos = 0;
double yPos = 0;
double theta = 0;

void setPose(){
     xPos = chassis.getPose().x;
     yPos = chassis.getPose().y;
     theta = chassis.getPose().theta;
}

// ---------- mechanism control (callable from autonomous and opcontrol) ----------

// toggles intPos
void toggleIntPos() {
    intPos.set_value(!intPos.get_value());
}

void rollerSpin(int vel){
    roller.move(vel);
}

// moves cas motors to a degree. move_absolute hands the target to the motor's
// built-in position controller, which ramps down and keeps holding the target
// on its own - no software tolerance or brake latch, so nothing to oscillate
static const int casMaxRpm = 200; // green cartridge

static void casGoTo(double targetDeg) {
    casL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    casR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    casL.move_absolute(targetDeg, casMaxRpm);
    casR.move_absolute(targetDeg, casMaxRpm);
}

// blocking version - returns once both cas motors have arrived (or after 2s).
// the motors keep holding the target after this returns.
void setCasDegree(double targetDeg) {
    casGoTo(targetDeg);
    uint32_t start = pros::millis();
    while ((fabs(casL.get_position() - targetDeg) > 1.0 ||
            fabs(casR.get_position() - targetDeg) > 1.0) &&
           pros::millis() - start < 2000) {
        pros::delay(20);
    }
}

// non-blocking version - sets the target and returns immediately so cas can
// move at the same time as driving or other tasks
void setCasDegreeAsync(double targetDeg) {
    casGoTo(targetDeg);
}

// stops cas where it currently is
void stopCasAsync() {
    casL.brake();
    casR.brake();
}



// runs intake, drops cascade and tilter down (cas and tilter move in
// the background so this returns immediately and driving isn't blocked)
void intakeDown() {
    intake.move(127); // Intake in
    roller.move(127);
    setCasDegreeAsync(0); // Drop cascade to bottom
    goDownAsync();        // Tilt tilter all the way down
}

void intakeSpin(int vel) {
    intake.move(vel);
    roller.move(vel);
}

void justIntakeSpin(int vel) {
    intake.move(vel);
    roller.move(vel);
}
