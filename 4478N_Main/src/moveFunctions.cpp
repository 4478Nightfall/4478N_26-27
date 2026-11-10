#include "main.h"
#include "api.h"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "vexAuton.h"
#include "moveFunctions.h"
#include <atomic>
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

void pivot(bool wheels/*True Left spin, false, right spin*/, int direction /*1 = forward, -1, backward*/, double heading, int timeout){
    if(wheels == true){
        right_motors.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        right_motors.brake();
        uint32_t start = pros::millis();
        while (pros::millis() - start < timeout) {
                    left_motors.move(50 * direction);
                    pros::delay(10);
        }

        left_motors.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        left_motors.brake();        
    }
    else if(wheels == false){
        left_motors.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        left_motors.brake();
        uint32_t start = pros::millis();
        while (pros::millis() - start < timeout) {
                    right_motors.move(50*direction);
                    pros::delay(10);
        }

        right_motors.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        right_motors.brake();        
    }
}

// ---------- mechanism control (callable from autonomous and opcontrol) ----------

// moves cas motors to a degree. move_absolute hands the target to the motor's
// built-in position controller, which ramps down and keeps holding the target
// on its own - no software tolerance or brake latch, so nothing to oscillate
static const int casMaxRpm = 200; // green cartridge

void casGoTo(double targetDeg) {
    casLTop.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    casLBottom.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    casRTop.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    casRBottom.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    casLTop.move_absolute(targetDeg, casMaxRpm);
    casLBottom.move_absolute(targetDeg, casMaxRpm);
    casRTop.move_absolute(targetDeg, casMaxRpm);
    casRBottom.move_absolute(targetDeg, casMaxRpm);
}

// ---------- claw ----------
void openClaw() { 
    claw.set_value(false); 
}
void closeClaw() { 
    claw.set_value(true);
    delay(200);
    casGoTo(430);
}
