#include "distanceOdom.h"
#include "main.h"
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "vexAuton.h"
#include "pros/adi.h"
#include "sensor.h"
#include "moveFunctions.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <numeric>
using namespace pros;
using namespace lemlib;

// ---------- auton routines ----------



void vexSkills(){
    allianceClose3Pin();
}

void allianceCloseAlleys(){
    closeClaw();
    chassis.setPose(-65,0,90);
    drivePID(-500,100,300);//toggle
    drivePID(4,100,200);
    drivePID(-500,100,300);//toggle
    chassis.setPose(-65,0,chassis.getPose().theta);
    casGoTo(800);
    chassis.moveToPose(-51,-22,160,1300,{.minSpeed = 90},false); //score preload
    drivePIDAsync(10,30,500);
    casGoTo(0);
    delay(200);
    openClaw();
    drivePID(-9,100,650);
    casGoTo(0);
    chassis.turnToHeading(220,800);
    chassis.setPose(-51,-7,chassis.getPose().theta);
    chassis.moveToPose(-65,-25,270,1000,{.lead = 0.3},false);
    drivePID(20,100,500);
    closeClaw();
    left_motors.move(-70);
    delay(300);
    casGoTo(1600);
    chassis.turnToHeading(0,700);
    resetOdom(false,true,true,false);
    chassis.setPose(-59,chassis.getPose().y,chassis.getPose().theta);
    chassis.moveToPose(-56,0,20, 800,{.minSpeed = 80, .earlyExitRange = 10},false);
    chassis.moveToPose(-55.5,23,30,700,{},false); //score preload
    drivePIDAsync(10,30,500);
    casGoTo(600);
    delay(200);
    openClaw();
    delay(100);
    drivePID(-10,100,650);
    casGoTo(0);
    chassis.turnToHeading(310,700);
    chassis.setPose(-48,15,chassis.getPose().theta);
    chassis.moveToPose(-65,25,270,1000,{.lead = 0.3},false);
    drivePID(20,100,500);
    closeClaw();
    right_motors.move(-70);
    delay(200);
    casGoTo(1400);
    chassis.turnToHeading(180,700);
    resetOdom(false,true,false,true);
    chassis.setPose(-59,chassis.getPose().y,chassis.getPose().theta);
    chassis.moveToPose(-44,0,160, 800,{.minSpeed = 80, .earlyExitRange = 10},false);
    chassis.moveToPose(-44,-23,180,700,{},false); //score preload
    drivePIDAsync(10,30,1000);
    casGoTo(600);
    delay(200);
    openClaw();
    delay(100);
}

void allianceClose3Pin(){
    closeClaw();
    chassis.setPose(-65,0,90);
    drivePID(-500,100,300);//toggle
    drivePID(4,100,200);
    drivePID(-500,100,300);//toggle
    chassis.setPose(-65,0,chassis.getPose().theta);
    casGoTo(800);
    chassis.moveToPose(-51,-22,160,1300,{.minSpeed = 90},false); //score preload
    drivePIDAsync(10,30,500);
    casGoTo(0);
    delay(200);
    openClaw();
    chassis.setPose(-49.5,-17.5,chassis.getPose().theta);
    drivePID(-9,100,650);
    chassis.turnToHeading(120,800);
    chassis.moveToPose(-29,-20,120,1000,{},false);
    autoClamp();
    casGoTo(1400);
    chassis.moveToPose(-40,-22,-100,1000,{},false);
    releaseStack();
    drivePID(-10,100,800);
    casGoTo(0);
    chassis.moveToPose(-44.5,-42.7,210,1200,{},false);
    autoClamp();
    casGoTo(2000);
    chassis.moveToPose(-45.5,-31,-20,1000,{},false);
    releaseStack();
}

void dataloggingRoute(){
    startDatalogging();
    chassis.setPose(0,0,0);
    drivePID(48,100,2000);
    drivePID(-48,100,2000);
    stopDatalogging();
}

// measures real accel + decel: full-speed velocity command for 2s, then brake hold.
// needs ~10-12ft of clear runway. read the fast datalog csv afterwards
void accelDecelTest(){
    startFastDatalogging();
    pros::delay(200); // rest baseline

    datalogMarker = 1;
    left_motors.move_velocity(600);
    right_motors.move_velocity(600);
    pros::delay(1200);

    datalogMarker = 2;
    left_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    left_motors.brake();
    right_motors.brake();
    pros::delay(1000); // capture the whole stop

    stopFastDatalogging();
}

void PIDtuning(){
    chassis.setPose(0,0,0);
    chassis.moveToPose(0,48,0,2000,{},false);
    chassis.moveToPose(0,24,0,2000,{.forwards = false},false);
    chassis.moveToPose(0,12,0,2000,{.forwards = false},false);
    chassis.moveToPose(24,36,90,2000,{.horizontalDrift = 8, .lead = 0.5},false);
}