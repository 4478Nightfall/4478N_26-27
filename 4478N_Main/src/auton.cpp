#include "distanceOdom.h"
#include "main.h"
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "pros/adi.h"
#include "sensor.h"
#include "moveFunctions.h"
#include "autonSelector.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <numeric>
using namespace pros;
using namespace lemlib;

// ---------- auton routines ----------

void dataloggingRoute(){
    startDatalogging();
    chassis.setPose(0,0,0);
    drivePID(48,100,2000);
    drivePID(-48,100,2000);
    stopDatalogging();
}

void PIDtuning(){
    chassis.setPose(0,0,0);
    chassis.moveToPose(0,48,0,3000,{});
    chassis.moveToPose(0,24,0,2000,{.forwards = false,.minSpeed = 50});
    turnToHeadingSmart(90,800);
    turnToHeadingSmart(45,800);
}

void allianceLeft(){
    chassis.setPose(-12,-65,180);

    drivePID(-5,100,300);
    drivePID(10,100,500);//toggle
    drivePID(-5,100,300);
    drivePID(10,100,800);//toggle
    intakeSpin(127);
    chassis.setPose(-15,-65,180);
        chassis.moveToPose(-19,-51,160,2000,{.forwards = false}, true);//go to first score
    goMid();
    delay(1000);
    rollerSpin(-127); //score first pin
    delay(1000);
    rollerSpin(0);
    goHigh();
    turnToHeadingSmart(270,1200); 
 
    chassis.setPose(-30,-54,chassis.getPose().theta);
    drivePID(10,100,500);
    chassis.turnToPoint(-46,-47,2000,{.forwards = false});
    chassis.moveToPose(-39,-47,90,2000,{.forwards = false});//go to second score
    goDown();
    drivePID(10,100,800);
    
     



}
