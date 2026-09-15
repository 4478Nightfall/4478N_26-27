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

void leftFarClose(){
    chassis.setPose(-12,65,180);
    drivePID(-10,100,500);
    drivePID(10,100,500);//toggle
    chassis.setPose(-12,65,180);
    chassis.moveToPose(-23,-47,180,2000,{.forwards = false, .minSpeed = 50}, true);//go to first score
    setCasDegreeAsync(100);
    goMidAsync();
    toggleIntPos();
    delay(1000);
    rollerSpin(-70); //score first pin
    delay(200);
    rollerSpin(0);
    goHigh();
    turnToHeadingSmart(180,600); 
    toggleIntPos();
    drivePIDAsync(24,60,1500);// go to 3 stack
    intakeDown();
    delay(1500);
    setCasDegreeAsync(300);
    goMidAsync();
    chassis.setPose(-24,65,180);
    chassis.moveToPose(-23,-47,180,2000,{.forwards = false,.minSpeed = 60}, false); // go to goal 2 
    rollerSpin(-70); // score second pin
    goHighAsync();
    drivePID(10,100,700);
    intakeDown();
    resetOdom(true,false,false,true);
    chassis.turnToPoint(-44,-49,800);// face last pin
    chassis.moveToPose(-44,-49,80, 800);
    delay(300);
    chassis.turnToHeading(-95,800,{.maxSpeed = 100});
    setCasDegreeAsync(500);
    goMidAsync();
    chassis.moveToPose(-23,-47,-90,2000,{.forwards = false,.minSpeed = 60}, false); // go to goal 3 
    rollerSpin(-70);
     



}
