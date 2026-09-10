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
    drivePID(-10,100,1000);
    drivePID(10,100,1000);
    chassis.setPose(-12,65,180);
    chassis.moveToPose(-23,-47,180,2000,{.forwards = false, .minSpeed = 50}, true);
    setCasDegreeAsync(300);
    goMidAsync();
    toggleIntPos();
    delay(1000);
    setCasDegree(100);
    rollerMove(-70);
    delay(200);
    rollerMove(0);
    goHigh();
    turnToHeadingSmart(180,600);
    drivePIDAsync(24,60,1500);
    intakeDown();
    delay(800);
    toggleIntPos();
    drivePID(-7,50,600);
    drivePID(7,50,600);
    delay(600);
    setCasDegreeAsync(500);
    goMidAsync();
    chassis.setPose(-24,65,180);
    chassis.moveToPose(-23,-47,180,{.forwards = false,.minSpeed = 60}, false);
    setCasDegree(300);
    rollerMove(-70);


}
