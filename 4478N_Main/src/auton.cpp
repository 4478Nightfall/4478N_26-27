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
    drivePID(-2,100,200);
    drivePID(10,100,300);//toggle
    drivePID(-5,100,200);
    drivePID(10,100,300);//toggle
    intakeSpin(127);
    chassis.setPose(-15,-65,chassis.getPose().theta);
    chassis.moveToPose(-10.5,-37.7,180,1300,{.forwards = false});//go to first roller
    // chassis.turnToHeading(225,1000);
        chassis.moveToPose(-37,-52,225,1500,{.forwards = false,.minSpeed = 50},true);//go to first score
        goMidAsync();
    delay(1200);
    rollerSpin(-70); //score first pin
    delay(300);
    rollerSpin(0);
    drivePID(10,100,500);
    chassis.turnToHeading(135,700);
    setCasDegreeAsync(750);
    drivePID(-16,50,1500);
    goDown();
    delay(200);
    drivePID(30,70,1000); 
    setCasDegreeAsync(0);
    goDownAsync();
    chassis.turnToHeading(310,1000,{},true);
    intakeSpin(127);
    drivePID(40,50,2500);
    delay(2000);
    chassis.turnToHeading(-6,1000);
    goMidAsync();
    setCasDegreeAsync(1000);
    drivePID(-30,60,2200);
    goMid();
    setCasDegreeAsync(400);
    delay(400);
    rollerSpin(-127);
    setCasDegree(1500);
}

void neutralLeft(){
    chassis.setPose(0,-65,270);
    drivePID(-2,100,200);
    drivePID(10,100,300);//toggle
    drivePID(-5,100,200);
    drivePID(10,100,300);//toggle
    intakeSpin(127);
    chassis.setPose(-62,0,chassis.getPose().theta);
    drivePID(-10,100,500);
    chassis.turnToHeading(85,1000);
    drivePID(30,60,2000);
    delay(500); 
    drivePID(10,60,1000);
    delay(2000);
    resetOdom(false,true,false,false);
    chassis.setPose(chassis.getPose().x,0,chassis.getPose().theta);
    chassis.moveToPose(-47,18,135,2000,{.forwards = false,.minSpeed = 50},true);
    delay(300);
    setCasDegreeAsync(1700);
    goMidAsync();
    delay(1700);
    delay(300);
    setCasDegreeAsync(400);
    delay(600);
    rollerSpin(-127);
    setCasDegree(1500);
    // chassis.turnToHeading(80,1000);
    // drivePID(7,100,700);
    // goDownAsync();
    // setCasDegreeAsync(0);
    // intakeSpin(127);
    // drivePID(10,100,1000);
    // delay(1000);
    // drivePID(-5,100,600);
    // chassis.turnToHeading(15,800);
    // drivePIDAsync(-55,100,2000);
    // delay(600);
    // goMidAsync();
    // delay(2000);
    // rollerSpin(-127);
}
