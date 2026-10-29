#include "distanceOdom.h"
#include "main.h"
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "recfAuton.h"
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

// ---------- RECF auton routines ----------

void skills(){
    closeClaw();
    chassis.setPose(0,0,-90);
    drivePID(-17,100,800);
    chassis.turnToHeading(0,800);
    drivePIDAsync(40,100,2000);
    casGoTo(1000);
    delay(1500);
        drivePIDAsync(10,50,500);
    casGoTo(300);
    delay(300);
    openClaw(); //score preload
    delay(200);
        chassis.turnToHeading(0,800,{},false);
    drivePID(-32.5,100,1500);
    casGoTo(0);
    chassis.turnToHeading(-91,700);
    drivePID(24,50,1500);
    closeClaw();
    drivePID(-19,100,800);
    chassis.turnToHeading(2,800);
    drivePIDAsync(56,100,2000);
    casGoTo(1300);
    delay(1700);
            drivePIDAsync(10,50,500);
    casGoTo(800);
    delay(300);
    openClaw(); //score 2nd pin
    delay(200);
        chassis.turnToHeading(0,800,{},false);
    drivePID(-32.5,100,1500);
    casGoTo(0);
    chassis.turnToHeading(-91,800);
    drivePID(24,50,1500);
    closeClaw();
    drivePID(-18,100,800);
    chassis.turnToHeading(2,800);
    drivePIDAsync(62,60,2000);
    casGoTo(2000); 
    delay(1700);
            drivePIDAsync(10,50,500);
    casGoTo(1500);
    delay(300);
    openClaw(); //score 3rd pin
    delay(200);
            chassis.turnToHeading(0,800,{},false);
    drivePID(-32.5,100,1500);
    casGoTo(0);
    chassis.turnToHeading(-91,700);
    drivePID(24,50,1500);
    closeClaw();
    drivePID(-18,100,800);
    chassis.turnToHeading(0,800);
    drivePIDAsync(56,70,2000);
    casGoTo(2700);
    delay(1700);
            drivePIDAsync(10,50,500);
    casGoTo(1700);
    delay(300);
    openClaw(); //score 4nd pin
    delay(200);
    chassis.turnToHeading(0,800,{},false);
    drivePID(-33,100,1500);
    casGoTo(0);
    chassis.turnToHeading(-93,700);
    drivePID(30,50,1500);
    closeClaw();
    drivePID(-18,100,800);
    chassis.turnToHeading(2,800);
    drivePIDAsync(56,70,2000);
    casGoTo(3500);
    delay(1700);
    drivePIDAsync(10,50,500);
    casGoTo(2500);
    delay(300);
    openClaw(); //score 5th pin
    delay(200);
    chassis.turnToHeading(0,800,{},false);
    drivePID(-33,100,1500);
    casGoTo(0);
    chassis.turnToHeading(-91,800);
    drivePID(24,50,1500);
    closeClaw(); //first matchload for neutral
    drivePID(-40,100,1500);
    turnToHeadingSmart(0,1000,{},false);
    resetOdom(false, true, true, false);
    chassis.moveToPose(0,4,30,2000,{},true);
    casGoTo(1800);
    delay(1600);
    drivePID(10,50,500);
    casGoTo(1000);
    delay(400);
    openClaw(); //first score on mid
    delay(200);
    chassis.setPose(0,0,chassis.getPose().theta);
    chassis.moveToPose(-35,-60,30,2000,{.forwards = false}, false);
    casGoTo(0);
    chassis.turnToHeading(-92,900);
    drivePID(45,70,2000);
    closeClaw(); //2nd matchload mid
    drivePID(-40,100,1500);
        turnToHeadingSmart(0,1000,{},false);
    resetOdom(false, true, true, false);
    chassis.moveToPose(0,4,30,2000,{},true);
    casGoTo(2400);
    delay(1600);
    drivePID(10,50,500);
    casGoTo(1500);
    delay(400);
    openClaw(); //second score mid
    delay(200);
    chassis.setPose(0,0,chassis.getPose().theta);
    chassis.moveToPose(-35,-60,30,1600,{.forwards = false}, false);
    casGoTo(0);
    chassis.turnToHeading(-92,900);
    drivePID(45,70,2000);
    closeClaw(); //3rd matchload mid
    drivePID(-40,100,1500);
        casGoTo(3200);
        turnToHeadingSmart(0,1000,{},false);
    resetOdom(false, true, true, false);
    chassis.moveToPose(-0.5,4,30,2000,{},true);
    delay(1600);
    drivePID(10,50,500);
    casGoTo(2300);
    delay(400);
    openClaw(); //3rd score mid
    delay(200);
    chassis.setPose(0,0,chassis.getPose().theta);
    chassis.moveToPose(-35,-60,30,1600,{.forwards = false}, false);
    casGoTo(0);
    chassis.turnToHeading(-92,900);
    drivePID(45,70,2000);
    closeClaw();


    


}

void left3_4(){
    closeClaw();
    chassis.setPose(0,0,153);
    drivePIDAsync(45,100,600);
    casGoTo(800);
    delay(600);
    drivePIDAsync(10,50,500);
    casGoTo(200);
    delay(300);
    openClaw();
    chassis.turnToHeading(170,400);
    drivePID(-32,100,1050);
    
    casGoTo(0);
    chassis.turnToHeading(-90,700);
    drivePID(24,80,1000);
    closeClaw();
    drivePID(-17.5,100,700);
    chassis.turnToHeading(180,600);
    drivePIDAsync(52,100,1100);
    casGoTo(1300);
    delay(1100);
    drivePIDAsync(10,50,500);
    casGoTo(600);
    delay(300);
    openClaw();
    delay(200);
    drivePID(-32,100,1100);
    casGoTo(0);
    chassis.turnToHeading(-87,700);

    drivePID(24,80,1100);
    closeClaw();
    drivePID(-14 ,100,700);
        casGoTo(2000);
    chassis.turnToHeading(180,700);
    drivePID(52,80,1200);
    drivePIDAsync(10,50,1000);
    casGoTo(1600);
    delay(400);
    openClaw();
    drivePID(-32,100,1100);
    casGoTo(0);
    chassis.turnToHeading(-87,800);
    drivePID(24,100,1150);
    closeClaw();
    drivePID(-17,100,700);
    chassis.turnToHeading(180,700);
    drivePIDAsync(52,80,1300);
    casGoTo(2500);
    delay(1500);
    drivePIDAsync(10,50,500);
    casGoTo(2000);
    delay(300);
    openClaw();
    delay(200);
}

void left3_4Odom(){
    closeClaw();
    chassis.setPose(0,0,153);
    drivePIDAsync(45,100,600);
    casGoTo(800);
    delay(600);
    drivePIDAsync(10,50,500);
    casGoTo(200);
    delay(300);
    openClaw();
    chassis.turnToHeading(167,400);
    drivePID(-30,100,1050);
    
    casGoTo(0);
    chassis.turnToHeading(-86,700);
    drivePID(30,100,900);
    closeClaw();
    casGoTo(1250);
    drivePID(-17,100,700);
    resetOdom(true,false,false,true);
    chassis.moveToPose(-61.5,29,180,1500,{.minSpeed = 80},false);
    casGoTo(600);
    delay(200);
    openClaw();
    delay(200);
    drivePID(-28,100,800);
    casGoTo(0);
   chassis.turnToHeading(270,700,{},false);

    resetOdom(false,false,false,true);
    chassis.setPose(-46,chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-67,59,270,700,{.minSpeed = 80},false);
    closeClaw();
    casGoTo(1700);
    drivePID(-17 ,100,700);
    resetOdom(true,false,false,true);
    chassis.moveToPose(-60,30,180,1500,{},false);
    casGoTo(1300);
    delay(200);
    openClaw();
        delay(200);
    drivePID(-28,100,800);
    casGoTo(0);
    chassis.turnToHeading(-90,700,{},false);
    resetOdom(false,false,false,true);
    chassis.setPose(-46,chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-67,59,-90,700,{.minSpeed = 80},false);
    closeClaw();
        casGoTo(1200);
    drivePID(-35,100,700);
    resetOdom(true,false,false,true);
    chassis.moveToPose(10,45,130,1100,{.minSpeed = 80},false);
    casGoTo(500);
    delay(300);
    openClaw();
    delay(200);
}

void leftToggle(){
    closeClaw();
    chassis.setPose(0,0,-90);
    drivePID(-17,100,1000);
    chassis.turnToHeading(180,800);
    drivePIDAsync(40,100,2000);
    casGoTo(1000);
    delay(1500);
            drivePIDAsync(10,50,500);
    casGoTo(300);
    delay(300);
    openClaw();
    delay(200);
    drivePID(-32,100,1050);
    casGoTo(0);
    chassis.turnToHeading(-86,700);
    drivePID(24,50,1500);
    closeClaw();
    drivePID(-18,100,800);
    chassis.turnToHeading(180,800);
    drivePIDAsync(56,70,2000);
    drivePIDAsync(56,70,2000);
    casGoTo(1300);
    delay(1700);
            drivePIDAsync(10,50,500);
    casGoTo(600);
    delay(300);
    openClaw();
    delay(200);
    drivePID(-10,100,500);
    chassis.turnToHeading(90,600);
    drivePID(13,100,800);
    drivePID(14,100,800);
    chassis.turnToHeading(0,600);
    drivePID(-40,100,1300);
    chassis.turnToHeading(90,600);
    drivePID(-100,100,1400);
    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle`
    drivePID(6,100,500);

}

void right2_3(){
    closeClaw();
    chassis.setPose(0,0,-90);
    drivePID(-17,100,800);
    chassis.turnToHeading(2,800);
    drivePIDAsync(40,100,2000);
    casGoTo(1000);
    delay(1500);
        drivePIDAsync(10,50,500);
    casGoTo(300);
    delay(300);
    openClaw();
    delay(200);
    drivePID(-33,100,1050);
    casGoTo(0);
    chassis.turnToHeading(-90,700);
    drivePID(24,50,1500);
    closeClaw();
    drivePID(-19,100,800);
    chassis.turnToHeading(2,800);
    drivePIDAsync(56,100,2000);
    casGoTo(1300);
    delay(1700);
            drivePIDAsync(10,50,500);
    casGoTo(600);
    delay(300);
    openClaw();
    delay(200);
    drivePID(-33,100,1200);
    casGoTo(0);
    chassis.turnToHeading(-90,800);
    drivePID(24,50,1500);
    closeClaw();
    drivePID(-19,100,800);
    chassis.turnToHeading(0,800);
    drivePIDAsync(62,60,2000);
    casGoTo(2000); 
    delay(1700);
            drivePIDAsync(10,50,500);
    casGoTo(1400);
    delay(300);
    openClaw();
    delay(200);

    // chassis.swingToHeading();
}

void rightToggle(){
    closeClaw();
    chassis.setPose(0,0,-90);
    drivePID(-17,100,1000);
    chassis.turnToHeading(0,800);
    drivePIDAsync(40,100,2000);
    casGoTo(1000);
    delay(1500);
            drivePIDAsync(10,50,500);
    casGoTo(300);
    delay(300);
    openClaw();
    delay(200);
    drivePID(-31,100,1050);
    casGoTo(0);
    chassis.turnToHeading(-90,700);
    drivePID(24,50,1500);
    closeClaw();
    drivePID(-19,100,800);
    chassis.turnToHeading(0,800);
    drivePIDAsync(56,70,2000);
        drivePIDAsync(52.5,70,2000);
    casGoTo(1300);
    delay(1700);
            drivePIDAsync(10,50,500);
    casGoTo(600);
    delay(300);
    openClaw();
    delay(200);
    drivePID(-10,100,500);
    chassis.turnToHeading(90,600);
    drivePID(13,100,800);
    drivePID(14,100,800);
    chassis.turnToHeading(180,600);
    drivePID(-40,100,1200);
    chassis.turnToHeading(90,600);
    drivePID(-100,100,1400);
    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle`
    drivePID(6,100,500);
}

void elimsRight(){
    closeClaw();
    chassis.setPose(-66,-50,90);
    chassis.moveToPose(-6,-50,90,1450,{.lead = 0.02,.minSpeed = 100},true);
    casGoTo(1000);
    delay(1400);
        drivePIDAsync(10,50,500);
    casGoTo(400);
    delay(300);
    openClaw(); // first score
    delay(200);
    chassis.turnToHeading(70,1000);
    drivePID(-40,100,1500);
    casGoTo(0);
    chassis.turnToHeading(-107,800);
    drivePID(40,70,1600);
    closeClaw();
    chassis.turnToHeading(-90,500);
    drivePID(-17,100,800);
    chassis.turnToHeading(0,800);
    drivePIDAsync(56,100,2000);
    casGoTo(1300);
    delay(1700);
            drivePIDAsync(10,50,500);
    casGoTo(600);
    delay(300);
    openClaw();
    delay(200);
    drivePID(-31,100,1200);
    casGoTo(0);
    chassis.turnToHeading(-93,800);
    drivePID(30,50,2000);
    closeClaw();
    drivePID(-17,100,800);
    chassis.turnToHeading(0,800);
    drivePIDAsync(62,60,2000);
    casGoTo(1800); 
    delay(1700);
    drivePIDAsync(10,50,500);
    casGoTo(1000);
    delay(300);
    openClaw();
    delay(200);
}

void elimsLeft(){
    closeClaw();
    chassis.setPose(-64,62,100);
    casGoTo(1000);
    chassis.moveToPose(-4,50,100,2000,{},true);
    delay(1200);
    casGoTo(400);
    delay(300);
    openClaw(); // first score
    delay(200);
    chassis,
    drivePID(-40,100,2000);
    casGoTo(0);
    chassis.turnToHeading(-90,800);
    drivePID(30,100,1500);
    closeClaw(); //matchload 1
    chassis.setPose(-66.5,58,chassis.getPose().theta);
    drivePID(-34,100,1400);
    resetOdom(false,false,false,true);
    chassis.setPose(chassis.getPose().x,58,chassis.getPose().theta);
    chassis.moveToPose(0,0,160,3000,{},true); //score 1
    casGoTo(1500);
    delay(2300);
    casGoTo(600);
    delay(300);
    openClaw(); //score 1
    chassis.moveToPose(-33,58,160,3000,{.forwards = false},true);
    casGoTo(0);
    delay(2500);
    chassis.turnToHeading(-90,800);
    drivePID(40,100,1500);
    closeClaw(); //matchload 2
    chassis.setPose(-66.5,58,chassis.getPose().theta);
    drivePID(-34,100,1400);
    resetOdom(false,false,true,false);
    chassis.setPose(chassis.getPose().x,58,chassis.getPose().theta);
    chassis.moveToPose(0,0,160,3000,{},true); //score 2
    casGoTo(2000);
    delay(2300);
    casGoTo(1100);
    delay(300);
    openClaw(); //score 2

}

void elimsRightCup(){
    closeClaw();
    chassis.setPose(-63,-64,90);
    chassis.moveToPose(-6,-50,80,2200,{.lead = 0.02},true);
    casGoTo(1200);
    delay(1500);
        drivePIDAsync(-10,50,500);
    casGoTo(400);
    delay(300);
    openClaw(); // first score
    chassis.turnToHeading(80,500);
        drivePID(-15,100,1000);

    turnToHeadingSmart(0,1000,{},false);
    casGoTo(0);
    resetOdom(false, true, true, false);
    chassis.moveToPose(0.75,-25,30,2000,{.maxSpeed = 60},true);
    delay(1400);
    closeClaw();
    chassis.turnToHeading(0,800);
    drivePIDAsync(35,100,1700);
    casGoTo(1800);
    delay(1400);
    drivePIDAsync(10,50,500);
    casGoTo(600);
    delay(300);
    openClaw();
    delay(200);
    turnToHeadingSmart(20,800);
    drivePID(-70,100,1800);
    chassis.turnToHeading(-92,900);
    casGoTo(0);
    drivePID(35,100,1400);
    closeClaw();
    drivePID(-40,100,1100);
    casGoTo(1500);
    chassis.turnToHeading(77,800);
    drivePIDAsync(50,100,1700);
    delay(1700);
    drivePIDAsync(10,50,500);
    delay(200);
    casGoTo(800);
    openClaw();
    delay(200);
    // drivePID(-10,100,1000);
}

void elimsLeftCup(){
    closeClaw();
    chassis.setPose(-63,64,90);
    chassis.moveToPose(-6,50,100,2500,{.lead = 0.02},true);
    casGoTo(1200);
    delay(1500);
        drivePIDAsync(-10,50,500);
    casGoTo(400);
    delay(300);
    openClaw(); // first score
    chassis.turnToHeading(100,500);
        drivePID(-15,100,1000);

    turnToHeadingSmart(180,1000,{},false);
    casGoTo(0);
    resetOdom(false, true, false, true);
    chassis.moveToPose(-1,25,150,2000,{.maxSpeed = 60},true);
    delay(1300);
    closeClaw();
    chassis.turnToHeading(178,800);
    drivePIDAsync(45,100,1500);
    casGoTo(1800);
    delay(1400);
    drivePIDAsync(10,50,500);
    casGoTo(600);
    delay(300);
    openClaw();
    delay(200);
    turnToHeadingSmart(155,800);
    drivePID(-70,100,1800);
    chassis.turnToHeading(-88,900);
    casGoTo(0);
    drivePID(35,100,1400);
    closeClaw();
    drivePID(-40,100,1100);
    casGoTo(1500);
    chassis.turnToHeading(103,800);
    drivePIDAsync(50,100,1700);
    delay(1700);
    drivePIDAsync(10,50,500);
    delay(200);
    casGoTo(800);
    openClaw();
    delay(200);
    // drivePID(-10,100,1000);
}