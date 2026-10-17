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

// validation run for driveProfiled - same distances the roadmap lists
void profileTest(){
    left_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    driveProfiled(12);
    pros::delay(500);
    driveProfiled(48);
    pros::delay(500);
    driveProfiled(-12);
    pros::delay(500);
    driveProfiled(-36);
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
    drivePID(500,100,500);//toggle
    drivePID(-10,100,350);
    drivePID(500,100,350);//toggle
    chassis.setPose(-15,-65,chassis.getPose().theta);
    chassis.moveToPose(-10.5,-32.7,180,1300,{.forwards = false});//go to first roller
    resetOdom(true,false,false,false);
    chassis.setPose(-10.5, chassis.getPose().y, chassis.getPose().theta);
    // chassis.turnToHeading(225,1000);
        chassis.moveToPose(-30,-51,225,1300,{.forwards = false,.minSpeed = 60},true);//go to first score
    delay(1300);
    delay(400);
    drivePID(10,100,1500);
    chassis.turnToHeading(135,700);
    setCasDegreeAsync(700);
    drivePID(-11,40,1500);
    delay(200);
    drivePID(30,70,1300); 
    setCasDegreeAsync(0);
    chassis.turnToHeading(305,1000,{},true);
    drivePID(45,50,2400);
    delay(500);
    delay(100);
    delay(1200);
    chassis.turnToHeading(-13,1000);
    setCasDegreeAsync(1000);
    drivePID(-26,60,2200);
    setCasDegreeAsync(400);
    delay(400);
    setCasDegree(1500);
}

void allianceRight(){
    chassis.setPose(12,-65,180);
    drivePID(500,100,500);//toggle
    drivePID(-10,100,350);
    drivePID(500,100,350);//toggle
    chassis.setPose(15,-65,chassis.getPose().theta);
    chassis.moveToPose(15.5,-40.7,180,1300,{.forwards = false,.maxSpeed = 60});//go to first roller
    // chassis.turnToHeading(225,1000);
        chassis.moveToPose(-37,-52,315,1300,{.forwards = false,.minSpeed = 50},true);//go to first score
    delay(1300);
    delay(300);
    drivePID(14,100,1500);
    chassis.turnToHeading(225,700);
    setCasDegreeAsync(730);
    drivePID(-15,40,1500);
    delay(400);
    drivePID(30,70,1300); 
    setCasDegreeAsync(0);
    chassis.turnToHeading(55,1000,{},true);
    drivePID(45,50,2400);
    delay(500);
    delay(100);
    delay(1200);
    chassis.turnToHeading(10,1000);
    setCasDegreeAsync(1000);
    drivePID(-32,60,2200);
    setCasDegreeAsync(400);
    delay(400);
    setCasDegree(1500);
}

void neutralLeftSimple(){
    chassis.setPose(0,-65,270);
    drivePID(-5,100,300);
    drivePID(500,100,400);//toggle
    drivePID(-5,100,300);
    drivePID(500,100,400);//toggle
    chassis.setPose(-62,0,chassis.getPose().theta);
    drivePID(-10,100,500);
    chassis.turnToHeading(85,1000);
    drivePID(30,60,2000);
    delay(500); 
    drivePID(10,40,1000);
    delay(100);
    delay(1500);
    resetOdom(false,true,false,false);
    chassis.setPose(chassis.getPose().x,0,chassis.getPose().theta);
    chassis.moveToPose(-47,18,135,2000,{.forwards = false,.maxSpeed = 50},true);
    delay(300);
    setCasDegreeAsync(1700);
    delay(1000);
    delay(700);
    delay(300);
    setCasDegree(400);
    setCasDegree(1500);
    // chassis.turnToHeading(80,1000);
    // drivePID(7,100,700);
    // setCasDegreeAsync(0);
    // delay(400);
    // drivePID(10,100,1000);
    // delay(1000);
    // drivePID(-5,100,600);
    // chassis.turnToHeading(10,800);
    // drivePID(-70,100,3000);
}

void neutralLeft(){
    chassis.setPose(0,-65,270);
    drivePID(-5,100,300);
    drivePID(500,100,400);//toggle
    drivePID(-5,100,300);
    drivePID(500,100,400);//toggle
    chassis.setPose(-62,0,chassis.getPose().theta);
    drivePID(-10,100,500);
    chassis.turnToHeading(85,900);
    drivePID(30,60,2000);
    drivePID(6,40,600);
    delay(300);
    delay(100);
    delay(1000);
    resetOdom(false,true,false,false);
    chassis.setPose(chassis.getPose().x,0,chassis.getPose().theta);
    chassis.moveToPose(-49,20,135,2000,{.forwards = false,.maxSpeed = 70},true);
    delay(300);
    setCasDegreeAsync(1700);
    delay(600);
    delay(600);    
    setCasDegreeAsync(600);
    delay(300);
    setCasDegreeAsync(1900);
delay(200);
    chassis.turnToHeading(95,1000);
        delay(300);
        drivePIDAsync(24,50,1200);
    delay(200);
    setCasDegreeAsync(0);
    delay(1500);
    chassis.setPose(-9,20,chassis.getPose().theta);
        chassis.moveToPose(-45,-32.5,20,3000,{.forwards = false},true);
        delay(1200);
                delay(1000);
}

void helperSolo(){
    delay(500);
    drivePID(-9,100,1200);
    delay(400);
    drivePID(10,100,2000);
    chassis.turnToHeading(30,1000);
    drivePID(500,100,1000);
     drivePID(-5,100,500);
    drivePID(500,100,1000);
     drivePID(-7,100,500);
}

void neutralRightSimple(){
    chassis.setPose(0,-65,180);
    drivePID(-5,100,300);
    drivePID(500,100,400);//toggle
    drivePID(-5,100,300);
    drivePID(500,100,400);//toggle
    chassis.setPose(0,-62,chassis.getPose().theta);
    drivePID(-10,100,500);
    chassis.turnToHeading(-5,1000);
    drivePID(30,60,2000);
    delay(500); 
    drivePID(7,40,1000);
    delay(100);
    delay(1500);
    resetOdom(false,true,false,false);
    chassis.setPose(0,chassis.getPose().y,chassis.getPose().theta);
    chassis.moveToPose(18,-47,315,2000,{.forwards = false,.maxSpeed = 50},true);
    delay(300);
    setCasDegreeAsync(1700);
    delay(1000);
    delay(700);
    delay(300);
    setCasDegree(400);
    setCasDegree(1500);
    // chassis.turnToHeading(80,1000);
    // drivePID(7,100,700);
    // setCasDegreeAsync(0);
    // delay(400);
    // drivePID(10,100,1000);
    // delay(1000);
    // drivePID(-5,100,600);
    // chassis.turnToHeading(10,800);
    // drivePID(-70,100,3000);
}

void neutralRight(){
    chassis.setPose(0,-65,180);
    drivePID(-5,100,300);
    drivePID(500,100,400);//toggle
    drivePID(-5,100,300);
    drivePID(500,100,400);//toggle
    chassis.setPose(0,-62,chassis.getPose().theta);
    drivePID(-10,100,500);
    chassis.turnToHeading(-5,900);
    drivePID(30,60,2000);
    drivePID(6,40,600);
    delay(300);
    delay(100);
    delay(1000);
    resetOdom(false,true,false,false);
    chassis.setPose(0,chassis.getPose().y,chassis.getPose().theta);
    chassis.moveToPose(18,-47,315,2000,{.forwards = false,.maxSpeed = 70},true);
    delay(300);
    setCasDegreeAsync(1700);
    delay(600);
    delay(600);    
    setCasDegreeAsync(600);
    delay(300);
    setCasDegreeAsync(1500);
delay(200);
    chassis.turnToHeading(-5,1000);
drivePID(20,50,1200);
    drivePIDAsync(24,50,1200);
    delay(200);
    setCasDegreeAsync(0);
    delay(1500);
    chassis.setPose(9,-20,chassis.getPose().theta);
        chassis.moveToPose(-32.5,-45,70,3000,{.forwards = false},true);
        delay(1200);
                delay(1400);
}

void close(){
        chassis.setPose(0,-65,90);
                        setCasDegreeAsync(600);

    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle`
    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle
    chassis.setPose(-64,0,chassis.getPose().theta);
    drivePID(18,100,1000);    
    // resetOdom(false,true,false,false);
    chassis.turnToHeading(180,500);
                    setCasDegreeAsync(600);
    drivePID(22,100,1200);
    setCasDegreeAsync(200);
    delay(400);
    claw.set_value(false);
    delay(300);
    drivePID(-9,100,1000);
    setCasDegreeAsync(0);
    chassis.turnToHeading(-125,1200);
    drivePID(30,70,1500); //go to 3 stack
    pivot(true,400);
closeClaw();     
    turnToHeadingSmart(-10,1000);
    drivePID(-10,100,1000);
        drivePID(-10,100,1000);

        resetOdom(false, true, true, false);
    chassis.moveToPose(-47,-23,0,2000,{.maxSpeed = 60},true); // score second pin
    setCasDegreeAsync(1500);
    delay(1200);
    setCasDegreeAsync(500);
    delay(400);
    claw.set_value(false);
    delay(300);
    drivePID(-10,100,1000);
        setCasDegreeAsync(0);
        turnToHeadingSmart(0,1000);

    resetOdom(false, true, true, false);
    delay(300);
    chassis.moveToPose(-52,-45,155,2000,{.maxSpeed = 40}); //grab 3rd pin
    closeClaw();
    delay(200);

    chassis.moveToPose(-49,-25,0,2000,{.maxSpeed = 60},true); //score 3rd pin
    setCasDegreeAsync(2000);
    delay(1000);
    setCasDegreeAsync(1200);
    delay(400);
    openClaw();
}

void far(){
        chassis.setPose(0,-65,0);
                        setCasDegreeAsync(600);

    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle`
    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle
    chassis.setPose(-10,-56,chassis.getPose().theta);
    drivePID(18,100,1000);    
    resetOdom(false,true,false,false);
    chassis.setPose(0,chassis.getPose().y,chassis.getPose().theta);
    chassis.turnToHeading(270,500);
                    setCasDegreeAsync(600);
                    chassis.moveToPose(-23,-46,270,2000,{.maxSpeed = 70}); // score first
    setCasDegreeAsync(200);
    delay(400);
    claw.set_value(false);
    delay(300);
    drivePID(-9,100,1000);
    chassis.turnToHeading(0,1000);
    setCasDegreeAsync(0);
    resetOdom(false, true, false, false);
    chassis.setPose(-11, chassis.getPose().y, 0);
    chassis.moveToPose(-20,-28,-30,2000,{.maxSpeed = 50});//grab 2nd pin
    closeClaw();
    delay(200);
    chassis.moveToPose(-23.6,-43,180,2000,{.maxSpeed = 70},true); //score 2rd pin
    setCasDegreeAsync(1500);
    delay(1200);
    setCasDegreeAsync(500);
    delay(400);
    claw.set_value(false);
    delay(300);
    chassis.setPose(-23.6,-42,chassis.getPose().theta);
    drivePID(-20,100,1000);
    chassis.moveToPose(-43,-43,230,2000,{.maxSpeed = 40}); // grab third pin
    closeClaw();
    delay(200);
    chassis.moveToPose(-23.6,-43,90,2000,{.maxSpeed = 70},true); //score 3rd pin
    setCasDegreeAsync(2000);
    delay(1000);
    setCasDegreeAsync(1200);
    delay(400);
    openClaw();

}

void skillsHard(){
    chassis.setPose(0,-65,90);
                        setCasDegreeAsync(600);

    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle`
    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle
    chassis.setPose(-64,-10,chassis.getPose().theta);
    drivePID(18,100,1000);    
    // resetOdom(false,true,false,false);
    chassis.turnToHeading(180,500);
                    setCasDegreeAsync(600);
    drivePID(22,100,1200);
    setCasDegreeAsync(200);
    delay(400);
    claw.set_value(false);
    delay(300);
    drivePID(-9,100,1000);
    setCasDegreeAsync(0);
    chassis.turnToHeading(-125,1200);
    drivePID(30,70,1500); //go to 3 stack
    pivot(true,400);    
    closeClaw();     
    turnToHeadingSmart(-10,1000);
    drivePID(-10,100,1000);
        drivePID(-10,100,1000);

        resetOdom(false, true, true, false);
    chassis.moveToPose(-47,-23,0,2000,{.maxSpeed = 60},true); // score second pin
    setCasDegreeAsync(1500);
    delay(1200);
    setCasDegreeAsync(500);
    delay(400);
    claw.set_value(false);
    delay(300);
    drivePID(-10,100,1000);
        setCasDegreeAsync(0);
        turnToHeadingSmart(0,1000);

    resetOdom(false, true, true, false);
    delay(300);
    chassis.moveToPose(-52,-45,155,2000,{.maxSpeed = 40}); //grab 3rd pin
    closeClaw();
    delay(200);

    chassis.moveToPose(-49,-25,0,2000,{.maxSpeed = 60},true); //score 3rd pin
    setCasDegreeAsync(2000);
    delay(1000);
    setCasDegreeAsync(1200);
    delay(400);
    openClaw();
    chassis.turnToHeading(0,1000);
    drivePID(-36,50,2000);
            setCasDegreeAsync(0);
    turnToHeadingSmart(-90,1000);
    drivePID(24,50,1500);
    closeClaw(); // grab 4th pin 1st matchload
    drivePID(-24,50,1500);
    chassis.turnToHeading(0,1000);
    setCasDegreeAsync(2000);
    delay(1000);
    drivePID(36,50,1500);
    setCasDegree(1500);
    openClaw(); //score 4th pin 1st matchload
    delay(300);
        setCasDegreeAsync(0);
    drivePID(-36,50,1500);
    turnToHeadingSmart(-90,1000);
    drivePID(24,50,1500);
    closeClaw();// grab 4th pin 1st matchload
    drivePID(-24,50,1500);
    chassis.turnToHeading(0,1000);
    setCasDegreeAsync(2000);
    delay(1000);
    drivePID(36,50,1500);
    setCasDegree(1500);
    openClaw();//score 4th pin 1st matchload 
    delay(300);
        setCasDegreeAsync(0);
    drivePID(-36,50,1500);
}

void skillsEasy(){
    chassis.setPose(0,-65,90);
                        setCasDegreeAsync(600);

    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle`
    drivePID(6,100,500);
    drivePID(-500,100,500);//toggle
    chassis.setPose(-64,-10,chassis.getPose().theta);
    drivePID(18,100,1000);    
    // resetOdom(false,true,false,false);
    chassis.turnToHeading(180,500);
                    setCasDegreeAsync(600);
    drivePID(22,100,1200);
    setCasDegreeAsync(200);
    delay(400);
    claw.set_value(false); // score first
    delay(300);
    drivePID(-9,100,1000);
    setCasDegreeAsync(0);
    chassis.turnToHeading(90,1000);
    drivePID(15,100,1000);
    chassis.turnToHeading(0,1000);
    drivePID(-30,70,1500); 
    resetOdom(false, true, true, false);
    chassis.moveToPose(-43,-42,220,2000,{.maxSpeed = 40}); // grab 2nd pin
    closeClaw();
    delay(200);
    chassis.moveToPose(-23.6,-43,0,2000,{.maxSpeed = 70},true); //score 2nd pin
    setCasDegreeAsync(1500);
    delay(1000);
    setCasDegreeAsync(600);
    delay(400);
    claw.set_value(false);
    delay(300);
    drivePID(-36,100,1000);
    setCasDegreeAsync(0);
    turnToHeadingSmart(-90,1000);
    drivePID(24,50,1500);
    closeClaw(); // grab 3rd pin 1st matchload
    delay(300);
    drivePID(-24,1000,1200);
    chassis.turnToHeading(0,1000);
    drivePIDAsync(40,70,1500);
    setCasDegreeAsync(2000);
    delay(1000);
    setCasDegreeAsync(1000);
    delay(400);
    openClaw(); //score 3rd pin 1st matchload
    delay(300);
    drivePID(-36,100,1000);
    setCasDegreeAsync(0);
    turnToHeadingSmart(-90,1000);
    drivePID(24,50,1500);
    closeClaw(); // grab 4th pin 2nd matchload
    delay(300);
    drivePID(-24,1000,1200);
    chassis.turnToHeading(0,1000);
         setCasDegreeAsync(2300);
         delay(300);
    drivePIDAsync(37,60,1500);
    delay(1300);
    setCasDegreeAsync(1500);
    delay(400);
    openClaw(); //score 4th pin 2nd matchload
    delay(300);
    drivePID(-36,100,1000);
    setCasDegreeAsync(0);
    turnToHeadingSmart(-90,1000);
    drivePID(24,50,1500);
     closeClaw(); // grab 5th pin 3rd matchload
    delay(300);
    drivePID(-24,1000,1200);
    chassis.turnToHeading(0,1000);
    setCasDegreeAsync(2600);
    delay(600);
    drivePIDAsync(37,60,1500);
    delay(1300);
    setCasDegreeAsync(2000);
    delay(400);
    openClaw(); //score 5th pin 3rd matchload
    delay(300);
    drivePID(-36,100,1000);
    setCasDegreeAsync(0);
    turnToHeadingSmart(-90,1000);
    drivePID(24,50,1500);
}