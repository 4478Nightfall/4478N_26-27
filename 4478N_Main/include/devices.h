#pragma once

#include "main.h"
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include <string>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <numeric>

using namespace pros;
using namespace lemlib;

extern Controller controller;
extern MotorGroup left_motors;
extern MotorGroup right_motors;
extern Motor casLTop;
extern Motor casLBottom;
extern Motor casRTop;
extern Motor casRBottom;
extern int casDownVal;
extern int casUpVal;
extern Rotation autonSelector;
extern Rotation hTracker;

extern Distance frontDistanceSensor;
extern Distance backDistance;
extern Distance leftDistanceSensor;
extern Distance rightDistanceSensor;
extern Distance liftDistance;

extern Distance* frontDistance;
extern Distance* backDistancePtr;
extern Distance* leftDistance;
extern Distance* rightDistance;

extern Imu imu1;

extern pros::Motor mbl;
extern pros::Motor mbr;
extern pros::Motor mfl;
extern pros::Motor mfr;
extern adi::Port claw;
// extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;
enum TeamColor { red, blue, None };
extern TeamColor currentTeam;