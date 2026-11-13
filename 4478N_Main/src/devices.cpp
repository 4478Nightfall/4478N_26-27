#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "api.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "vexAuton.h"
#include "moveFunctions.h"
#include <string>
#include <iostream>
#include <thread>
using namespace pros;
using namespace lemlib;

// ---------- devices / lemlib setup ----------
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor mfl(-7 , pros::MotorGearset::blue);
pros::Motor mbl(-8, pros::MotorGearset::blue);
pros::Motor mfr(3, pros::MotorGearset::blue);
pros::Motor mbr(5, pros::MotorGearset::blue);
pros::MotorGroup right_motors({3, 5}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({-7, -8}, pros::MotorGearset::blue);
pros::Motor casLTop(2, pros::MotorGearset::green);
pros::Motor casLBottom(-1, pros::MotorGearset::green);
pros::Motor casRTop(-9, pros::MotorGearset::green);
pros::Motor casRBottom(10, pros::MotorGearset::green);

int casDownVal = 10;
int casUpVal = 3200; // top limit (degrees) for casLTop

// pneumatic claw on the cascade
pros::adi::Port claw('E', E_ADI_DIGITAL_OUT);


pros::Rotation autonSelector(18);
pros::Rotation hTracker(17);
pros::Imu imu1(15);

pros::Distance frontDistanceSensor(16);
pros::Distance backDistance(20);
pros::Distance leftDistanceSensor(19);
pros::Distance rightDistanceSensor(11);
pros::Distance liftDistance(12);

Distance* frontDistance = &frontDistanceSensor;
Distance* backDistancePtr = &backDistance;
Distance* leftDistance = &leftDistanceSensor;
Distance* rightDistance = &rightDistanceSensor;


lemlib::TrackingWheel horizontal_tracking_wheel(&hTracker, lemlib::Omniwheel::NEW_2, -5.5);

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              13,
                              lemlib::Omniwheel::NEW_275,
                              450,
                              2);

lemlib::OdomSensors sensors(nullptr,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu1
);

lemlib::ControllerSettings lateral_controller(19,
                                              0,
                                              1,
                                              0,
                                              1,
                                              100,
                                              3,
                                              500,
                                              20);

lemlib::ControllerSettings angular_controller(6,
                                              0,
                                              34,
                                              0,
                                              1,
                                              100,
                                              3,
                                              500,
                                              0);

ExpoDriveCurve throttleCurve(3,
                             10,
                             1.019);

ExpoDriveCurve steerCurve(3,
                          10,
                          1.019);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttleCurve,
                        &steerCurve);

