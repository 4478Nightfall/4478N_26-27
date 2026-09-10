#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "api.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "moveFunctions.h"
#include <string>
#include <iostream>
#include <thread>
using namespace pros;
using namespace lemlib;

// ---------- dual imu ----------
// inherits pros imu, forwards everything to both and averages reads

DualIMU::DualIMU(pros::Imu* imu1, pros::Imu* imu2) 
    : pros::Imu(imu1->get_port()), imu1(imu1), imu2(imu2) {
}

std::int32_t DualIMU::reset(bool blocking) const {
    imu1->reset(blocking);
    return imu2->reset(blocking);
}

std::int32_t DualIMU::set_data_rate(std::uint32_t rate) const {
    imu1->set_data_rate(rate);
    return imu2->set_data_rate(rate);
}

// sensor reads - avg
double DualIMU::get_rotation() const {
    return (imu1->get_rotation() + imu2->get_rotation()) / 2.0;
}

double DualIMU::get_heading() const {
    return (imu1->get_heading() + imu2->get_heading()) / 2.0;
}

pros::quaternion_s_t DualIMU::get_quaternion() const {
    pros::quaternion_s_t q1 = imu1->get_quaternion();
    pros::quaternion_s_t q2 = imu2->get_quaternion();
    pros::quaternion_s_t avg;
    avg.x = (q1.x + q2.x) / 2.0;
    avg.y = (q1.y + q2.y) / 2.0;
    avg.z = (q1.z + q2.z) / 2.0;
    avg.w = (q1.w + q2.w) / 2.0;
    return avg;
}

pros::euler_s_t DualIMU::get_euler() const {
    pros::euler_s_t e1 = imu1->get_euler();
    pros::euler_s_t e2 = imu2->get_euler();
    pros::euler_s_t avg;
    avg.pitch = (e1.pitch + e2.pitch) / 2.0;
    avg.roll = (e1.roll + e2.roll) / 2.0;
    avg.yaw = (e1.yaw + e2.yaw) / 2.0;
    return avg;
}

double DualIMU::get_pitch() const {
    return (imu1->get_pitch() + imu2->get_pitch()) / 2.0;
}

double DualIMU::get_roll() const {
    return (imu1->get_roll() + imu2->get_roll()) / 2.0;
}

double DualIMU::get_yaw() const {
    return (imu1->get_yaw() + imu2->get_yaw()) / 2.0;
}

pros::imu_gyro_s_t DualIMU::get_gyro_rate() const {
    pros::imu_gyro_s_t g1 = imu1->get_gyro_rate();
    pros::imu_gyro_s_t g2 = imu2->get_gyro_rate();
    pros::imu_gyro_s_t avg;
    avg.x = (g1.x + g2.x) / 2.0;
    avg.y = (g1.y + g2.y) / 2.0;
    avg.z = (g1.z + g2.z) / 2.0;
    return avg;
}

pros::imu_accel_s_t DualIMU::get_accel() const {
    pros::imu_accel_s_t a1 = imu1->get_accel();
    pros::imu_accel_s_t a2 = imu2->get_accel();
    pros::imu_accel_s_t avg;
    avg.x = (a1.x + a2.x) / 2.0;
    avg.y = (a1.y + a2.y) / 2.0;
    avg.z = (a1.z + a2.z) / 2.0;
    return avg;
}

// tares + sets - do both imus
std::int32_t DualIMU::tare_rotation() const {
    imu1->tare_rotation();
    return imu2->tare_rotation();
}

std::int32_t DualIMU::tare_heading() const {
    imu1->tare_heading();
    return imu2->tare_heading();
}

std::int32_t DualIMU::tare_pitch() const {
    imu1->tare_pitch();
    return imu2->tare_pitch();
}

std::int32_t DualIMU::tare_yaw() const {
    imu1->tare_yaw();
    return imu2->tare_yaw();
}

std::int32_t DualIMU::tare_roll() const {
    imu1->tare_roll();
    return imu2->tare_roll();
}

std::int32_t DualIMU::tare() const {
    imu1->tare();
    return imu2->tare();
}

std::int32_t DualIMU::tare_euler() const {
    imu1->tare_euler();
    return imu2->tare_euler();
}

std::int32_t DualIMU::set_heading(const double target) const {
    imu1->set_heading(target);
    return imu2->set_heading(target);
}

std::int32_t DualIMU::set_rotation(const double target) const {
    imu1->set_rotation(target);
    return imu2->set_rotation(target);
}

std::int32_t DualIMU::set_yaw(const double target) const {
    imu1->set_yaw(target);
    return imu2->set_yaw(target);
}

std::int32_t DualIMU::set_pitch(const double target) const {
    imu1->set_pitch(target);
    return imu2->set_pitch(target);
}

std::int32_t DualIMU::set_roll(const double target) const {
    imu1->set_roll(target);
    return imu2->set_roll(target);
}

std::int32_t DualIMU::set_euler(const pros::euler_s_t target) const {
    imu1->set_euler(target);
    return imu2->set_euler(target);
}

// status - just use imu1 for orientation stuff, calibrating checks either
pros::ImuStatus DualIMU::get_status() const {
    return imu1->get_status();
}

bool DualIMU::is_calibrating() const {
    return imu1->is_calibrating() || imu2->is_calibrating();
}

imu_orientation_e_t DualIMU::get_physical_orientation() const {
    return imu1->get_physical_orientation();
}

// ---------- devices / lemlib setup ----------
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup right_motors({18, 20}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({13, -14}, pros::MotorGearset::blue);
pros::Motor casL(-7, pros::MotorGearset::blue);
pros::Motor casR(10, pros::MotorGearset::blue);
pros::Motor intake(-2, pros::MotorGearset::blue);
pros::Motor roller(-3, pros::MotorGearset::green);
pros::Motor rollerPos(-4, pros::MotorGearset::green);
pros::Rotation tilter(5);
int highVal = 0;
int midVal = 10000;
int downVal = 20000;
int casDownVal = 0;
bool high;
bool mid;
bool low = true;

void goHigh(){
    while(tilter.get_position() > highVal){
        rollerPos.move(50);
    }
    rollerPos.brake();
     low = false;
    high = true;
    mid = false;
}

void goMid(){
        while(tilter.get_position() > midVal){
            rollerPos.move(50);
       }
       rollerPos.brake();
        low = false;
    high = false;
    mid = true;

       while(tilter.get_position() < midVal){
            rollerPos.move(-50);
       }
       rollerPos.brake();
        low = false;
    high = false;
    mid = true;
}

void goDown(){
    while(tilter.get_position() < downVal){
        rollerPos.move(-50);
    }
    low = true;
    high = false;
    mid = false;
    rollerPos.brake();
}

pros::Motor mfl(-13 , pros::MotorGearset::blue);
pros::Motor mbl(-14, pros::MotorGearset::blue);
pros::Motor mfr(18, pros::MotorGearset::blue);
pros::Motor mbr(20, pros::MotorGearset::blue);
pros::Rotation autonSelector(7);
pros::Rotation hTracker(21);
pros::Imu imu1(6);
pros::Imu imu2(18);
DualIMU imu(&imu1, &imu2); // combined imu object

pros::Distance frontDistanceSensor(8);
pros::Distance backDistance(17);
pros::Distance leftDistanceSensor(1);
pros::Distance rightDistanceSensor(9);

Distance* frontDistance = &frontDistanceSensor;
Distance* backDistancePtr = &backDistance;
Distance* leftDistance = &leftDistanceSensor;
Distance* rightDistance = &rightDistanceSensor;
adi::Port intPos('A', E_ADI_DIGITAL_OUT);


lemlib::TrackingWheel horizontal_tracking_wheel(&hTracker, lemlib::Omniwheel::NEW_2, 1, 1);

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              10.5,
                              lemlib::Omniwheel::NEW_275,
                              450,
                              2);

lemlib::OdomSensors sensors(nullptr,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu1
);

lemlib::ControllerSettings lateral_controller(50,
                                              0,
                                              12,
                                              0,
                                              1,
                                              100,
                                              3,
                                              500,
                                              20);

lemlib::ControllerSettings angular_controller(2,
                                              0,
                                              11.5,
                                              0,
                                              2,
                                              200,
                                              4,
                                              300,
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

