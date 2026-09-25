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

// 2 imus averaged together, see cpp for impl
class DualIMU : public pros::Imu {
private:
    pros::Imu* imu1;
    pros::Imu* imu2;
    
public:
    DualIMU(pros::Imu* imu1, pros::Imu* imu2);
    
    std::int32_t reset(bool blocking = false) const override;
    std::int32_t set_data_rate(std::uint32_t rate) const override;
    
    double get_rotation() const override;
    double get_heading() const override;
    pros::quaternion_s_t get_quaternion() const override;
    pros::euler_s_t get_euler() const override;
    double get_pitch() const override;
    double get_roll() const override;
    double get_yaw() const override;
    pros::imu_gyro_s_t get_gyro_rate() const override;
    pros::imu_accel_s_t get_accel() const override;
    
    std::int32_t tare_rotation() const override;
    std::int32_t tare_heading() const override;
    std::int32_t tare_pitch() const override;
    std::int32_t tare_yaw() const override;
    std::int32_t tare_roll() const override;
    std::int32_t tare() const override;
    std::int32_t tare_euler() const override;
    
    std::int32_t set_heading(const double target) const override;
    std::int32_t set_rotation(const double target) const override;
    std::int32_t set_yaw(const double target) const override;
    std::int32_t set_pitch(const double target) const override;
    std::int32_t set_roll(const double target) const override;
    std::int32_t set_euler(const pros::euler_s_t target) const override;
    
    pros::ImuStatus get_status() const override;
    bool is_calibrating() const override;
    imu_orientation_e_t get_physical_orientation() const override;
};

extern Controller controller;
extern MotorGroup left_motors;
extern MotorGroup right_motors;
extern Motor firstStage;
extern Motor middleStage;
extern Motor backBottom;
extern Motor intake;
extern Motor casL;
extern Motor casR;
extern Motor roller;
extern Motor tilter;
extern Rotation rollerPos;
extern int highVal;
extern int midVal;
extern int downVal;
extern int tilterSpeed;
extern int tilterTimeout;
extern int casDownVal;
extern bool high;
extern bool mid;
extern bool low;
extern void goHigh();
extern void goMid();
extern void goDown();
extern void goHighAsync();
extern void goMidAsync();
extern void goDownAsync();
extern void stopRollerPosAsync();
extern Rotation autonSelector;
extern Rotation hTracker;
extern Optical colorSort;
extern Optical counter;

extern Distance frontDistanceSensor;
extern Distance backDistance;
extern Distance leftDistanceSensor;
extern Distance rightDistanceSensor;

extern Distance* frontDistance;
extern Distance* backDistancePtr;
extern Distance* leftDistance;
extern Distance* rightDistance;

extern Imu imu1;
extern Imu imu2;
extern DualIMU imu;

extern pros::Motor mbl;
extern pros::Motor mbr;
extern pros::Motor mfl;
extern pros::Motor mfr;
extern adi::Port intPos;
extern adi::Port hood;
extern adi::Port matchloadMech;
extern adi::Port descore;
extern adi::Port midGoal;
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