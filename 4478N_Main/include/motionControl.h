#pragma once

#include "main.h"
#include "lemlib/chassis/chassis.hpp"

// ---------- drive / pid utils ----------
extern double slewStep;
extern double slew(double, double);
// fwdVal = inches (or deg for turn fns). maxSpeed 0-100
extern void drivePID(double, double maxSpeedPercent = 100.0, double timeout = 3000, bool claw = false);
extern void driveProfiled(double inches, double maxSpeedPercent = 100.0, double timeout = 3000);
extern double profileVelocityRpm(double progressRev, double remainingRev, double maxVelRpm);
extern void drivePIDAsync(double, double maxSpeedPercent = 100.0, double timeout = 3000);
extern void stopDrivePIDAsync();
extern bool isDrivePIDRunning();
extern void driveForTime(int, int);
extern void turnToHeadingSmart(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);

// ---------- motion profile limits ----------
// motor-side rpm (what move_velocity() takes). blue cartridge = 600, geared 36:48 -> 450 rpm at the wheel.
// upper bound only - if the robot plateaus lower, the profile still works since it's position-indexed
constexpr double profileMaxVelRpm = 600.0;
// measured with accelDecelTest() (datalog_1): accel ~1700 rpm/s (slowest motor 1580), so ~75% of that.
// the hold-brake decel (~8000 rpm/s) is a hard stop, not a usable ramp, so decel starts equal to accel - raise it if stops feel soft
constexpr double profileMaxAccelRpmPerS = 1250.0;
constexpr double profileMaxDecelRpmPerS = 1250.0;
// slowest speed the profile ever commands: lets accel start from rest and is the creep speed into the target
constexpr double profileMinVelRpm = 30.0;
// the profile is followed until this many inches from the target, then speed is commanded to 0.
// keep it small: the decel curve only reaches the creep speed (profileMinVelRpm) right at the target,
// so a big value cuts the curve off while it is still fast and the robot rolls past the target
constexpr double profileToleranceIn = 0.05;
