#pragma once

#include "main.h"
#include "lemlib/chassis/chassis.hpp"

// ---------- drive / pid utils ----------
extern double slewStep;
extern double slew(double, double);
// fwdVal = inches (or deg for turn fns). maxSpeed 0-100
extern void drivePID(double, double maxSpeedPercent = 100.0, double timeout = 3000);
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
// PLACEHOLDERS - conservative guesses for the current robot. measure on the finished robot
// with accelDecelTest() and set these to ~70-80% of the measured values
constexpr double profileMaxAccelRpmPerS = 1000.0;
constexpr double profileMaxDecelRpmPerS = 1000.0;
// slowest speed the profile ever commands: lets accel start from rest and is the creep speed into the target
constexpr double profileMinVelRpm = 30.0;
// last stretch of the move is a plain position P-loop instead of the profile (inches / inches)
constexpr double profileTailZoneIn = 1.0;
constexpr double profileToleranceIn = 0.25;
