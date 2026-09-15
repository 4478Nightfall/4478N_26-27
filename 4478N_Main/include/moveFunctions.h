#ifndef __FUNCTIONS__
#define __FUNCTIONS__

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
using namespace pros;

// ---------- drive / pid utils ----------
extern void setPose();
extern double slewStep;
extern double slew(double, double);
// fwdVal = inches (or deg for turn fns). maxSpeed 0-100
extern void drivePID(double, double maxSpeedPercent = 100.0, double timeout = 3000);
extern void driveForTime(int, int);
extern void drivePIDAsync(double, double maxSpeedPercent = 100.0, double timeout = 3000);
extern void stopDrivePIDAsync();
extern bool isDrivePIDRunning();
extern void turnToHeadingSmart(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);

// ---------- mechanism control (callable from autonomous and opcontrol) ----------
extern void toggleIntPos();
extern void rollerSpin(int vel);
extern void setCasDegree(double targetDeg);
extern void setCasDegreeAsync(double targetDeg);
extern void stopCasAsync();
extern void intakeDown();
extern void intakeSpin(int vel);

#endif
