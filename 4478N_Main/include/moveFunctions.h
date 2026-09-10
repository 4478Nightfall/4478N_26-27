#ifndef __FUNCTIONS__
#define __FUNCTIONS__

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
using namespace pros;

extern void setPose();
extern void holdPush();
extern void spinIntake(double=1); // -1 outtake, 1 intake
extern void scoreMiddle();
extern void scoreMiddleSkills();
extern void scoreLongClose(int time);
extern void scoreLongFar(int time);
extern void matchLoad(int timeMs = 600);
extern void stopIntake();
extern void handleHood();
extern void handleDescore();
extern void wing();
extern double slewStep;
extern double slew(double, double);
// fwdVal = inches (or deg for turn fns). maxSpeed 0-100
extern void drivePID(double, double maxSpeedPercent = 100.0, double timeout = 3000);
extern void turnPID(double, double maxSpeedPercent = 100.0);
extern void driveForTime(int, int);
extern void handleMLMech();
extern void handleMidGoal();
extern void adjustLongClose();
extern void startHandleMidGoalTask();
extern void drivePIDAsync(double, double maxSpeedPercent = 100.0, double timeout = 3000);
extern void stopDrivePIDAsync();
extern void turnToHeadingSmart(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);
extern void backIntoLongGoalML();

// ---------- mechanism control (callable from autonomous and opcontrol) ----------
extern void toggleIntPos();
extern void setCasDegree(double targetDeg);
extern void setCasDegreeAsync(double targetDeg);
extern void stopCasAsync();
extern void intakeDown();
extern void outtake();

#endif