#ifndef __FUNCTIONS__
#define __FUNCTIONS__

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "motionControl.h"
using namespace pros;

extern void setPose();
// ---------- mechanism control (callable from autonomous and opcontrol) ----------
extern void toggleIntPos();
extern void rollerSpin(int vel);
extern void setCasDegree(double targetDeg);
extern void setCasDegreeAsync(double targetDeg);
extern void stopCasAsync();
extern void intakeDown();
extern void intakeSpin(int vel);
extern void justIntakeSpin(int vel);

#endif
