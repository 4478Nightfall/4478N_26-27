#ifndef __FUNCTIONS__
#define __FUNCTIONS__

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "motionControl.h"
using namespace pros;

extern void setPose();
// ---------- mechanism control (callable from autonomous and opcontrol) ----------
extern void casGoTo(double targetDeg);
extern void stopCasAsync();

extern void openClaw();
extern void closeClaw();
extern void pivot (bool wheels /*True Left spin, false, right spin*/, int direction, int timeout); 

#endif
