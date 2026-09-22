#pragma once

// required files for devices
#include "main.h" 
#include "lemlib/chassis/chassis.hpp"

extern void drivePIDTune(double kP, double kI, double kD, double fwdVal, double maxSpeedPercent, double timeout);
extern void tuningLoop();
extern void pidTuningMode();
extern volatile bool pidTuningActive;
extern double kP;
extern double kI;
extern double kD;