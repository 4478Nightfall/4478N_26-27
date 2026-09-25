#include "main.h"
#include "api.h"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "moveFunctions.h"
#include <string>
#include <iostream>
#include <thread>
#include <algorithm>
#include <atomic>
#include <memory>
#include <cmath>
#include "pros/rtos.hpp"

using namespace pros;
using namespace lemlib;

// ---------- drive / intake / pid utils ----------


double xPos = 0;
double yPos = 0;
double theta = 0;

void setPose(){
     xPos = chassis.getPose().x;
     yPos = chassis.getPose().y;
     theta = chassis.getPose().theta;
}

// ---------- mechanism control (callable from autonomous and opcontrol) ----------

// toggles intPos
void toggleIntPos() {
    intPos.set_value(!intPos.get_value());
}

void rollerSpin(int vel){
    roller.move(vel);
}

// moves cas motors to a degree. move_absolute hands the target to the motor's
// built-in position controller, which ramps down and keeps holding the target
// on its own - no software tolerance or brake latch, so nothing to oscillate
static const int casMaxRpm = 200; // green cartridge

static void casGoTo(double targetDeg) {
    casL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    casR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    casL.move_absolute(targetDeg, casMaxRpm);
    casR.move_absolute(targetDeg, casMaxRpm);
}

// blocking version - returns once both cas motors have arrived (or after 2s).
// the motors keep holding the target after this returns.
void setCasDegree(double targetDeg) {
    casGoTo(targetDeg);
    uint32_t start = pros::millis();
    while ((fabs(casL.get_position() - targetDeg) > 1.0 ||
            fabs(casR.get_position() - targetDeg) > 1.0) &&
           pros::millis() - start < 2000) {
        pros::delay(20);
    }
}

// non-blocking version - sets the target and returns immediately so cas can
// move at the same time as driving or other tasks
void setCasDegreeAsync(double targetDeg) {
    casGoTo(targetDeg);
}

// stops cas where it currently is
void stopCasAsync() {
    casL.brake();
    casR.brake();
}



// runs intake, drops cascade and tilter down (cas and tilter move in
// the background so this returns immediately and driving isn't blocked)
void intakeDown() {
    intake.move(127); // Intake in
    roller.move(127);
    setCasDegreeAsync(0); // Drop cascade to bottom
    goDownAsync();        // Tilt tilter all the way down
}

void intakeSpin(int vel) {
    intake.move(vel);
    roller.move(vel);
}

void spinAroundGoal(double heading, double timeout){

    left_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    uint32_t start = pros::millis();

    // wrapped error: positive = target is ahead in heading, negative = behind (shortest way)
    double error = lemlib::angleError(heading, chassis.getPose().theta, false);

    if (error > 0){
        while (error > 0 && pros::millis() - start < timeout){
            error = lemlib::angleError(heading, chassis.getPose().theta, false);
            left_motors.move(-30);
            right_motors.move(-100);
            pros::delay(10);
        }
        left_motors.brake();
        right_motors.brake();
    }
    else if (error <= 0){
        while (error <= 0 && pros::millis() - start < timeout){
            error = lemlib::angleError(heading, chassis.getPose().theta, false);
            left_motors.move(-100);
            right_motors.move(-30);
            pros::delay(10);
        }
        left_motors.brake();
        right_motors.brake();
    }

}

double slewStep = 15.0;
double slewRate = 0.2;

double slew(double val, double fwdVal)
{
    static double prevVal = 0;
    static double prevFwdVal = 0;

    // Reset slew if target direction changes significantly
    if ((fwdVal >= 0 && prevFwdVal < 0) || (fwdVal < 0 && prevFwdVal >= 0))
    {
        prevVal = 0;
    }
    prevFwdVal = fwdVal;

    // Calculate the difference between target and current
    double difference = val - prevVal;

    // If difference is small, just return the target
    if (fabs(difference) < 0.5)
    {
        prevVal = val;
        return val;
    }

    // Calculate slew rate based on difference (adaptive)
    double currentSlewStep = slewStep + (fabs(difference) * slewRate);

    // Apply slew rate limiting
    if (difference > 0)
    {
        // Positive direction
        if (prevVal + currentSlewStep < val)
        {
            prevVal += currentSlewStep;
        }
        else
        {
            prevVal = val;
        }
    }
    else
    {
        // Negative direction
        if (prevVal - currentSlewStep > val)
        {
            prevVal -= currentSlewStep;
        }
        else
        {
            prevVal = val;
        }
    }

    return prevVal;
}

void drivePID(double fwdVal, double maxSpeedPercent, double timeout)
{
    
    double kP = 0.21; 
    double kI = 0.000000; 
    double kD = 0.12; 

    const double diameter = 3.25;
    const double pi = 3.14159;
    const double outputGear = 48;
    const double inputGear = 36;

    double num = fwdVal;
    double denom = (diameter * pi) * (inputGear / outputGear);
    double target = (num / denom) * 360;
    
    left_motors.tare_position();
    right_motors.tare_position();
    mbl.tare_position();
    mbr.tare_position();
    mfl.tare_position();
    mfr.tare_position();

    double startTime = pros::millis();
    double error = 0;
    double prevError = 0;
    double integral = 0;
    
    const double pollingRate = 20;
    const double errorThreshold = 5.0;
    int inGoal = 0;
    const int goalsNeeded = 2;
    if (maxSpeedPercent < 0)
        maxSpeedPercent = 0;
    if (maxSpeedPercent > 100)
        maxSpeedPercent = 100;
    double maxMotor = (maxSpeedPercent / 100.0) * 127.0;

    bool hasMoved = false;
    
    while (inGoal < goalsNeeded)
    {
        double leftAvg = (mbl.get_position() + mfl.get_position()) / 2;
        double rightAvg = (mbr.get_position() + mfr.get_position()) / 2;

        double medianPos = (leftAvg + rightAvg)/2;
        double processVariable = medianPos * 360;
        
        error = target - processVariable;
        
        // Track if robot has moved - for small movements, lower threshold and longer time
        // This prevents immediate exit at start while allowing small movements to work
        if (fabs(processVariable) > 5 || fabs(error) < errorThreshold || (pros::millis() - startTime) > 150) {
            hasMoved = true;
        }

        double errorInches = fabs(error) * (denom / 360.0);
        double currentKP = kP;
        double currentKD = kD;
        

       
        // Proportional term
        double P = error * currentKP;
        
        // Integral term with anti-windup 
        integral += error;
        double I = integral * kI;
        I = std::clamp(I, -50.0, 50.0); // Clamp integral contribution
        
        // Derivative term (using scheduled gain)
        double D = ((error - prevError) / pollingRate) * currentKD;
        
        // Calculate total PID output
        double motorPower = P + I + D;
        
        // Apply slew only at start for smooth acceleration
        if ((pros::millis() - startTime) < 300) {
            motorPower = slew(motorPower, fwdVal);
        }
        
        // Clamp output
        motorPower = std::clamp(motorPower, -maxMotor, maxMotor);

        // Move motors
        left_motors.move(motorPower);
        right_motors.move(motorPower);

        if (hasMoved && fabs(error) < errorThreshold)
        {
            inGoal++;
            if (inGoal >= goalsNeeded) {
                break;
            }
        }
        else
        {
            inGoal = 0;
        }

        if ((pros::millis() - startTime) >= timeout)
        {
            break;
        }

        prevError = error;
        
        pros::delay(pollingRate);
    }
    
    left_motors.brake();
    right_motors.brake();
}

static std::atomic<bool> drivePidStopRequested(false);
static std::atomic<bool> drivePidRunning(false);
static pros::Task* drivePidTaskPtr = nullptr;

struct DrivePidArgs {
    double fwdVal;
    double maxSpeedPercent;
    double timeout;
};

static void drivePidTaskFn(void* rawArgs) {
    std::unique_ptr<DrivePidArgs> args(reinterpret_cast<DrivePidArgs*>(rawArgs));
    double fwdVal = args->fwdVal;
    double maxSpeedPercent = args->maxSpeedPercent;
    double timeout = args->timeout;

    drivePidRunning = true;
    drivePidStopRequested = false;

    double kP = 0.21; 
    double kI = 0.000000; 
    double kD = 0.12; 

    const double diameter = 3.25;
    const double pi = 3.14159;
    const double outputGear = 48;
    const double inputGear = 36;

    double num = fwdVal;
    double denom = (diameter * pi) * (inputGear / outputGear);
    double target = (num / denom) * 360;

    left_motors.tare_position();
    right_motors.tare_position();
    mbl.tare_position();
    mbr.tare_position();
    mfl.tare_position();
    mfr.tare_position();

    double startTime = pros::millis();
    double error = 0;
    double prevError = 0;
    double integral = 0;

    const double pollingRate = 20;
    const double errorThreshold = 5.0;
    int inGoal = 0;
    const int goalsNeeded = 3;
    if (maxSpeedPercent < 0)
        maxSpeedPercent = 0;
    if (maxSpeedPercent > 100)
        maxSpeedPercent = 100;
    double maxMotor = (maxSpeedPercent / 100.0) * 127.0;

    bool hasMoved = false;

    while (inGoal < goalsNeeded && !drivePidStopRequested) {
        double leftAvg = (mbl.get_position() + mfl.get_position()) / 2;
        double rightAvg = (mbr.get_position() + mfr.get_position()) / 2;

        double medianPos = (leftAvg + rightAvg);
        double processVariable = medianPos * 360;
        
        error = target - processVariable;
        
        if (fabs(processVariable) > 5 || (pros::millis() - startTime) > 200) {
            hasMoved = true;
        }

        double errorInches = fabs(error) * (denom / 360.0);
        double currentKP = kP;
        double currentKD = kD;
        
       
        // Proportional term
        double P = error * currentKP;
        
        // Integral term with anti-windup 
        integral += error;
        double I = integral * kI;
        I = std::clamp(I, -50.0, 50.0); // Clamp integral contribution
        
        // Derivative term (using scheduled gain)
        double D = ((error - prevError) / pollingRate) * currentKD;

        // Calculate total PID output
        double motorPower = P + I + D;
        
        // Apply slew only at start for smooth acceleration
        if ((pros::millis() - startTime) < 300) {
            motorPower = slew(motorPower, fwdVal);
        }
        
        // Clamp output
        motorPower = std::clamp(motorPower, -maxMotor, maxMotor);

        // Move motors
        left_motors.move(motorPower);
        right_motors.move(motorPower);

        if (hasMoved && fabs(error) < errorThreshold && (pros::millis() - startTime) > 100)
        {
            inGoal++;
            if (inGoal >= goalsNeeded) {
                break;
            }
        }
        else
        {
            inGoal = 0;
        }

        if ((pros::millis() - startTime) >= timeout)
        {
            break;
        }

        prevError = error;
        
        pros::delay(pollingRate);
    }

    left_motors.brake();
    right_motors.brake();

    drivePidRunning = false;
    return;
}

void drivePIDAsync(double fwdVal, double maxSpeedPercent, double timeout) {
    if (drivePidRunning) {
        drivePidStopRequested = true;
        for (int i = 0; i < 50 && drivePidRunning; ++i) {
            pros::delay(10);
        }
    }

    auto* args = new DrivePidArgs{fwdVal, maxSpeedPercent, timeout};

    if (drivePidTaskPtr != nullptr && !drivePidRunning) {
        delete drivePidTaskPtr;
        drivePidTaskPtr = nullptr;
    }

    drivePidTaskPtr = new pros::Task(
        drivePidTaskFn,
        args,
        TASK_PRIORITY_DEFAULT,
        4096,
        "drivePIDAsync");
}
// ...existing code...

void stopDrivePIDAsync() {
    drivePidStopRequested = true;
    for (int i = 0; i < 100 && drivePidRunning; ++i) {
        pros::delay(10);
    }
    if (drivePidTaskPtr != nullptr && !drivePidRunning) {
        delete drivePidTaskPtr;
        drivePidTaskPtr = nullptr;
    }
}

bool isDrivePIDRunning() {
    return static_cast<bool>(drivePidRunning);
}

void driveForTime(int power, int time)
{
    left_motors.move(power);
    right_motors.move(power);
    pros::delay(time);
    left_motors.brake();
    right_motors.brake();
}


void turnToHeadingSmart(float theta, int timeout, TurnToHeadingParams params, bool async)
{
    // Get current heading
    float currentHeading = chassis.getPose().theta;
    
    // Calculate angle difference
    float angleDiff = theta - currentHeading;
    
    // Normalize to [-180, 180]
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    if (params.minSpeed == 0)
    {
        float absAngleDiff = fabs(angleDiff);
        
        if (absAngleDiff < 15.0)
        {
            params.minSpeed = 100; // Higher minimum speed for very small turns
            params.earlyExitRange = 0; // Tighter exit range for precision
        }
        else if (absAngleDiff < 30.0)
        {
            params.minSpeed = 90; // Moderate minimum speed
            params.earlyExitRange = 0;
        }
    }
    
    chassis.turnToHeading(theta, timeout, params, async);
}
