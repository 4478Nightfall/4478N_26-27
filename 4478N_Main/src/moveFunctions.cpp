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




double slewStep = 20.0;
double slewRate = 0.5;

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
    
    double kP = 0.15; 
    double kI = 0.000000; 
    double kD = 0.18; 

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
    mml.tare_position();
    mmr.tare_position();

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
        double leftPositions[3] = {
            mbl.get_position(), mfl.get_position(), mml.get_position()
        };
        double rightPositions[3] = {
            mbr.get_position(), mfr.get_position(), mmr.get_position()
        };
        
        std::nth_element(leftPositions, leftPositions + 1, leftPositions + 3);
        double leftMedian = leftPositions[1];
        
        std::nth_element(rightPositions, rightPositions + 1, rightPositions + 3);
        double rightMedian = rightPositions[1];
        
        double medianPos = (leftMedian + rightMedian)/2;
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

    double kP = 0.2; 
    double kI = 0.000000; 
    double kD = 0.17; 

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
    mml.tare_position();
    mmr.tare_position();

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
        double leftPositions[3] = {
            mbl.get_position(), mfl.get_position(), mml.get_position()
        };
        double rightPositions[3] = {
            mbr.get_position(), mfr.get_position(), mmr.get_position()
        };
        
        std::nth_element(leftPositions, leftPositions + 1, leftPositions + 3);
        double leftMedian = leftPositions[1];
        
        std::nth_element(rightPositions, rightPositions + 1, rightPositions + 3);
        double rightMedian = rightPositions[1];
        
        double medianPos = (leftMedian + rightMedian);
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
