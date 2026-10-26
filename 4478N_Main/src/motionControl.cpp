#include "main.h"
#include "api.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "motionControl.h"
#include "moveFunctions.h"
#include <algorithm>
#include <atomic>
#include <memory>
#include <cmath>
#include "pros/rtos.hpp"

using namespace pros;
using namespace lemlib;

// ---------- motion control: everything that drives the robot around ----------
// slew, drivePID (sync + async), turn helpers, timed drive.
// motion profiling goes in here too.

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

void drivePID(double fwdVal, double maxSpeedPercent, double timeout, bool claw)
{
    
    double kP = 0.15; 
    double kI = 0.000000; 
    double kD = 0.02; 

    const double diameter = 2.75;
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

        // Claw mode: object is in the claw, so stop right now (brake happens after the loop)
        if (claw == true && frontDistanceSensor.get() < 100)
        {
            closeClaw();
            break;
        }

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

    double kP = 0.20; 
    double kI = 0.000000; 
    double kD = 0.12; 

    const double diameter = 2.75;
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

// ---------- 1D trapezoidal motion profile ----------
// everything here is in motor units: rotations of the motor, rpm of the motor.
// 1 motor rotation = (wheel circumference * 36/48) inches of travel

// target speed for a given spot in the move. this is the only place the profile
// shape lives, so swapping trapezoid for an s-curve later means editing just this function.
// progressRev = distance already covered, remainingRev = distance left (both >= 0, motor rotations)
double profileVelocityRpm(double progressRev, double remainingRev, double maxVelRpm)
{
    const double vMin = profileMinVelRpm / 60.0;             // rev/s
    const double accel = profileMaxAccelRpmPerS / 60.0;      // rev/s^2
    const double decel = profileMaxDecelRpmPerS / 60.0;      // rev/s^2

    // v^2 = v0^2 + 2*a*d  -> fastest speed allowed this far into the move
    double vAccel = std::sqrt(vMin * vMin + 2.0 * accel * std::max(progressRev, 0.0));
    // same equation run backwards from the target -> fastest speed we can still stop from
    double vDecel = std::sqrt(vMin * vMin + 2.0 * decel * std::max(remainingRev, 0.0));

    double vRev = std::min({maxVelRpm / 60.0, vAccel, vDecel});
    return vRev * 60.0;
}

// motor position in rotations no matter what encoder units the motor was set to
static double motorRotations(pros::Motor& m)
{
    double pos = m.get_position();
    return (m.get_encoder_units() == pros::v5::MotorUnits::degrees) ? pos / 360.0 : pos;
}

void driveProfiled(double inches, double maxSpeedPercent, double timeout)
{
    const double diameter = 2.75;
    const double pi = 3.14159;
    const double outputGear = 48;
    const double inputGear = 36;
    const double inchesPerRev = (diameter * pi) * (inputGear / outputGear);

    maxSpeedPercent = std::clamp(maxSpeedPercent, 0.0, 100.0);
    const double maxVelRpm = profileMaxVelRpm * (maxSpeedPercent / 100.0);

    const double direction = (inches >= 0) ? 1.0 : -1.0;
    const double totalRev = std::fabs(inches) / inchesPerRev;
    const double tolRev = profileToleranceIn / inchesPerRev;

    pros::Motor* motors[4] = {&mfl, &mbl, &mfr, &mbr};
    double startRot[4];
    for (int i = 0; i < 4; i++) startRot[i] = motorRotations(*motors[i]);

    const uint32_t pollMs = 10;
    uint32_t startTime = pros::millis();
    uint32_t now = startTime;
    int settled = 0;

    while (pros::millis() - startTime < timeout)
    {
        double sum = 0, velSum = 0;
        for (int i = 0; i < 4; i++)
        {
            sum += motorRotations(*motors[i]) - startRot[i];
            velSum += motors[i]->get_actual_velocity();
        }
        double progress = direction * (sum / 4.0); // rotations covered toward the target
        double avgVel = std::fabs(velSum / 4.0);
        double remaining = totalRev - progress;    // negative = overshot

        double cmdRpm;
        if (remaining > tolRev)
        {
            // profile runs all the way to the tolerance band - its decel curve already
            // bottoms out at the creep speed, so there is no separate slow final stretch
            cmdRpm = profileVelocityRpm(progress, remaining, maxVelRpm);
        }
        else
        {
            // inside tolerance (or past it): command 0 and let it stop. no reverse correction -
            // backing up at creep speed and braking overshot the other side and made it hunt
            cmdRpm = 0.0;
        }
        cmdRpm *= direction;

        left_motors.move_velocity(cmdRpm);
        right_motors.move_velocity(cmdRpm);

        if (remaining <= tolRev && avgVel < 20.0)
        {
            if (++settled >= 3) break;
        }
        else
        {
            settled = 0;
        }

        pros::Task::delay_until(&now, pollMs);
    }

    left_motors.brake();
    right_motors.brake();
}
