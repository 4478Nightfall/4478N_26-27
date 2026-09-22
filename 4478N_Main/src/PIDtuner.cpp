#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "autonSelector.h"
#include "moveFunctions.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include <cmath>
using namespace pros;
using namespace lemlib;

// ---------- pid tuning / test ----------

double kP = 0.22;
double kI = 0.000000;
double kD = 0.1;

void drivePIDTune(double kP, double kI, double kD, double fwdVal, double maxSpeedPercent, double timeout)
{
    
    const double diameter = 3.25;
    const double pi = 3.14159;
    const double outputGear = 48;
    const double inputGear = 36;

    double num = fwdVal;                                       // leave this one alone
    double denom = (diameter * pi) * (inputGear / outputGear); // leave this one alone
    double target = (num / denom) * 360;                       // leave this one alone
    
    // Reset motor positions to zero
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
    
    // Dynamic goal tracking (proportional to distance)
    const double pollingRate = 20; // ms - consistent with sister team
    
    // Exit conditions: position error only (degrees)
    const double errorThreshold = 5.0; // degrees - position threshold
    int inGoal = 0;
    const int goalsNeeded = 3; // Require 3 consecutive iterations in goal for proper settling
    
    // Clamp and convert maxSpeedPercent (0-100) -> max motor units (0-127)
    if (maxSpeedPercent < 0)
        maxSpeedPercent = 0;
    if (maxSpeedPercent > 100)
        maxSpeedPercent = 100;
    double maxMotor = (maxSpeedPercent / 100.0) * 127.0;

    bool hasMoved = false; // Track if robot has started moving (prevent immediate exit)
    
    while (inGoal < goalsNeeded)
    {
        // Average left side motors and right side motors separately, then average the two sides
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

        // Gain scheduling: adjust PID gains based on error magnitude (in inches) for consistent behavior
        // Larger errors need more aggressive response, smaller errors need finer control
        // Convert error from degrees to inches: error_inches = error_degrees * (denom / 360)
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

        // Fast exit condition: position error is small AND robot has moved
        // This allows immediate exit when robot has reached the target
        if (hasMoved && fabs(error) < errorThreshold)
        {
            inGoal++;
            // Exit immediately once condition is met (no need to wait for multiple iterations)
            if (inGoal >= goalsNeeded) {
                break;
            }
        }
        else
        {
            inGoal = 0; // Reset if conditions not met
        }

        // Timeout safety - break immediately if timeout reached
        if ((pros::millis() - startTime) >= timeout)
        {
            break;
        }

        prevError = error;
        
        prevError = error;
        
        // Consistent polling rate
        pros::delay(pollingRate);
    }
    
    // Stop motors once goal is reached
    left_motors.brake();
    right_motors.brake();
}

void tuningLoop(){
    // turnToHeadingSmart(30,2000);
        drivePIDTune(kP,kI,kD,48,100,3000);
        turnToHeadingSmart(90,800);

}

volatile bool pidTuningActive = false;

void pidTuningMode() {
    // Stop the background screen task from overwriting the LCD while tuning
    pidTuningActive = true;
    pros::delay(150);
    // lcd::clear() deletes the LLEMU label objects and crashes the next print, so clear line by line
    for (int line = 0; line < 8; line++) pros::lcd::clear_line(line);

    // Reset all PID values to zero when entering tuning mode
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    
    int selectedParam = 0;
    
    const double kPIncrement = 0.01;
    const double kIIncrement = 0.00001;
    const double kDIncrement = 0.01;
    bool lastUpState = false;
    bool lastDownState = false;
    bool lastLeftState = false;
    bool lastRightState = false;
    bool lastR2State = false;
    
    int displayCounter = 0;
    
    while (true) {
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
            break;
        }

        // Get current button states
        bool upPressed = controller.get_digital(E_CONTROLLER_DIGITAL_UP);
        bool downPressed = controller.get_digital(E_CONTROLLER_DIGITAL_DOWN);
        bool leftPressed = controller.get_digital(E_CONTROLLER_DIGITAL_LEFT);
        bool rightPressed = controller.get_digital(E_CONTROLLER_DIGITAL_RIGHT);
        bool r2Pressed = controller.get_digital(E_CONTROLLER_DIGITAL_R2);
        
        // Navigate between parameters (UP/DOWN)
        if (upPressed && !lastUpState) {
            // Move up in parameter list (KP -> KD -> KI -> KP)
            selectedParam = (selectedParam - 1 + 3) % 3;
        }
        if (downPressed && !lastDownState) {
            // Move down in parameter list (KP -> KI -> KD -> KP)
            selectedParam = (selectedParam + 1) % 3;
        }
        
        // Change selected parameter value (LEFT decreases, RIGHT increases)
        if (leftPressed && !lastLeftState) {
            // Decrease selected parameter
            switch (selectedParam) {
                case 0: // KP
                    kP -= kPIncrement;
                    if (kP < 0) kP = 0;
                    break;
                case 1: // KI
                    kI -= kIIncrement;
                    if (kI < 0) kI = 0;
                    break;
                case 2: // KD
                    kD -= kDIncrement;
                    if (kD < 0) kD = 0;
                    break;
            }
        }
        if (rightPressed && !lastRightState) {
            // Increase selected parameter
            switch (selectedParam) {
                case 0: // KP
                    kP += kPIncrement;
                    break;
                case 1: // KI
                    kI += kIIncrement;
                    break;
                case 2: // KD
                    kD += kDIncrement;
                    break;
            }
        }
        
        // Update last button states
        lastUpState = upPressed;
        lastDownState = downPressed;
        lastLeftState = leftPressed;
        lastRightState = rightPressed;
        
        if (displayCounter % 10 == 0) {
            // Display KP on line 1 (index 0)
            if (selectedParam == 0) {
                pros::lcd::print(0, "-> KP: %.4f", kP);
            } else {
                pros::lcd::print(0, "   KP: %.4f", kP);
            }
            
            // Display KI on line 2 (index 1)
            if (selectedParam == 1) {
                pros::lcd::print(1, "-> KI: %.6f", kI);
            } else {
                pros::lcd::print(1, "   KI: %.6f", kI);
            }
            
            // Display KD on line 3 (index 2)
            if (selectedParam == 2) {
                pros::lcd::print(2, "-> KD: %.4f", kD);
            } else {
                pros::lcd::print(2, "   KD: %.4f", kD);
            }
        }
        displayCounter++;
        
        if (r2Pressed && !lastR2State) {
            // Test sequence: 48 in forward, -24 in back, 12 in forward
            drivePIDTune(kP, kI, kD, 48.0, 100, 3000);
            pros::delay(500);
            drivePIDTune(kP, kI, kD, -24.0, 100, 3000);
            pros::delay(500);
            drivePIDTune(kP, kI, kD, 12.0, 100, 3000);
            drivePIDTune(kP, kI, kD, -36.0, 100, 3000);

        }
        
        lastR2State = r2Pressed;

        pros::delay(20);
    }

    for (int line = 0; line < 8; line++) pros::lcd::clear_line(line);
    pidTuningActive = false;
}