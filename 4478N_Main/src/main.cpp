 #include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "autonSelector.h"
#include "moveFunctions.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include <cmath>  // For fabs()
#include <cstdint>
using namespace pros;
using namespace lemlib;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors

    // Initialize color sensor
    pros::delay(100);           // Allow sensor to stabilize

    intPos.set_value(LOW); // Set back gate to default position (closed/down)
 

    // Add a small delay to ensure solenoid has time to respond
    pros::delay(100);

    // Start handleMidGoal background task - runs in both autonomous and driver control
    // startHandleMidGoalTask();

    pros::Task screen_task([&]() {
        while (true) {
            const Pose lem = chassis.getPose();
            pros::lcd::print(0, "LEM %5.1f %5.1f h%4.0f", lem.x, lem.y, lem.theta);

            auto distLine = [&](int line, const char* tag, Distance* dev) {
                if (dev == nullptr) {
                    pros::lcd::print(line, "%s --- no dev", tag);
                    return;
                }
                const std::int32_t mm = dev->get();
                const int sz = dev->get_object_size();
                if (mm >= 0) {
                    pros::lcd::print(line, "%s %5.1fin s%3d", tag, static_cast<double>(mm) / 25.4, sz);
                } else {
                    pros::lcd::print(line, "%s ---- s%3d", tag, sz);
                }
            };
            distLine(2, "F", frontDistance);
            distLine(3, "B", backDistancePtr);
            distLine(4, "L", leftDistance);
            distLine(5, "R", rightDistance);

            pros::lcd::print(6, "dist rst: opctl B/Dpad/Y");
            pros::lcd::print(7, "raw in, obj sz");
            pros::delay(100);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
    // pros::lcd::register_btn1_cb(autonSelector); // Example for registering a callback
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
    
    int selection = getAutonSelection(); // Get selected auton routine
    left_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    intPos.set_value(LOW);


    // Run the selected autonomous routine
    // switch (selection)
    // {
    // case 0:
    //     // Skills
    // workingSkills();
    //     break;
    // case 1:
    //     // PID Testing
    //     PIDTesting();
    //     break;
    // case 2:
    //     // Solo AWP
    //     soloAWPTap();
    //     break;
    // case 3:
    //     // Left 3 Long Blast (Last)
    //     fakeLeft();
    //     break;
    // case 4:
    //     // Left 3 Long First
    //     Left4_3LongFirst();
    //     break;
    // case 5:
    //     // Right 3 Long Last
    //     fakeRight();
    //     break;
    // case 6:
    //     // Right 3 Long First
    //     Right4_3LongFirst();
    //     break;
    // case 7:
    //     // Left 7 Ball Hold
    //     left7BallHold();
    //     break;
    // case 8:
    //     // Left 7 Ball Wing
    //     left7BallFast();
    //     break;
    // case 9:
    //     // Right 7 Ball Hold
    //     right7BallHold();
    //     break;
    // case 10:
    //     // Right 7 Ball Wing
    //     right7BallWing();
    //     break;
    // case 11:
    //     // Left 4 Wing
    //     left4Wing();
    //     break;
    // case 12:
    //     // Right 4 Wing
    //     right4Wing();
    //     break;
    // default:
    //     // Default routine or do nothing
    //     workingSkills();
    //     break;
    // }

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
    intPos.set_value(LOW); // Start with front gate closed (down)
    // Add a small delay to ensure solenoid has time to respond
    pros::delay(100);

    // Start color sorting task once

    // Outtake speed toggle variables (outside loop to persist state)
    bool wasAPressed = false;
    bool rolMode; //change between settings for roller position

    // Main driver control loop
    while (true)
    {
                 
        // Display color sensor debugging info

        casL.set_brake_mode(MOTOR_BRAKE_HOLD);
        casR.set_brake_mode(MOTOR_BRAKE_HOLD);
        intake.set_brake_mode(MOTOR_BRAKE_HOLD);
        right_motors.set_brake_mode(MOTOR_BRAKE_COAST); // Coast for smoother drive
        left_motors.set_brake_mode(MOTOR_BRAKE_COAST);

        // Get joystick values for tank drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // Apply exponential curve to joystick values for smoother control
        leftY = leftY * abs(leftY) / 100;
        rightY = rightY * abs(rightY) / 100;

        // Move the robot using tank drive
        chassis.tank(leftY, rightY);

        // Intake: Y (middle-goal spin) takes priority over R2/R1.
        
        if (controller.get_digital(E_CONTROLLER_DIGITAL_L2))
        {
            intake.move(-127); // Intake in
            roller.move(127);
        }
        else if (controller.get_digital(E_CONTROLLER_DIGITAL_L1))
        {
            intake.move(127); // Intake out
        }
        else
        {
            intake.set_brake_mode(MOTOR_BRAKE_HOLD);
            intake.brake(); // Stop intake when neither button is pressed
        }

        if (controller.get_digital(E_CONTROLLER_DIGITAL_B)){
            roller.move(127);
        }
        else if(controller.get_digital(E_CONTROLLER_DIGITAL_Y)){
            roller.move(-127);
        }
        else if (!controller.get_digital(E_CONTROLLER_DIGITAL_L2)){
            roller.brake(); // Stop roller when no button driving it is pressed
        }
        if (controller.get_digital(E_CONTROLLER_DIGITAL_R1))
        {
            casL.move(127); // Spin left cas out
            rolMode = true; // change between up and mid values when going up
        }
        else if (controller.get_digital(E_CONTROLLER_DIGITAL_R2))
        {
            casL.move(-127); // Spin left cas in
            rolMode = false; //change bt down and mid val when going down
        }
        else if (controller.get_digital(E_CONTROLLER_DIGITAL_L2))
        {
            // Auto-retract cas to the bottom while intaking (R1/R2 above take priority over this)
            if (casL.get_position() > casDownVal) {
                casL.move(-50);
            } else {
                casL.brake();
            }
        }
        else
        {
            casL.set_brake_mode(MOTOR_BRAKE_HOLD);
            casL.brake(); // Stop left cas when neither button is pressed
        }

        if (controller.get_digital(E_CONTROLLER_DIGITAL_UP))
        {
            casR.move(127); // Spin right cas out
            rolMode = true; // change between up and mid values when going up
        }
        else if (controller.get_digital(E_CONTROLLER_DIGITAL_DOWN))
        {
            casR.move(-127); // Spin right cas in
            rolMode = false; //change bt down and mid val when going down
        }
        else if (controller.get_digital(E_CONTROLLER_DIGITAL_L2))
        {
            // Auto-retract cas to the bottom while intaking (R1/R2 above take priority over this)
            if (casR.get_position() > casDownVal) {
                casR.move(-50);
            } else {
                casR.brake();
            }
        }
        else
        {
            casR.set_brake_mode(MOTOR_BRAKE_HOLD);
            casR.brake(); // Stop right cas when neither button is pressed
        }

    //    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
    //     intPos.set_value(!intPos.get_value());
    //    }
       
       if (rolMode == false){
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
            if(high == true)
            {
                goDownAsync();
            }
            else if (mid == true){
                goDownAsync();
            }
            else if (low == true){
                goMidAsync();
            }
        }
       }

       if (rolMode == true){
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
            if(high == true)
            {
                goMidAsync();
            }
            else if (mid == true){
                goHighAsync();
            }
            else if (low == true){
                goMidAsync();
            }
        }
       }
       
        // delay to save resources
        pros::delay(25);
    }
}