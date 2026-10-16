 #include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "autonSelector.h"
#include "moveFunctions.h"
#include "PIDtuner.h"
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
    claw.set_value(true);
    pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors

    // Initialize color sensor
    pros::delay(100);           // Allow sensor to stabilize

    casLTop.tare_position(); // cascade starts fully down = 0
    casLBottom.tare_position();
    casRTop.tare_position();
    casRBottom.tare_position();

    // Add a small delay to ensure solenoid has time to respond
    pros::delay(100);

    // Start handleMidGoal background task - runs in both autonomous and driver control
    // startHandleMidGoalTask();

    pros::Task screen_task([&]() {
        while (true) {
            if (pidTuningActive) {
                pros::delay(100);
                continue;
            }
            pros::lcd::print(0, "cas L %.0f R %.0f", casLTop.get_position(), casRTop.get_position());
            // const Pose lem = chassis.getPose();
            // pros::lcd::print(0, "LEM %5.1f %5.1f h%4.0f", lem.x, lem.y, lem.theta);

            // auto distLine = [&](int line, const char* tag, Distance* dev) {
            //     if (dev == nullptr) {
            //         pros::lcd::print(line, "%s --- no dev", tag);
            //         return;
            //     }
            //     const std::int32_t mm = dev->get();
            //     const int sz = dev->get_object_size();
            //     if (mm >= 0) {
            //         pros::lcd::print(line, "%s %5.1fin s%3d", tag, static_cast<double>(mm) / 25.4, sz);
            //     } else {
            //         pros::lcd::print(line, "%s ---- s%3d", tag, sz);
            //     }
            // };
            // distLine(2, "F", frontDistance);
            // distLine(3, "B", backDistancePtr);
            // distLine(4, "L", leftDistance);
            // distLine(5, "R", rightDistance);

            // pros::lcd::print(6, "dist rst: opctl B/Dpad/Y");
            // pros::lcd::print(7, "raw in, obj sz");
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
    
    // int selection = getAutonSelection(); // Get selected auton routine
    left_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
    claw.set_value(true);
    skills();
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
    // Add a small delay to ensure solenoid has time to respond
    pros::delay(100);

    // Main driver control loop
    bool clawOpen = true;
    uint32_t clawReadyAt = 0; // millis() time when auto-close is allowed again
    const double clawLiftDeg = 500; // cascade lift (motor degrees) after the claw closes
    bool clawLifting = false;
    bool cascadeReturning = false; // R1: drive the cascade back down to 0
    double clawLiftTarget = 0;
    claw.set_value(false);
    while (true)
    {
        casLTop.set_brake_mode(MOTOR_BRAKE_HOLD);
        casLBottom.set_brake_mode(MOTOR_BRAKE_HOLD);
        casRTop.set_brake_mode(MOTOR_BRAKE_HOLD);
        casRBottom.set_brake_mode(MOTOR_BRAKE_HOLD);
        right_motors.set_brake_mode(MOTOR_BRAKE_COAST); // Coast for smoother drive
        left_motors.set_brake_mode(MOTOR_BRAKE_COAST);

        // Hold X to enter the PID tuner (blocks driving until B is pressed to exit)


        // Get joystick values for tank drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // Apply exponential curve to joystick values for smoother control
        leftY = leftY * abs(leftY) / 100;
        rightY = rightY * abs(rightY) / 100;

        // Move the robot using tank drive
        chassis.tank(leftY, rightY);

        // Cascade: L1 up, L2 down (stops at the 0 floor)
        const bool cascadeManual = controller.get_digital(E_CONTROLLER_DIGITAL_L1) ||
                                   controller.get_digital(E_CONTROLLER_DIGITAL_L2);
        if (cascadeManual)
        {
            clawLifting = false; // driver input overrides the automatic lift
            cascadeReturning = false;
        }

        if (cascadeReturning)
        {
            // move_absolute is running; let it finish unless the driver takes over
            if (fabs(casLTop.get_position()) < 5) {
                cascadeReturning = false;
            }
        }
        else if (clawLifting)
        {
            // drive up like L1 does until we reach the target
            if (casLTop.get_position() >= clawLiftTarget) {
                clawLifting = false;
                casLTop.brake();
                casLBottom.brake();
                casRTop.brake();
                casRBottom.brake();
            } else {
                casLTop.move(127);
                casLBottom.move(127);
                casRTop.move(127);
                casRBottom.move(127);
            }
        }
        else if (controller.get_digital(E_CONTROLLER_DIGITAL_L1))
        {
            if (casLTop.get_position() >= casUpVal) {
                // top limit reached - stop everything, can't go farther up
                casLTop.brake();
                casLBottom.brake();
                casRTop.brake();
                casRBottom.brake();
            } else {
                casLTop.move(127);
                casLBottom.move(127);
                casRTop.move(127);
                casRBottom.move(127);
            }
        }
        else if (controller.get_digital(E_CONTROLLER_DIGITAL_L2))
        {
            if (casLTop.get_position() <= casDownVal || casRTop.get_position() <= casDownVal) {
                casLTop.brake();
                casLBottom.brake();
                casRTop.brake();
                casRBottom.brake();
            } else {
                casLTop.move(-127);
                casLBottom.move(-127);
                casRTop.move(-127);
                casRBottom.move(-127);
            }
        }
        else
        {
            casLTop.brake();
            casLBottom.brake();
            casRTop.brake();
            casRBottom.brake();
        }

        // claw: A toggles, R1 opens, and it closes by itself when something is close
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
            const bool wasOpen = clawOpen;
            clawOpen = !clawOpen;
            claw.set_value(!clawOpen); // false = open, true = closed
            delay(200);
            if (wasOpen) {
                // closing on a game piece: lift the cascade ~60 degrees
                clawLiftTarget = fmin(casLTop.get_position() + clawLiftDeg, casUpVal);
                clawLifting = true;
                cascadeReturning = false;
            }
        }
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){
            // send the cascade back to 0 degrees
            clawLifting = false;
            cascadeReturning = true;
            casLTop.move_absolute(0, 200);
            casLBottom.move_absolute(0, 200);
            casRTop.move_absolute(0, 200);
            casRBottom.move_absolute(0, 200);
        }
        //     if (clawOpen) clawReadyAt = pros::millis() + 1600; // no auto-close for 1 s after opening
        // }
        // if ((controller.get_digital(E_CONTROLLER_DIGITAL_R2) && clawOpen == false)) {
        //     clawOpen = true;
        //     claw.set_value(false);
        //     clawReadyAt = pros::millis() + 1600; // no auto-close for 1 s after opening
        // }
        // int clawDist = frontDistanceSensor.get();
        // if (clawDist > 0 && clawDist <= 67 && clawOpen == true && pros::millis() >= clawReadyAt) {
        //     clawOpen = false;
        //     claw.set_value(true);
    // }


    

        // delay to save resources
        pros::delay(25);
    }
}