#include "devices.h"
#include "vexAutonSelector.h"
#include "vexAuton.h"
#include <string>
using namespace pros;
using namespace lemlib;

const int numAutons = 6; // must divide 360 evenly (selector splits the dial into equal segments)

// ---------- auton selector (rotation sensor) ----------
void displayVEXSelection()
{
    int selection = getAutonSelection(numAutons);

    std::string autonName;
    switch (selection)
    {
    case 0:
        autonName = "Skills";
        break;
    case 1:
        autonName = "Alliance Close Alleys";
        break;
    case 2:
        autonName = "Alliance Close 3 Pin";
        break;
    case 3:
        autonName = "Datalogging Route";
        break;
    case 4:
        autonName = "Accel/Decel Test";
        break;
    case 5:
        autonName = "PID Tuning";
        break;
    default:
        autonName = "Unknown";
    }

    pros::lcd::set_text(1, "Selected: " + autonName);
}

// ---------- run the VEX auton picked on the selector ----------
void runVexAuton()
{
    switch (getAutonSelection(numAutons))
    {
    case 0:
        vexSkills();
        break;
    case 1:
        allianceCloseAlleys();
        break;
    case 2:
        allianceClose3Pin();
        break;
    case 3:
        dataloggingRoute();
        break;
    case 4:
        accelDecelTest();
        break;
    case 5:
        PIDtuning();
        break;
    default:
        break;
    }
}
