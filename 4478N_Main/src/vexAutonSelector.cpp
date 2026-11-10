#include "devices.h"
#include "vexAutonSelector.h"
#include "vexAuton.h"
#include <string>
using namespace pros;
using namespace lemlib;

const int numAutons = 12;

// ---------- auton selector (rotation sensor) ----------
void displayVEXSelection()
{
    int selection = getAutonSelection(numAutons);
    pros::lcd::set_text(4, "VEX Autonomous Selector");

    std::string autonName;
    switch (selection)
    {
    case 0:
        autonName = "Skills Hard";
        break;
    case 1:
        autonName = "Skills Easy";
        break;
    case 2:
        autonName = "Alliance Left";
        break;
    case 3:
        autonName = "Alliance Close";
        break;
    case 4:
        autonName = "Neutral Left";
        break;
    case 5:
        autonName = "Neutral Right";
        break;
    case 6:
        autonName = "Neutral Left Simple";
        break;
    case 7:
        autonName = "Neutral Right Simple";
        break;
    case 8:
        autonName = "Helper Solo";
        break;
    case 9:
        autonName = "Close";
        break;
    case 10:
        autonName = "Far";
        break;
    case 11:
        autonName = "PID tuning";
        break;
    default:
        autonName = "Unknown";
    }

    // Show the selected routine name
    pros::lcd::set_text(5, "Selected: " + autonName);
}

// ---------- run the VEX auton picked on the selector ----------
void runVexAuton()
{
    switch (getAutonSelection(numAutons))
    {
    case 0:
        skillsHard();
        break;
    case 1:
        skillsEasy();
        break;
    case 2:
        allianceLeft();
        break;
    case 3:
        allianceClose();
        break;
    case 4:
        neutralLeft();
        break;
    case 5:
        neutralRight();
        break;
    case 6:
        neutralLeftSimple();
        break;
    case 7:
        neutralRightSimple();
        break;
    case 8:
        helperSolo();
        break;
    case 9:
        close();
        break;
    case 10:
        far();
        break;
    default:
        PIDtuning();
        break;
    }
}
