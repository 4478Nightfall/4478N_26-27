#include "devices.h"
#include "recfAutonSelector.h"
#include "recfAuton.h"
#include "vexAuton.h"
#include <string>
using namespace pros;
using namespace lemlib;

const int numAutons = 15;

// ---------- auton selector (rotation sensor) ----------
void displayRECFSelection()
{
    int selection = getAutonSelection(numAutons);
    pros::lcd::set_text(4, "Autonomous Selector");

    std::string autonName;
    switch (selection)
    {
    case 0:
        autonName = "Skills";
        break;
    case 1:
        autonName = "Left 3_4";
        break;
    case 2:
        autonName = "Left Toggle";
        break;
    case 3:
        autonName = "Right 3_4";
        break;
    case 4:
        autonName = "Right toggle";
        break;
    case 5:
        autonName = "Elims Right Cup";
        break;
    case 6:
        autonName = "Elims Left Cup";
        break;
    case 7: 
        autonName = "PID tuning";
        break;
    default:
        autonName = "Unknown";
    }

    // Show the selected routine name and the raw rotation sensor value
    pros::lcd::set_text(5, "Selected: " + autonName);
}

// ---------- run the RECF auton picked on the selector ----------
void runRecfAuton()
{
    switch (getAutonSelection(numAutons))
    {
    case 0:
        skillsNew();
        break;
    case 1:
        left3_4Alliance();
        break;
    case 2:
        leftToggle();
        break;
    case 3:
        right3_4();
        break;
    case 4:
        rightToggle();
        break;
    case 5:
        elimsRight();
        break;
    case 6:
        elimsLeftCup();
        break;
    case 7:
        PIDtuning();
        break;
    default:
        PIDtuning();
        break;
    }
}
