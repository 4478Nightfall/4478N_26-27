#include "devices.h"
#include "autonSelector.h"
#include <string>
using namespace pros;
using namespace lemlib;

const int numAutons = 15;

// ---------- auton selector (rotation sensor) ----------
int getAutonSelection()
{
    double rotation = autonSelector.get_position() / 100.0;

    while (rotation < 0)
        rotation += 360;
    rotation = fmod(rotation, 360);

    int segmentSize = 360 / numAutons;
    return (int)(rotation / segmentSize);
}

void displaySelection()
{
    int selection = getAutonSelection();
    pros::lcd::set_text(1, "Autonomous Selector");

    std::string autonName;
    switch (selection)
    {
    case 0:
        autonName = "Skills";
        break;
    case 1:
        autonName = "Left 2_3";
        break;
    case 2:
        autonName = "Left Toggle";
        break;
    case 3:
        autonName = "Right 2_3";
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
    default:
        autonName = "Unknown";
    }

    // Show the selected routine name and the raw rotation sensor value
    pros::lcd::set_text(1, "Selected: " + autonName);
}