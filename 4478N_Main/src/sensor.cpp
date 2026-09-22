#include "sensor.h"
#include "devices.h"
#include <cstdio>
#include <numeric>
#include <algorithm>
#include <cstring>

using namespace pros;

// ---------- sd datalog ----------
volatile bool dataLoggingRunning = false;
pros::Task* dataLoggingTask = nullptr;
static char datalogFilename[64] = "";

const char* getDatalogFilename()
{
    if (strlen(datalogFilename) > 0)
    {
        return datalogFilename;
    }
    
    // Check if base datalog.csv exists
    FILE *testFile = std::fopen("/usd/datalog.csv", "r");
    bool baseExists = (testFile != nullptr);
    if (testFile)
    {
        std::fclose(testFile);
    }
    
    // If base file doesn't exist, use it
    if (!baseExists)
    {
        std::strncpy(datalogFilename, "/usd/datalog.csv", sizeof(datalogFilename) - 1);
        datalogFilename[sizeof(datalogFilename) - 1] = '\0';
        return datalogFilename;
    }
    
    // Base file exists, find next available numbered file
    int fileNum = 1;
    char testFilename[64];
    do
    {
        std::snprintf(testFilename, sizeof(testFilename), "/usd/datalog_%d.csv", fileNum);
        testFile = std::fopen(testFilename, "r");
        if (testFile)
        {
            // File exists, try next number
            std::fclose(testFile);
            fileNum++;
        }
        else
        {
            // Found available filename
            std::strncpy(datalogFilename, testFilename, sizeof(datalogFilename) - 1);
            datalogFilename[sizeof(datalogFilename) - 1] = '\0';
            return datalogFilename;
        }
    } while (fileNum < 1000); // Safety limit
    
    // Fallback (should never reach here)
    std::strncpy(datalogFilename, "/usd/datalog.csv", sizeof(datalogFilename) - 1);
    datalogFilename[sizeof(datalogFilename) - 1] = '\0';
    return datalogFilename;
}

void logData()
{
    const char* filename = getDatalogFilename();
    FILE *file = std::fopen(filename, "a");
    if (!file)
    {
        pros::lcd::set_text(0, "SD open fail");
        return;
    }
    static bool headerWritten = false;
    static char lastFilename[64] = "";

    if (std::strlen(lastFilename) == 0 || std::strcmp(filename, lastFilename) != 0)
    {
        headerWritten = false;
        std::strncpy(lastFilename, filename, sizeof(lastFilename) - 1);
        lastFilename[sizeof(lastFilename) - 1] = '\0';
    }
    
    if (!headerWritten)
    {
        std::fseek(file, 0, SEEK_END);
        if (std::ftell(file) == 0)
        {
            std::fprintf(file, "timestamp,left_temp,left_vel,left_deg,left_voltage,left_current,left_power,left_torque,left_efficiency,right_temp,right_vel,right_deg,right_voltage,right_current,right_power,right_torque,right_efficiency,rotation_deg,autonSelector_deg,hTracker_deg,imu_heading,"
                "mfl_temp,mfl_vel,mfl_deg,mfl_voltage,mfl_current,mfl_power,mfl_torque,mfl_efficiency,mfl_overtemp,mfl_overcurrent,"
                "mbl_temp,mbl_vel,mbl_deg,mbl_voltage,mbl_current,mbl_power,mbl_torque,mbl_efficiency,mbl_overtemp,mbl_overcurrent,"
                "mfr_temp,mfr_vel,mfr_deg,mfr_voltage,mfr_current,mfr_power,mfr_torque,mfr_efficiency,mfr_overtemp,mfr_overcurrent,"
                "mbr_temp,mbr_vel,mbr_deg,mbr_voltage,mbr_current,mbr_power,mbr_torque,mbr_efficiency,mbr_overtemp,mbr_overcurrent,"
                "imu_rotation,imu_pitch,imu_roll\n");
        }
        headerWritten = true;
    }
    uint32_t timestamp = pros::millis();

    auto leftTemps = left_motors.get_temperature_all();
    auto leftVels = left_motors.get_actual_velocity_all();
    auto leftDegs = left_motors.get_position_all();
    auto leftVoltages = left_motors.get_voltage_all();
    auto leftCurrents = left_motors.get_current_draw_all();
    auto leftPowers = left_motors.get_power_all();
    auto leftTorques = left_motors.get_torque_all();
    auto leftEfficiencies = left_motors.get_efficiency_all();
    double leftTemp = 0, leftVel = 0, leftDeg = 0, leftVoltage = 0, leftCurrent = 0, leftPower = 0, leftTorque = 0, leftEfficiency = 0;
    if (!leftTemps.empty())
        leftTemp = std::accumulate(leftTemps.begin(), leftTemps.end(), 0.0) / leftTemps.size();
    if (!leftVels.empty())
        leftVel = std::accumulate(leftVels.begin(), leftVels.end(), 0.0) / leftVels.size();
    if (!leftDegs.empty())
        leftDeg = std::accumulate(leftDegs.begin(), leftDegs.end(), 0.0) / leftDegs.size();
    if (!leftVoltages.empty())
        leftVoltage = std::accumulate(leftVoltages.begin(), leftVoltages.end(), 0.0) / leftVoltages.size() / 1000.0;
    if (!leftCurrents.empty())
        leftCurrent = std::accumulate(leftCurrents.begin(), leftCurrents.end(), 0.0) / leftCurrents.size() / 1000.0;
    if (!leftPowers.empty())
        leftPower = std::accumulate(leftPowers.begin(), leftPowers.end(), 0.0) / leftPowers.size();
    if (!leftTorques.empty())
        leftTorque = std::accumulate(leftTorques.begin(), leftTorques.end(), 0.0) / leftTorques.size();
    if (!leftEfficiencies.empty())
        leftEfficiency = std::accumulate(leftEfficiencies.begin(), leftEfficiencies.end(), 0.0) / leftEfficiencies.size();

    auto rightTemps = right_motors.get_temperature_all();
    auto rightVels = right_motors.get_actual_velocity_all();
    auto rightDegs = right_motors.get_position_all();
    auto rightVoltages = right_motors.get_voltage_all();
    auto rightCurrents = right_motors.get_current_draw_all();
    auto rightPowers = right_motors.get_power_all();
    auto rightTorques = right_motors.get_torque_all();
    auto rightEfficiencies = right_motors.get_efficiency_all();
    double rightTemp = 0, rightVel = 0, rightDeg = 0, rightVoltage = 0, rightCurrent = 0, rightPower = 0, rightTorque = 0, rightEfficiency = 0;
    if (!rightTemps.empty())
        rightTemp = std::accumulate(rightTemps.begin(), rightTemps.end(), 0.0) / rightTemps.size();
    if (!rightVels.empty())
        rightVel = std::accumulate(rightVels.begin(), rightVels.end(), 0.0) / rightVels.size();
    if (!rightDegs.empty())
        rightDeg = std::accumulate(rightDegs.begin(), rightDegs.end(), 0.0) / rightDegs.size();
    if (!rightVoltages.empty())
        rightVoltage = std::accumulate(rightVoltages.begin(), rightVoltages.end(), 0.0) / rightVoltages.size() / 1000.0;
    if (!rightCurrents.empty())
        rightCurrent = std::accumulate(rightCurrents.begin(), rightCurrents.end(), 0.0) / rightCurrents.size() / 1000.0;
    if (!rightPowers.empty())
        rightPower = std::accumulate(rightPowers.begin(), rightPowers.end(), 0.0) / rightPowers.size();
    if (!rightTorques.empty())
        rightTorque = std::accumulate(rightTorques.begin(), rightTorques.end(), 0.0) / rightTorques.size();
    if (!rightEfficiencies.empty())
        rightEfficiency = std::accumulate(rightEfficiencies.begin(), rightEfficiencies.end(), 0.0) / rightEfficiencies.size();

    // average of 4 wheel encoders
    double positions[4] = {
        mbl.get_position(), mfl.get_position(),
        mbr.get_position(), mfr.get_position()
    };
    double rotationDeg = std::accumulate(positions, positions + 4, 0.0) / 4.0;

    double autonSelectorDeg = autonSelector.get_position() / 100.0;
    double hTrackerDeg = hTracker.get_position() / 100.0;
    double imuHeading = imu2.get_heading();

    double mflTemp = mfl.get_temperature();
    double mflVel = mfl.get_actual_velocity();
    double mflDeg = mfl.get_position();
    double mflVoltage = mfl.get_voltage() / 1000.0;
    double mflCurrent = mfl.get_current_draw() / 1000.0;
    double mflPower = mfl.get_power();
    double mflTorque = mfl.get_torque();
    double mflEfficiency = mfl.get_efficiency();
    int mflOverTemp = mfl.is_over_temp();
    int mflOverCurrent = mfl.is_over_current();

    double mblTemp = mbl.get_temperature();
    double mblVel = mbl.get_actual_velocity();
    double mblDeg = mbl.get_position();
    double mblVoltage = mbl.get_voltage() / 1000.0;
    double mblCurrent = mbl.get_current_draw() / 1000.0;
    double mblPower = mbl.get_power();
    double mblTorque = mbl.get_torque();
    double mblEfficiency = mbl.get_efficiency();
    int mblOverTemp = mbl.is_over_temp();
    int mblOverCurrent = mbl.is_over_current();

    double mfrTemp = mfr.get_temperature();
    double mfrVel = mfr.get_actual_velocity();
    double mfrDeg = mfr.get_position();
    double mfrVoltage = mfr.get_voltage() / 1000.0;
    double mfrCurrent = mfr.get_current_draw() / 1000.0;
    double mfrPower = mfr.get_power();
    double mfrTorque = mfr.get_torque();
    double mfrEfficiency = mfr.get_efficiency();
    int mfrOverTemp = mfr.is_over_temp();
    int mfrOverCurrent = mfr.is_over_current();

    double mbrTemp = mbr.get_temperature();
    double mbrVel = mbr.get_actual_velocity();
    double mbrDeg = mbr.get_position();
    double mbrVoltage = mbr.get_voltage() / 1000.0;
    double mbrCurrent = mbr.get_current_draw() / 1000.0;
    double mbrPower = mbr.get_power();
    double mbrTorque = mbr.get_torque();
    double mbrEfficiency = mbr.get_efficiency();
    int mbrOverTemp = mbr.is_over_temp();
    int mbrOverCurrent = mbr.is_over_current();

    double imuRotation = imu2.get_rotation();
    double imuPitch = imu2.get_pitch();
    double imuRoll = imu2.get_roll();

    std::fprintf(file, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,"
        "%f,%f,%f\n",
        timestamp,
        leftTemp,
        leftVel,
        leftDeg,
        leftVoltage,
        leftCurrent,
        leftPower,
        leftTorque,
        leftEfficiency,
        rightTemp,
        rightVel,
        rightDeg,
        rightVoltage,
        rightCurrent,
        rightPower,
        rightTorque,
        rightEfficiency,
        rotationDeg,
        autonSelectorDeg,
        hTrackerDeg,
        imuHeading,
        mflTemp,
        mflVel,
        mflDeg,
        mflVoltage,
        mflCurrent,
        mflPower,
        mflTorque,
        mflEfficiency,
        mflOverTemp,
        mflOverCurrent,
        mblTemp,
        mblVel,
        mblDeg,
        mblVoltage,
        mblCurrent,
        mblPower,
        mblTorque,
        mblEfficiency,
        mblOverTemp,
        mblOverCurrent,
        mfrTemp,
        mfrVel,
        mfrDeg,
        mfrVoltage,
        mfrCurrent,
        mfrPower,
        mfrTorque,
        mfrEfficiency,
        mfrOverTemp,
        mfrOverCurrent,
        mbrTemp,
        mbrVel,
        mbrDeg,
        mbrVoltage,
        mbrCurrent,
        mbrPower,
        mbrTorque,
        mbrEfficiency,
        mbrOverTemp,
        mbrOverCurrent,
        imuRotation,
        imuPitch,
        imuRoll);
    
    std::fclose(file);
}

void dataloggingLoopFn(void *)
{
    while (dataLoggingRunning)
    {
        logData();
        pros::delay(100);
    }
}
void startDatalogging()
{
    if (dataLoggingRunning)
        return;
    
    datalogFilename[0] = '\0';

    dataLoggingRunning = true;
    dataLoggingTask = new pros::Task(dataloggingLoopFn, nullptr, "Datalogging Task");
}
void stopDatalogging()
{
    dataLoggingRunning = false;
    if (dataLoggingTask)
    {
        dataLoggingTask->remove();
        delete dataLoggingTask;
        dataLoggingTask = nullptr;
    }
    datalogFilename[0] = '\0';
}

