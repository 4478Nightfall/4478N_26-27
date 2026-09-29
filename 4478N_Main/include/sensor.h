#pragma once

// sd datalog csv stuff
void logData();
void startDatalogging();
void stopDatalogging();

// fast drive-motor logger for measuring accel/decel: 10ms samples buffered in ram,
// written to one csv when stopped. marker is written into every row so the
// csv shows exactly when the test changed state (0 = rest, 1 = drive cmd, 2 = brake)
extern volatile int datalogMarker;
void startFastDatalogging();
void stopFastDatalogging();
