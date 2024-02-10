// ElevatorPositionStorage.h

#ifndef ELEVATOR_POSITION_TRACKER_H
#define ELEVATOR_POSITION_TRACKER_H

#include <frc/Preferences.h>

class ElevatorPositionTracker {
public:
    static void SavePosition(double positionInches);

    static double LoadPosition();
};

#endif // ELEVATOR_POSITION_STORAGE_H
