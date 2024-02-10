#include "utils/ElevatorPositionTracker.h"

void ElevatorPositionTracker::SavePosition(double positionInches) {
    // Use Preferences to save position data persistently
    frc::Preferences::SetDouble("ElevatorPosition", positionInches);
}

double ElevatorPositionTracker::LoadPosition() {
    // Load the saved position; default to 0 if not found
    return frc::Preferences::GetDouble("ElevatorPosition", 0.0);
}
