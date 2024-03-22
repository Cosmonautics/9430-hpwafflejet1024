#include "commands2/DoRealignGyro.h"  // include command header

DoRealignGyro::DoRealignGyro(  // Find and Replace "DoRealignGyro" with the name
                               // of the command
    Limelight* limelightSubSystem)      // add all the relevant subsystems to constructor
    : m_limelightSubsystem(limelightSubSystem) {
  AddRequirements({m_limelightSubsystem});
}

void DoRealignGyro::Initialize() {}

void DoRealignGyro::Execute() {
  // Put code here
}

bool DoRealignGyro::IsFinished() {}
