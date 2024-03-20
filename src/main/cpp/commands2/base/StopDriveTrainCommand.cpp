#include "commands2/base/StopDriveTrainCommand.h"  // include command header

StopDriveTrainCommand::StopDriveTrainCommand(
    DriveSubsystem* driveSubsystem)  // add all the relevant subsystems to constructor
    : m_driveSubsystem(driveSubsystem) {
  AddRequirements({driveSubsystem});
}

void StopDriveTrainCommand::Initialize() {}

void StopDriveTrainCommand::Execute() { m_driveSubsystem->Drive(0_mps, 0_mps, 0_rad_per_s, false, false); }

bool StopDriveTrainCommand::IsFinished() { return true; }
