#include "commands2/DoAlignShooterWithAprilTagCommand.h"

#include <commands2/DoAlignShooterWithAprilTagCommand.h>
#include <frc/DriverStation.h>
#include <units/angular_velocity.h>

#include <cmath>

DoAlignShooterWithAprilTagCommand::DoAlignShooterWithAprilTagCommand(
    Shooter* shooter, Limelight* limelight)
    : m_shooter(shooter),
      m_limelight(limelight),
      m_targetOffsetAngleHorizontal(0.0),
      m_isAligned(false) {
  AddRequirements({shooter, limelight});
}

void DoAlignShooterWithAprilTagCommand::Initialize() {
  m_limelight
      ->SetLEDOn();  // Assuming you want to turn on LEDs to detect AprilTag
  m_isAligned = false;
}

void DoAlignShooterWithAprilTagCommand::Execute() {
  if (m_limelight->HasTarget()) {
    m_shooter->SetAngleBasedOnDistance(m_limelight->CalculateDistanceToTarget(
        frc::DriverStation::GetAlliance() == frc::DriverStation::kRed));
  }
  m_isAligned = true;
}

void DoAlignShooterWithAprilTagCommand::End(bool interrupted) {
  m_limelight->SetLEDOff();  // Turn off LEDs after alignment
}

bool DoAlignShooterWithAprilTagCommand::IsFinished() { return m_isAligned; }
