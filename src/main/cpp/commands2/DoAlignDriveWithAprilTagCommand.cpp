#include "commands2/DoAlignDriveWithAprilTagCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>

#include <cmath>

DoAlignDriveWithAprilTagCommand::DoAlignDriveWithAprilTagCommand(
    DriveSubsystem* drive, Limelight* limelight)
    : m_drive(drive),
      m_limelight(limelight),
      m_targetOffsetAngleHorizontal(0.0),
      m_isAligned(false) {
  AddRequirements({drive, limelight});
}

void DoAlignDriveWithAprilTagCommand::Initialize() {
  m_limelight
      ->SetLEDOn();  // Assuming you want to turn on LEDs to detect AprilTag
  m_isAligned = false;
}

void DoAlignDriveWithAprilTagCommand::Execute() {
  if (m_limelight->HasTarget()) {
    m_targetOffsetAngleHorizontal =
        m_limelight
            ->GetTargetX();  // Assuming this gets the horizontal offset angle
    double rotationSpeed =
        CalculateRotationSpeed(m_targetOffsetAngleHorizontal);

    // Rotate the robot at calculated speed. Negative speed if the angle is to
    // the left, positive if to the right.
    m_drive->Drive(0_mps, 0_mps, units::radians_per_second_t(-rotationSpeed),
                   false, false);
  } else {
    m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
    m_isAligned = true;
  }
}

void DoAlignDriveWithAprilTagCommand::End(bool interrupted) {
  m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
  m_limelight->SetLEDOff();  // Turn off LEDs after alignment
}

bool DoAlignDriveWithAprilTagCommand::IsFinished() { return m_isAligned; }

double DoAlignDriveWithAprilTagCommand::CalculateRotationSpeed(
    double targetOffsetAngle) {
  const double Kp = 0.03;
  double controlEffort = Kp * targetOffsetAngle;

  // Limit the control effort to maximum allowed rotation speed to prevent the
  // robot from rotating too fast
  const double maxRotationSpeed = 0.5;  // Maximum rotation speed
  controlEffort =
      std::clamp(controlEffort, -maxRotationSpeed, maxRotationSpeed);

  const double alignmentThreshold =
      2.0;  // Degrees, adjust based on desired precision
  m_isAligned = std::abs(targetOffsetAngle) < alignmentThreshold;

  return controlEffort;
}
