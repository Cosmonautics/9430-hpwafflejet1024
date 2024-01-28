#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <rev/CANSparkFlex.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "MAXSwerveModule.h"

class Intake : public frc2::Subsystem {
 public:
  Intake();

  void PickUpNote(bool isPressed, double speed);

  void DropNote(bool isPressed, double speed);

  void ControlIntakeMotors(bool isPressed, double speed);

 private:
  // 2 motors
  // Design:
  // 1 for pivot angle
  rev::CANSparkMax m_intakePivotMax;
  // 1 motor for rollers
  rev::CANSparkFlex m_intakeRollersFlex;

  // 1 throughbore encoder
};