#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class Intake : public frc2::Subsystem {
 public:
  Intake();

  void IntakePickUpNote(bool isPressed, double speed);

  void IntakeDropNote(bool isPressed, double speed);

  void ControlIntakeMotors(bool isPressed, double speed);
 private:
  // 2 motors
  // Design:
  // 1 for pivot angle

  rev::CANSparkFlex m_intakeMotorLeft{
      IntakeConstants::kIntakeLeftCanId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_intakeMotorRight{
      IntakeConstants::kIntakeRightCanId,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  // rev::CANSparkMax m_intakePivotMax;
  //  1 motor for rollers
  // rev::CANSparkFlex m_intakeRollersFlex;
  //  m_intakePivotMax(IntakeConstants::kIntakeRightCanId,rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  //  m_intakeRollersFlex(IntakeConstants::kIntakeLeftCanId,rev::CANSparkLowLevel::MotorType::kBrushless)
  //  1 throughbore encoder
};