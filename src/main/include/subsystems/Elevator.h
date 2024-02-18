#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "utils/ElevatorPositionTracker.h"

using namespace ElevatorConstants;

class Elevator : public frc2::Subsystem {
 public:
  Elevator();
  void Periodic() override;
  void MoveToPosition(double position);
  bool AtTargetPosition();
  // m_ElevatorEncoder.GetPosition();

  // Helper methods
  void ConfigureMotors();
  void UpdatePosition();
  double CalculateTargetHeight(units::degree_t theta2);
  double InchesToRotations(double inches);
  double RotationsToInches(double revolution);

  rev::SparkMaxAbsoluteEncoder
  GetkElevatorThroughBoreEncoder();  // these functions are needed to get
                                     // private class attributes
  rev::SparkMaxPIDController
  GetkElevatorPIDController();  // these functions are needed to get private
                                // class attributes

private:
  rev::CANSparkMax m_ElevatorMotorLeft{
      kElevatorLeftCanId, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_ElevatorMotorRight{
      kElevatorRightCanId, rev::CANSparkMax::MotorType::kBrushless};
  // Changed to SparkMaxAbsoluteEncoder for absolute position measurement
  // private class attributes

  rev::SparkMaxAbsoluteEncoder m_ElevatorThroughBoreEncoder =
      m_ElevatorMotorLeft.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle);
  // Define CANPIDController for direct control through SparkMax

  rev::SparkRelativeEncoder m_ElevatorRelativeEncoder =
      m_ElevatorMotorLeft.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor, 42);

  rev::SparkMaxPIDController m_ElevatorPIDController =
      m_ElevatorMotorLeft.GetPIDController();

  ElevatorPositionTracker positionTracker;

  double currentPositionInches = 0;  // Current elevator position in inches
  double targetPositionInches = 0;   // Target elevator position in inches
  // trigger: xbox controller button

  // go down
  // trigger: xbox controller button

  // throughbore encoder
  // 2 motors, one inverted

  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder()
   * method on an existing CANSparkMax object. If using a REV Through Bore
   * Encoder, the type should be set to quadrature and the counts per
   * revolution set to 8192
   */
};
