// include statements

// if X button pressed, move X meters

// set according to power / xbox button pressed

// bunch of static constant expressions defining basic physic limits and
// voltages

// setting variables for joystick, encoder, and sparkmax from FRC module

// create PID controller with constraints

#include "subsystems/Elevator.h"

#include "Constants.h"

using namespace ElevatorConstants;

Elevator::Elevator() {
  ConfigureMotors();
  m_pidController.SetTolerance(kPositionToleranceInches);
}

void Elevator::Periodic() { UpdatePosition(); }

void Elevator::MoveToPosition(double positionInches) {
  if (positionInches > kElevatorUpperSoftLimit ||
      positionInches < kElevatorLowerSoftLimit) {
    return;
  }
  targetPositionInches = positionInches;
  double targetPositionUnits = ConvertInchesToEncoderUnits(positionInches);
  double currentPositionUnits = m_ElevatorEncoder.GetPosition();
  double output =
      m_pidController.Calculate(currentPositionUnits, targetPositionUnits);
  Move(output);
}
double Elevator::ConvertInchesToEncoderUnits(double inches) {
  return inches * ElevatorConstants::kEncoderUnitsPerInch;
}

double Elevator::ConvertEncoderUnitsToInches(double units) {
  return units / ElevatorConstants::kEncoderUnitsPerInch;
}

bool Elevator::AtTargetPosition() const {
  return std::abs(currentPositionInches - targetPositionInches) <=
         ElevatorConstants::kPositionToleranceInches;
}
void Elevator::ConfigureMotors() {
  m_ElevatorMotorLeft.RestoreFactoryDefaults();
  m_ElevatorMotorRight.RestoreFactoryDefaults();
  m_ElevatorMotorRight.Follow(m_ElevatorMotorLeft, true);
}

void Elevator::UpdatePosition() {
  currentPositionInches =
      ConvertEncoderUnitsToInches(m_ElevatorEncoder.GetPosition());
}

void Elevator::Move(double speed) {
  m_ElevatorMotorLeft.Set(speed);
  // m_ElevatorMotorRight is following m_ElevatorMotorLeft in inverted mode, so
  // no need to set it separately
}