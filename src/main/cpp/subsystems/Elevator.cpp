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
  m_pidController.SetTolerance(kPositionToleranceCm);
}

void Elevator::Periodic() { UpdatePosition(); }

void Elevator::MoveToPosition(double positionCm) {
  targetPositionCm = positionCm;  // Update the target position
  double targetPositionUnits = ConvertCmToEncoderUnits(positionCm);
  double currentPositionUnits = m_ElevatorEncoder.GetPosition();
  double output =
      m_pidController.Calculate(currentPositionUnits, targetPositionUnits);
  Move(output);
}
bool Elevator::AtTargetPosition() const {
  return std::abs(currentPositionCm - targetPositionCm) <=
         ElevatorConstants::kPositionToleranceCm;
}
void Elevator::ConfigureMotors() {
  m_ElevatorMotorLeft.RestoreFactoryDefaults();
  m_ElevatorMotorRight.RestoreFactoryDefaults();
  m_ElevatorMotorRight.Follow(m_ElevatorMotorLeft, true);
}

void Elevator::UpdatePosition() {
  currentPositionCm = ConvertEncoderUnitsToCm(m_ElevatorEncoder.GetPosition());
}

double Elevator::ConvertCmToEncoderUnits(double cm) {
  return cm / kEncoderUnitsPerCm;
}

double Elevator::ConvertEncoderUnitsToCm(double units) {
  return units * kEncoderUnitsPerCm;
}

void Elevator::Move(double speed) {
  m_ElevatorMotorLeft.Set(speed);
  // m_ElevatorMotorRight is following m_ElevatorMotorLeft in inverted mode, so
  // no need to set it separately
}