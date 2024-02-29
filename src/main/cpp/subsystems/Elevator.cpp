// include statements

// if X button pressed, move X meters

// set according to power / xbox button pressed

// bunch of static constant expressions defining basic physic limits and
// voltages

// setting variables for joystick, encoder, and sparkmax from FRC module

// create PID controller with constraints

#include "subsystems/Elevator.h"

#include <cmath>  // For std::abs

#include "Constants.h"

using namespace ElevatorConstants;

Elevator::Elevator() { ConfigureMotors(); }

void Elevator::ConfigureMotors() {
  m_ElevatorMotorLeft.RestoreFactoryDefaults();
  m_ElevatorMotorRight.RestoreFactoryDefaults();
  m_ElevatorMotorLeft.Follow(m_ElevatorMotorRight, true);
  
  m_ElevatorMotorLeft.SetSmartCurrentLimit(40);
  m_ElevatorMotorRight.SetSmartCurrentLimit(40);
  // Configure PID controller on SparkMax
  // m_ElevatorPIDController.SetPositionPIDWrappingEnabled(true);
  try {
    m_ElevatorPIDController.SetFeedbackDevice(m_ElevatorThroughBoreEncoder);
  } catch (std::exception &ex) {
    std::string what_string = ex.what();
    std::string err_msg("Error Setting feedback device:  " + what_string);
    const char *p_err_msg = err_msg.c_str();
    // std::cout << p_err_msg;
    std::cout << err_msg;
  }
  m_ElevatorPIDController.SetP(kP);
  m_ElevatorPIDController.SetI(kI);
  m_ElevatorPIDController.SetD(kD);
  m_ElevatorPIDController.SetOutputRange(-1.0, 1.0);
  m_ElevatorMotorRight.SetSoftLimit(
      rev::CANSparkBase::SoftLimitDirection::kForward,
      ElevatorConstants::kElevatorForwardSoftLimit);
  m_ElevatorMotorRight.SetSoftLimit(
      rev::CANSparkBase::SoftLimitDirection::kReverse,
      ElevatorConstants::kElevatorReverseSoftLimit);
  m_ElevatorMotorRight.EnableSoftLimit(
      rev::CANSparkBase::SoftLimitDirection::kForward, true);
  m_ElevatorMotorRight.EnableSoftLimit(
      rev::CANSparkBase::SoftLimitDirection::kReverse, true);
}

void Elevator::MoveToPosition(double positionRotations, bool isClimb) {
  if (isClimb) {
    m_ElevatorPIDController.SetP(1.0); // once elevator moves to climb position, it will have a constant P value set to 1.0 
  } else {
    m_ElevatorPIDController.SetP(ElevatorConstants::kP);
  }
  double targetPositionInches = RotationsToInches(positionRotations);
  m_ElevatorPIDController.SetReference(positionRotations,
                                       rev::ControlType::kPosition);
}

void Elevator::Periodic() { UpdatePosition(); }

void Elevator::SetToBrakeMode() {
  m_ElevatorMotorLeft.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_ElevatorMotorRight.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
}

void Elevator::UpdatePosition() {
  double currentPositionRotations = m_ElevatorThroughBoreEncoder.GetPosition();
  currentPositionInches = RotationsToInches(currentPositionRotations);
}

double Elevator::CalculateTargetHeight(units::degree_t targetRevolutions) {
  units::degree_t currentRevolutions = units::degree_t{
      m_ElevatorThroughBoreEncoder.GetPosition() / kElevatorEncoderResolution};

  units::degree_t revolutionDifference = targetRevolutions - currentRevolutions;

  return revolutionDifference.to<double>();
}

double Elevator::InchesToRotations(double inches) {
  return ((((kGearBoxScale) * (inches)) / (kPullyDiameter * M_PI)));
}

double Elevator::RotationsToInches(double rotations) {
  return (rotations * kPullyDiameter * M_PI) / kGearBoxScale;
}

bool Elevator::AtTargetPosition() {
  bool flag = std::abs(currentPositionInches - targetPositionInches) <=
              ElevatorConstants::kPositionToleranceInches;
  return flag;
}

bool Elevator::ToggleManualOverride() {
  manualOverride = !manualOverride;
  return manualOverride;
}

void Elevator::ManualMove(double speed) {
  if (manualOverride) {
    double currentElevatorPosition = m_ElevatorThroughBoreEncoder.GetPosition();

    double speedFactor = 0.01;
    double targetPositionRotations =
        currentElevatorPosition + (speed * speedFactor);
    double targetPositionInches = RotationsToInches(targetPositionRotations);

    m_ElevatorPIDController.SetReference(targetPositionRotations,
                                         rev::ControlType::kPosition);
  }
}