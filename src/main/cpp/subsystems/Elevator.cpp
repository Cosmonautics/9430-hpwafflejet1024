// include statements

// if X button pressed, move X meters

// set according to power / xbox button pressed

// bunch of static constant expressions defining basic physic limits and voltages

// setting variables for joystick, encoder, and sparkmax from FRC module

// create PID controller with constraints

#include "subsystems/Elevator.h"
#include "Constants.h"

using namespace ElevatorConstants;

Elevator::Elevator(rev::CANSparkMax m_ElevatorMoterLeft)
    : m_ElevatorMoterLeft{kElevatorLeftCanId,rev::CANSparkMaxLowLevel::MotorType::kBrushless} {}
    
