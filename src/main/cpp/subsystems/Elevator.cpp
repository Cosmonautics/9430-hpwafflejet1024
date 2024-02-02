// include statements

// if X button pressed, move X meters

// set according to power / xbox button pressed

// bunch of static constant expressions defining basic physic limits and voltages

// setting variables for joystick, encoder, and sparkmax from FRC module

// create PID controller with constraints

#include "subsystems/Elevator.h"
#include "Constants.h"

using namespace ElevatorConstants;

Elevator::Elevator() : 
    m_ElevatorMotorLeft{kElevatorLeftCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless}, 
    m_ElevatorMotorRight{kElevatorRightCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless}
    {
        //uses the throughbore encoder instead of the other one
        m_ElevatorPIDController.SetFeedbackDevice( m_ElevatorThroughboreEncoder);

        //Sets values
        m_ElevatorPIDController.SetP(kElevatorP);
        m_ElevatorPIDController.SetI(kElevatorI);
        m_ElevatorPIDController.SetD(kElevatorD);
        m_ElevatorPIDController.SetFF(kElevatorFF);
        m_ElevatorPIDController.SetOutputRange(kElevatorMinOutput, kElevatorMaxOutput);

        


}
    


