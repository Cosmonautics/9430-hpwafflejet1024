#pragma once 


#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/XboxController.h>
#include <rev/CanSparkMax.h>

#include "Constants.h"

using namespace ElevatorConstants;

class Elevator : public frc2::Subsystem {
    public: 
        Elevator();
  
// go up
    // trigger: xbox controller button

// go down
    // trigger: xbox controller button

    
//throughbore encoder

static constexpr int kCPR = 8192;

/**
* An alternate encoder object is constructed using the GetAlternateEncoder()
* method on an existing CANSparkMax object. If using a REV Through Bore
* Encoder, the type should be set to quadrature and the counts per
* revolution set to 8192
*/



//PID controller
    

    //values
    

// 2 motors, one inverted 
    
    private: 
    rev::CANSparkMax m_ElevatorMotorLeft;
    rev::CANSparkMax m_ElevatorMotorRight;
    
    rev::SparkPIDController m_ElevatorPIDController =
        m_ElevatorMotorLeft.GetPIDController();
  
     rev::SparkMaxAlternateEncoder m_ElevatorThroughboreEncoder =
        m_ElevatorMotorLeft.GetAlternateEncoder(rev::SparkMaxAlternateEncoder::Type::kQuadrature,
            kCPR);
 
};
