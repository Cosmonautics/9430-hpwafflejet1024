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

class Elevator : public frc2::Subsystem {
    public: 
    std::cout << "hello";
// go up
    // trigger: xbox controller button

// go down
    // trigger: xbox controller button

    //throughbore encoder
// 2 motors, one inverted 
    

    private: 
    rev::CANSparkMax m_ElevatorMoterLeft{DriveConstants:: kElevatorLeftCanId,rev::CANSparkMaxLowLevel::MotorType::kBrushless};

    rev::CANSparkMax m_ElevatorMoterLeft{DriveConstants:: kElevatorLeftCanId,rev::CANSparkMaxLowLevel::MotorType::kBrushless};
}
    

    

