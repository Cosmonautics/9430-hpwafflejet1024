#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

class GoToFloorIntakePositionCommand 
    : public frc2::CommandHelper<frc2::Command, GoToFloorIntakePositionCommand> {
    
    public:
        explicit GoToFloorIntakePositionCommand(Elevator& elevator, Shooter& shooter, Intake& intake);
        void Initialize() override;
        bool IsFinished() override;
        void Execute() override;

    private:
        Elevator& m_elevator;
        Shooter& m_shooter;
        Intake& m_intake;

};