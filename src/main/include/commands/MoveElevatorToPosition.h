#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h" // Include the Elevator subsystem header

class MoveElevatorToPosition : public frc2::CommandHelper<frc2::Command, MoveElevatorToPosition> {
public:
    explicit MoveElevatorToPosition(Elevator& elevator, double position);
    void Initialize() override;
    bool IsFinished() override;

private:
    Elevator& m_elevator;
    double m_position;
};
