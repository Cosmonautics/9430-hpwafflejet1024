#ifndef DRL_h
#define DRL_h // rename these to the name of the command in all caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/Limelight.h" // import all relevant subsystems 

class DoRealignGyro : public frc2::CommandHelper<frc2::Command, DoRealignGyro> {
public:
    DoRealignGyro(Limelight* limelight); // Put in all relevant subsystems

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Limelight* m_limelightSubsystem;
};

#endif // COMMAND_TEMPLATE_H // (replace this with the name of the command name)