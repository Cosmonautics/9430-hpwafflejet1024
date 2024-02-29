#ifndef DO_SPEAKER_SCORE_ACTION_COMMAND_H
#define DO_SPEAKER_SCORE_ACTION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"

class DoSpeakerScoreActionCommand
    : public frc2::CommandHelper<frc2::Command, DoSpeakerScoreActionCommand> {
 public:
  DoSpeakerScoreActionCommand(Elevator* elevatorSubsystem,
                              Shooter* shooterSubsystem);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Elevator* m_elevatorSubsystem;
  Shooter* m_shooterSubsystem;
  bool cmdFinished;
  frc::Timer* timer;
};

#endif  // DO_SPEAKER_SCORE_ACTION_COMMAND_H