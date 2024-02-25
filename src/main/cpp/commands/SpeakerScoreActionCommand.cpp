#include "commands/SpeakerScoreActionCommand.h"

SpeakerScoreActionCommand::SpeakerScoreActionCommand(Shooter* shooterSubsystem)
: m_shooterSubsystem(shooterSubsystem) {
    AddRequirements({shooterSubsystem});
}

void SpeakerScoreActionCommand::Initialize() {

}

void SpeakerScoreActionCommand::Execute() {}

bool SpeakerScoreActionCommand::IsFinished() {

}