#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "Constants.h"
using namespace pathplanner;

class DoFollowPathPlannerPath
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 DoFollowPathPlannerPath> {
 public:
  DoFollowPathPlannerPath(std::string pathPath);
};