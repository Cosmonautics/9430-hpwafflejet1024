#include <networktables/NetworkTable.h>
#include <networktables/Topic.h>
#include "subsystems/limelight.h"


std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight");