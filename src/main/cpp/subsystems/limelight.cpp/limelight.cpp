#include <networktables/NetworkTable.h>
#include <networktables/Topic.h>

nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("<variablename>",0.0);


