#include <networktables/NetworkTable.h>
#include <networktables/Topic.h>

nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
std::shared_ptr<nt::NetworkTable> table = inst.GetTable("datatable");
