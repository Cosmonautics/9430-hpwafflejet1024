#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include <memory>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

int main() {
    // Initialize NetworkTables instance
    auto ntinst = nt::NetworkTableInstance::GetDefault();
    auto table = ntinst.GetTable("limelight");

    // Retrieve values from the "limelight" table
    double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
    double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
    double targetArea = table->GetNumber("ta", 0.0);
    double targetSkew = table->GetNumber("ts", 0.0);

}
