#include <Robot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include "frc/GenericHID.h"
#include <frc/drive/differentialdrive.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
