# MAXSwerve C++ Template v2023.1

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Cpp-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

A template project for an FRC swerve drivetrain that uses REV MAXSwerve Modules.

Note that this is meant to be used with a drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in the Hardware Client using the `Absolute Encoder` tab under the associated turning SPARK MAX devices.

## Prerequisites

* SPARK MAX Firmware v1.6.2 - Adds features that are required for swerve
* REVLib v2023.1.2 - Includes APIs for the new firmware features
* XBox One Gamepad
* AHRS Libraries (navX MXP)

## Robot Controls
This section is written for our drive team as a reference for robot controls. The listed controls are for an XBox One Gamepad, which is required for operation.
### Operator Guide

#### Robot Positions

* AMP Score / Speaker Score Position : (*TOGGLE*) Press **D-Pad Right** to switch between AMP Scoring Mode and Speaker Scoring Mode
* Climb : Press **D-Pad Up**
* Floor Intake Position : **Press D-Pad Left** 
* Transit Position : Press **D-Pad Down**

#### Robot Actions

* Climb Action : -- Postpone (Press **Y**)
* Note Intake Action : Press **Right Bumper**
* Note Eject Action : Press **Left Bumper**
* Speaker Score Action : Press **B**

### Driver Guide

#### Movement 
* **Left Thumbstick**: Moves and strafes the robot relative to the field
* **Right Thumbstick**: 
    * Left: Rotate the robot counterclockwise
    * Right: Rotate the robot clockwise 
* **SELECT**: Press to reset robot heading for field relative drive
    * Face the front of the robot in the same direction of the Driver (the driver and the robot should be facing the same way).
    * Press **SELECT** and observe haptic feedback. 



## Configuration

### Assigned CAN IDs 

# Resources

## WPILib Documentation Homepage
+ https://docs.wpilib.org/en/stable/

## Zero To Robot Programming Fundamentals
+ https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html

## WPILib Programming Resources
+ https://docs.wpilib.org/en/stable/stubs/programming-basics-stub.html (Basic)
+ https://docs.wpilib.org/en/stable/stubs/advanced-programming-stub.html (Advanced)
+ https://github.wpilib.org/allwpilib/docs/release/cpp/index.html (Full WPILibC++ Docs)

## REV and CTRE API Docs
+ https://docs.revrobotics.com/docs/
+ https://store.ctr-electronics.com/software/

## Swerve Drive Programming Resources
+ Template MaxSwerve Project Source: https://github.com/REVrobotics/MAXSwerve-Cpp-Template
+ Working CPP code: https://github.com/ahayden04/swerve-falcon
+ Example of swerve robot: https://www.youtube.com/watch?v=wc8wh6RsMCs
+ Source (Chief Delphi): https://www.chiefdelphi.com/t/working-c-swerve-drive-super-proud/407116

# Branching
- git branch (new_branch) -- create a new local branch
    - same name as upstream origin/(branch_name)
- git branch -u (existing_branch) -- Set upstream branch to (existing_branch)
    - This will set your local branch to **push** committed changes to the target upstream branch on origin/(existing_branch)
- git checkout (existing_branch) -- switch to an existing local branch 
    - switch out your local copy to **commit** to an existing local branch (existing_branch)
- git push --set-upstream origin (branch_name)
    - push changes explicitly to (branch_name)
    - NEVER PUSH TO MAIN