// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/teleopCommand.h"


TeleopCommand::TeleopCommand(DifferentialDriveSubsystem* subsystem, frc::Joystick* stick)
    : m_subsystem{subsystem} {}
TeleopCommand::TeleopCommand(ArmSubsystem* subsystem, frc::Joystick* xbox)
    : m_armsubsystem{subsystem} {}

