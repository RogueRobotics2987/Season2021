// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include "subsystems/DifferentialDriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"


/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TeleopCommand : public frc2::CommandHelper<frc2::CommandBase, TeleopCommand> {
    
 public:
  /**
   * Creates a new TeleopCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  TeleopCommand(DifferentialDriveSubsystem* subsystem, frc::Joystick* stick);
  TeleopCommand(ArmSubsystem* subsystem, frc::Joystick* xbox);
 private:
 DifferentialDriveSubsystem* m_subsystem;
 ArmSubsystem* m_armsubsystem;
  
};
