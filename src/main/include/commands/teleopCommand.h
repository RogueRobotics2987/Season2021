// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
<<<<<<< HEAD

=======
#include <frc/Joystick.h>
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef
#include "subsystems/DifferentialDriveSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TeleopCommand
    : public frc2::CommandHelper<frc2::CommandBase, TeleopCommand> {
 public:
  /**
<<<<<<< HEAD
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit TeleopCommand(DifferentialDriveSubsystem* subsystem);

 private:
  DifferentialDriveSubsystem* m_subsystem;
 
=======
   * Creates a new TeleopCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit TeleopCommand(DifferentialDriveSubsystem* subsystem, frc::Joystick* stick);

 private:
 DifferentialDriveSubsystem* m_subsystem;
  
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef
};
