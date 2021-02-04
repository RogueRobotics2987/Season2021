// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include "subsystems/DifferentialDriveSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DifferentialDriveCommand : public frc2::CommandHelper<frc2::CommandBase, DifferentialDriveCommand> {
    
 public:
  /**
   * Creates a new DifferentialDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  DifferentialDriveCommand(DifferentialDriveSubsystem* subsystem, frc::Joystick* stick);
  //DifferentialDriveCommand(ArmSubsystem* subsystem, frc::Joystick* xbox);
  void Execute() override;
  bool IsFinished() override;
  void End(bool Interrupted) override;
 private:
 DifferentialDriveSubsystem* m_subsystem;
  frc::Joystick* stick;
  };
