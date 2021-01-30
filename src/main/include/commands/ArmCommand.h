// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include "subsystems/ArmSubsystem.h"


/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ArmCommand : public frc2::CommandHelper<frc2::CommandBase, ArmCommand> {
    
 public:
  /**
   * Creates a new ArmCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  ArmCommand(ArmSubsystem* subsystem, frc::Joystick* stick);
  //ArmCommand(ArmSubsystem* subsystem, frc::Joystick* xbox);
  void Execute() override;
  bool IsFinished() override;
  void End(bool Interrupted) override;
 private:
 ArmSubsystem* m_subsystem;
 frc::Joystick* stick;
frc::Joystick* xbox;
};
