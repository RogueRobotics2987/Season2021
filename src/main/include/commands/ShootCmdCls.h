/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Shooter.h"
#include <frc/Joystick.h> 
#include <iostream>

using namespace std;

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootCmdCls
    : public frc2::CommandHelper<frc2::CommandBase, ShootCmdCls> {
 public:
 //not sure why we need the extra joystsick
  ShootCmdCls(Shooter* a_shooter /*, frc::Joystick* a_joystick */);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  private:
  Shooter* m_shooter=nullptr;
  //frc::Joystick* m_joystick;
};
