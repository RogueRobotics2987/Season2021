// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "IntakeOut.h"
#include "PickupBall.h"
#include <frc2/Timer.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoPickup
    : public frc2::CommandHelper<frc2::CommandBase, AutoPickup> {
 public:
  AutoPickup(Intake* c_intake, bool c_run, double c_time);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;


 private:

  frc::Timer* m_timer = nullptr;

  bool m_run = false;

  double m_time = 0;
  
  Intake* m_intake = nullptr;


};
