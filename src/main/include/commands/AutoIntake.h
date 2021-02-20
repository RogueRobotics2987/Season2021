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
class AutoIntake
    : public frc2::CommandHelper<frc2::CommandBase, AutoIntake> {
 public:
  AutoIntake(/*Intake m_intakeOut, PickupBall m_pickupBall, bool IsFinished()*/);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc::Timer* m_timer = nullptr;
};
