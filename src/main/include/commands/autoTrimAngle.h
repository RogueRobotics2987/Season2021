// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h> 
#include "subsystems/ShooterActuator.h"
#include "subsystems/Shooter.h"
#include <iostream>
#include <frc/Timer.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class autoTrimAngle
    : public frc2::CommandHelper<frc2::CommandBase, autoTrimAngle> {
 public:
  autoTrimAngle(ShooterActuator* a_actuator, bool shootingMode);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  ShooterActuator* m_actuator = nullptr;
  bool m_shootingMode = false;
  frc::Timer* myTimer = nullptr;
  double currTime = 0;
  
};
