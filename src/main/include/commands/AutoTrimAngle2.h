// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterActuator.h" 
//#include "subsystems/Shooter.h" 
#include <frc/Timer.h> 
#include <frc/smartdashboard/SmartDashboard.h> 
#include <iostream>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoTrimAngle
    : public frc2::CommandHelper<frc2::CommandBase, AutoTrimAngle> {
 public:
  AutoTrimAngle(ShooterActuator* a_actuator, bool shootingMode);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ShooterActuator* m_actuator = nullptr; 
  bool m_shootingMode= false;
  frc::Timer* myTimer = nullptr;
  double curTime = 0;
};
