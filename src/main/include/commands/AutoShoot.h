// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h" 
#include "subsystems/ShooterActuator.h"
#include <frc/Timer.h> 

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoShoot
    : public frc2::CommandHelper<frc2::CommandBase, AutoShoot> {
 public:
  AutoShoot(Shooter* c_shooter, ShooterActuator* c_actuator, Intake* c_intake, double spinupTime, double shootTime);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  Shooter* m_shooter = nullptr;
  Intake* m_intake = nullptr;
  ShooterActuator* m_actuator = nullptr;
  frc::Timer* myTimer1 = nullptr;
  double currTime = 0;
  double m_spinupTime = 5;
  double m_shootTime = 15;
};
