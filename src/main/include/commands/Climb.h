/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Climber.h"
#include <frc/Joystick.h>
#include <frc/Timer.h> 
#include <frc/smartdashboard/SmartDashboard.h> 

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Climb
    : public frc2::CommandHelper<frc2::CommandBase, Climb> {
 public:
  Climb(Climber* c_climber, frc::Joystick* p_stick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Climber* m_climber;
  frc::Joystick* m_stick;
  frc::Timer* matchTimer; 
};
