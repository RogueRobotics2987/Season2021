/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Intake.h"
#include <frc/Joystick.h> 

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PickupBall
    : public frc2::CommandHelper<frc2::CommandBase, PickupBall> {
 public:
  PickupBall(Intake* c_intake, frc::Joystick* c_xbox, frc::Joystick* c_Lstick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Intake* m_intake;
  frc::Joystick* m_xbox; 
  frc::Joystick* m_Lstick; 
  bool dontRun = false; 


};
