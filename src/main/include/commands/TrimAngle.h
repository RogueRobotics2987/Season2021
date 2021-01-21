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
#include <frc/XboxController.h>
#include "subsystems/ShooterActuator.h"
#include <frc/Timer.h>


using namespace std;

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TrimAngle
    : public frc2::CommandHelper<frc2::CommandBase, TrimAngle> {
 public:
  TrimAngle(frc::Joystick* cStick, ShooterActuator* a_actuator, frc::Joystick* cJoy);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

   private: 
   ShooterActuator* m_actuator;
   // Real Val
   frc::Joystick* rStick; 
   frc::Timer* myTimer;
   frc::Joystick* mJoy; 
   bool camVal = false; 
   bool run = true; 


};
