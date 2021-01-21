/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/BalanceRobot.h"

BalanceRobot::BalanceRobot(Climber* c_climber) {
  
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BalanceRobot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void BalanceRobot::Execute() {
}

// Called once the command ends or is interrupted.
void BalanceRobot::End(bool interrupted) {}

// Returns true when the command should end.
bool BalanceRobot::IsFinished() { return false; }
