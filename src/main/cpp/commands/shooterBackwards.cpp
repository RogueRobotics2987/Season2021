/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/shooterBackwards.h"

shooterBackwards::shooterBackwards(Shooter* c_shooter) {
  m_shooter = c_shooter; 
  AddRequirements(m_shooter); 
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void shooterBackwards::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void shooterBackwards::Execute() {
  m_shooter->setPercent(-.3); 
}

// Called once the command ends or is interrupted.
void shooterBackwards::End(bool interrupted) {
  m_shooter->setPercent(0); 
}

// Returns true when the command should end.
bool shooterBackwards::IsFinished() { return false; }
