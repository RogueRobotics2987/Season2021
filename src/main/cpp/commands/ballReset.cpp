/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ballReset.h"

ballReset::ballReset(Intake* c_intake) {
  m_intake = c_intake; 
  AddRequirements(m_intake); 
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ballReset::Initialize() {
  m_intake->ResetBallCount(); 
}

// Called repeatedly when this Command is scheduled to run
void ballReset::Execute() {}

// Called once the command ends or is interrupted.
void ballReset::End(bool interrupted) {}

// Returns true when the command should end.
bool ballReset::IsFinished() { return true; }
