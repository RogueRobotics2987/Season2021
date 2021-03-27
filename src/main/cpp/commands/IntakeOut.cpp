/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeOut.h"

IntakeOut::IntakeOut(Intake* c_intake, bool SolState) {
  m_intake = c_intake; 
  m_SolState = SolState;
  AddRequirements(m_intake); 
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IntakeOut::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeOut::Execute() {
  
  if (m_SolState == true){
    m_intake->setSolenoidTrue(); 
  } else {
    m_intake->setSolenoidFalse(); 
  }
  
}

// Called once the command ends or is interrupted.
void IntakeOut::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeOut::IsFinished() { return true; }
