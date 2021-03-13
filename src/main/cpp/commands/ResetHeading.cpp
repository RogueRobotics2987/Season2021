/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ResetHeading.h"

ResetHeading::ResetHeading(DriveSubsystem* m_Drive):m_Drive(m_Drive) {
  AddRequirements(m_Drive);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ResetHeading::Initialize() {
  
}



// Called repeatedly when this Command is scheduled to run
void ResetHeading::Execute() {
   m_Drive->ZeroHeading(); 

}

// Called once the command ends or is interrupted.
void ResetHeading::End(bool interrupted) {
}

// Returns true when the command should end.
bool ResetHeading::IsFinished() { 
  return true;
    
    

 }