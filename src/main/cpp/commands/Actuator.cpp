/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Actuator.h"

Actuator::Actuator(ActuatorSubsystem* m_Actuator):m_Actuator(m_Actuator) {
  AddRequirements(m_Actuator);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Actuator::Initialize() {
  
}



// Called repeatedly when this Command is scheduled to run
void Actuator::Execute() {

 
}

// Called once the command ends or is interrupted.
void Actuator::End(bool interrupted) {
}

// Returns true when the command should end.
bool Actuator::IsFinished() { 
  return false;
    
    

 }