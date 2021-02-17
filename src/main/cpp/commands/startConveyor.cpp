/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/startConveyor.h"

startConveyor::startConveyor(Intake* c_intake, double c_conveyorVal) {
  m_conveyorVal = c_conveyorVal; 
  m_intake = c_intake; 
  //AddRequirements(m_intake); 
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void startConveyor::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void startConveyor::Execute() {
  m_intake->StartConveyor(m_conveyorVal); 
}

// Called once the command ends or is interrupted.
void startConveyor::End(bool interrupted) {
  m_intake->StopMotors(); 
}

// Returns true when the command should end.
bool startConveyor::IsFinished() { return false; }
