/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Climb.h"

Climb::Climb(Climber* c_climber, frc::Joystick* p_stick) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_climber = c_climber;
  AddRequirements(m_climber);
  m_stick = p_stick;
  matchTimer = new frc::Timer; 
}

// Called when the command is initially scheduled.
void Climb::Initialize() {
  matchTimer->Start(); 
  matchTimer->Reset(); 
  m_climber->movePin(false); 


  frc::SmartDashboard::PutNumber("Init Test Time", matchTimer->Get()); 
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  frc::SmartDashboard::PutNumber("Time Left in Match", 150 - matchTimer->Get()); 
  //Fwd True 
  // Rev False 
  double curLeftTrig = m_stick->GetRawAxis(2);
  double curRightTrig = m_stick->GetRawAxis(3);

  if (curLeftTrig > 0.1 && curRightTrig < 0.1){
    m_climber->TheClimb(curLeftTrig);
  } else if (curRightTrig > 0.1 && curLeftTrig < 0.1){
    m_climber->TheClimb(-curRightTrig);
  } else {
    m_climber->TheClimb(0.0);
  }
  if(150 - matchTimer->Get() < .5){
    m_climber->movePin(true); 
  }
  //m_climber->TheClimb(m_stick->GetRawAxis(5)); 
  m_climber->Balance(m_stick->GetRawAxis(4)); // updated button
  /*if(m_stick->GetPOV() == 0) {
//  if(m_stick->GetRawButton(5)){
    m_climber->movePin(true); 
  } 
  else if (m_stick->GetPOV() == 180) {
  m_climber->movePin(false); 
  }*/

}



// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {}

// Returns true when the command should end.
bool Climb::IsFinished() { return false; }
