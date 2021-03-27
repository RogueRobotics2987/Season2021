/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/PickupBall.h"


PickupBall::PickupBall(Intake* c_intake, frc::Joystick* c_xbox, frc::Joystick* c_Lstick)  {
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake = c_intake;
  m_xbox = c_xbox; 
  m_Lstick = c_Lstick;

  AddRequirements(m_intake);
}

// Called when the command is initially scheduled.
void PickupBall::Initialize() {
  m_intake->startTimer(); 
}

// Called repeatedly when this Command is scheduled to run
void PickupBall::Execute() {
  if(m_xbox->GetRawButton(4) || (m_Lstick->GetRawButton(1))){  
    // updated button
    m_intake->IntakeBall(.65); // intake in

  }

  else if(m_xbox->GetRawButton(2)){ // updated button
  
    m_intake->IntakeBall(-.65); // intake out
  }
  else{
    m_intake->IntakeBall(0); 
    
  }

  
  
  if(m_xbox->GetRawButton(1)){
    m_intake->StartConveyor(.5);
    dontRun = true; 

    m_intake->ResetBallCount();
    
  }
  else if(m_Lstick->GetRawButton(11)){
    m_intake->StartConveyor(-.3); 
    dontRun = true; 
  }
  else{
    dontRun = false; 
  }
 

  if(!dontRun){
  m_intake->PrepareBall();}
  
}
    

// Called once the command ends or is interrupted.
void PickupBall::End(bool interrupted) {
  m_intake->StopMotors();
}

// Returns true when the command should end.
bool PickupBall::IsFinished() { return false; }
