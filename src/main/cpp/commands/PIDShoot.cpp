/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/PIDShoot.h"

PIDShoot::PIDShoot(Shooter* c_shooter, Intake* c_intake) {
  m_intake = c_intake; 
  m_shooter = c_shooter;
  AddRequirements(m_shooter);
  //AddRequirements(m_intake);
  
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PIDShoot::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void PIDShoot::Execute() {


  if(m_shooter->getVelocity() < 500){
    m_shooter->setPercent(0.5);
  } else {
    m_shooter->setShooter();
  }
  //m_shooter->setPercent(.8);


  
 

}

// Called once the command ends or is interrupted.
void PIDShoot::End(bool interrupted) {
  m_intake->ResetBallCount(); 
  m_intake->resetOutBalls(); 
  m_shooter->stopShooter();
}

// Returns true when the command should end.
bool PIDShoot::IsFinished() { return false; }
