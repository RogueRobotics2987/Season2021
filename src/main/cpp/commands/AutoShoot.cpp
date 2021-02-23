// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoShoot.h"

AutoShoot::AutoShoot(Shooter* c_shooter, ShooterActuator* c_actuator, Intake* c_intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_actuator = c_actuator;
  m_shooter = c_shooter;
  m_intake = c_intake;
  myTimer1 = new frc::Timer;

  AddRequirements(m_shooter);
  AddRequirements(m_actuator);
  AddRequirements(m_intake);
}

// Called when the command is initially scheduled.
void AutoShoot::Initialize() {
  myTimer1 -> Reset();
  myTimer1 -> Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoShoot::Execute() {
    currTime = myTimer1 -> Get();
    if (currTime >= 45 && currTime <= 70) {

    
      if(m_shooter->getVelocity() < 500){
         m_shooter->setPercent(.5);
      }else{
         m_shooter->setShooter(3950); 
      }
    } else {
        m_shooter->stopShooter();
    }

  //m_actuator->SetAutoAim(true); //from Brandon's autonomous
   // m_actuator->setAngleH(0); 
   // m_actuator->setAngleV(0); 
    if(m_shooter->getVelocity() >= .95 * 3950 && m_actuator->GetTX() < 1 && m_actuator->GetTX() > -1 && m_actuator->GetTY() < 1 && m_actuator->GetTY() > -1){
      
      m_intake->StartConveyor(.5);
    } else{
      m_intake->StartConveyor(0);
    }
}

// Called once the command ends or is interrupted.
void AutoShoot::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoShoot::IsFinished() {
   
  return false;
}
