// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoShoot.h"

AutoShoot::AutoShoot(Shooter* c_shooter, Intake* c_intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake = c_intake; 
  m_shooter = c_shooter;
  AddRequirements(m_shooter);
  //AddRequirements(m_intake);
  myTimer1 = new frc::Timer;
}

// Called when the command is initially scheduled.
void AutoShoot::Initialize() {
  myTimer1 -> Reset();
  myTimer1 -> Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoShoot::Execute() {
    curTime = myTimer1 -> Get();

// if (curTime )
if(m_shooter->getVelocity() < 500){ //from PIDShoot
    m_shooter->setPercent(0.5);
  } else {
    m_shooter->setShooter(4000);
  }
/*if(m_xbox->GetRawButton(1)){
    m_intake->StartConveyor(.5);
    dontRun = true; 
    }else{
    dontRun = false; 
  } */ //from pickupBall 
}

// Called once the command ends or is interrupted.
void AutoShoot::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoShoot::IsFinished() {
  return false;
}
