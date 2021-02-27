// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoTrimAngle.h"

AutoTrimAngle::AutoTrimAngle(ShooterActuator* a_actuator, bool shootingMode) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_actuator = a_actuator; 
  m_shootingMode = shootingMode; 
  AddRequirements(m_actuator);
  myTimer = new frc::Timer;
  
}

// Called when the command is initially scheduled.
void AutoTrimAngle::Initialize() {
  myTimer -> Reset();
  myTimer -> Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoTrimAngle::Execute() {
    curTime = myTimer -> Get();

  if (curTime <= 15 && m_shootingMode == true){
    m_actuator -> limeStream(2); 
      frc::SmartDashboard::PutNumber("Set RPM", 3950); 
    m_actuator->SetAutoAim(m_shootingMode); 

  } else {
    m_shootingMode = false;
  } 
}

// Called once the command ends or is interrupted.
void AutoTrimAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoTrimAngle::IsFinished() {
     curTime = myTimer -> Get();

  if (curTime>= 15) {
    return true;
  } else {
    return false;
  }
}
