// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autoTrimAngle.h"

autoTrimAngle::autoTrimAngle(ShooterActuator* a_actuator, bool shootingMode) {
  m_actuator = a_actuator;
  m_shootingMode = shootingMode;
  AddRequirements(m_actuator);
  myTimer = new frc::Timer;
  myTimer -> Reset();
  myTimer -> Start();
  myTimer -> Get();
  
}

// Called when the command is initially scheduled.
void autoTrimAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void autoTrimAngle::Execute() {
  currTime = myTimer -> Get();

  if (currTime <= 5 && m_shootingMode == true){
    m_actuator->SetAutoAim(m_shootingMode);
  } else {
    m_shootingMode = false;
  }
}

// Called once the command ends or is interrupted.
void autoTrimAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool autoTrimAngle::IsFinished() {
  currTime = myTimer -> Get();
  
  if (currTime > 5){
    return true;
  } else { 
    return false;
  }
}
