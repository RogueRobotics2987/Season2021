// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoPickup.h"

AutoPickup::AutoPickup(Intake* c_intake, bool c_run, double c_time){
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake = c_intake;
  m_run = c_run;
  m_time = c_time;
  m_timer = new frc::Timer();
}

// Called when the command is initially scheduled.
void AutoPickup::Initialize() {
  m_timer->Start();
  m_timer->Reset();
}

// Called repeatedly when this Command is scheduled to run
void AutoPickup::Execute() {

  if(m_run == true && m_timer->Get() <= m_time){
    m_intake->IntakeBall(0.65);
  }else{
    m_intake->IntakeBall(0.0);
  }

}

// Called once the command ends or is interrupted.
void AutoPickup::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoPickup::IsFinished() {
  if(m_timer->Get() >= m_time){
    return true;
  }else{
    return false;
  }
}
